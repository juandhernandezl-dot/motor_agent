#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rl_motor_reinforce.py  -  ESTRATEGIA 3: REINFORCE (policy gradient, Monte Carlo)
================================================================================

POLICY-BASED, Monte Carlo, con LINEA BASE y bonus de ENTROPIA. A diferencia de
Q-learning y SARSA (que aprenden VALORES y de ahi derivan la accion), REINFORCE
aprende DIRECTAMENTE una POLITICA ESTOCASTICA parametrizada, SIN discretizar el
estado en una tabla.

Idea central
------------
Softmax sobre las N acciones (incrementos de PWM), con preferencias lineales
sobre un vector de caracteristicas phi(s) del error de velocidad:

    h(s,a) = theta[a] . phi(s)      ;      pi(a|s) = softmax(h(s, .))

Tras cada EPISODIO se calcula el retorno descontado G_t y se empujan los pesos
en la direccion que hace MAS probables las acciones que llevaron a retornos
altos (respecto de una linea base b, que reduce la varianza):

    theta[a] += alpha * (G_t - b) * grad_theta[a] log pi(a_t | s_t)
    grad log pi(a|s) respecto de theta[c] = phi(s) * ( 1[a=c] - pi(c|s) )

El termino de ENTROPIA (beta) evita que la politica colapse demasiado pronto.

Ventaja aqui: como NO discretiza, generaliza entre errores parecidos y da una
respuesta SUAVE y continua; el mismo entrenamiento sirve para consignas que
nunca vio. Inconveniente: Monte Carlo -> aprende mas lento y con mas varianza.

ESTABILIDAD (importante en el motor)
------------------------------------
El gradiente MC puede disparar los pesos si un episodio sale muy raro. Aqui se
NORMALIZAN las ventajas por su desviacion tipica y se recorta la norma del
gradiente (GRAD_CLIP): sin eso los theta crecen sin freno y la politica se queda
saturada en una sola accion.

Uso
---
    python3 motor_bus.py --sim
    python3 rl_motor_reinforce.py --reentrenar --acelerado --sp 1500
"""

import json
import math
import os
import random

from rl_motor_common import (Discretizer, ACCIONES, N_ACCIONES, DERR_MAX,
                             RPM_MAX, guardar_json_atomico, hacer_estado,
                             preparar)


# =============================================================================
# MANIFIESTO PARA LA GUI (ver controller_registry.py). Sin esto, el controlador
# no aparecia en el lanzador de motor_gui.py: solo se podia usar por consola.
# =============================================================================

CONTROLLER = {
    "clave": "reinforce",
    "nombre": "REINFORCE",
    "familia": "RL",
    "kind": "policy",
    "descripcion": ("Policy gradient Monte Carlo con linea base y entropia. No discretiza: respuesta suave y generaliza entre consignas; aprende mas lento."),
    "params": [
        {"tipo": "choice", "etiqueta": "Modo", "default": "",
         "opciones": [["Preguntar (menu en consola)", ""],
                      ["Controlar con lo aprendido", "--controlar"],
                      ["Entrenar (continuar)", "--entrenar"],
                      ["Reentrenar desde cero", "--reentrenar"]]},
        {"tipo": "choice", "etiqueta": "Ritmo", "default": "",
         "opciones": [["Preguntar (menu en consola)", ""],
                      ["Acelerado (pre-entrena en el modelo)", "--acelerado"],
                      ["Directo (solo motor real)", "--directo"]]},
        {"tipo": "float", "etiqueta": "Consigna (rpm)", "flag": "--sp",
         "default": None, "ayuda": "vacio = la de la GUI"},
    ],
}


# =============================================================================
# CONFIG (edita aqui)
# =============================================================================

ALPHA = 0.01            # tasa de aprendizaje de la politica
GAMMA = 0.95            # factor de descuento
ENTROPY_BETA = 0.01     # peso del bonus de entropia (exploracion)
BASELINE_LR = 0.05      # rapidez con que la linea base sigue al retorno medio
GRAD_CLIP = 5.0         # recorte de |coef| por paso (estabilidad)
NORMALIZAR_VENTAJA = True   # dividir (G-b) por su desviacion tipica

# Rejilla usada SOLO para dibujar la politica en la GUI (el agente no la usa).
ERR_BINS = 25
DERR_BINS = 13

POLICY_PATH = "policy_motor_reinforce.json"


# =============================================================================
# CARACTERISTICAS (features) del estado
# =============================================================================

def features(state):
    """
    phi(s) compacto y NORMALIZADO (clave: sin normalizar, los rpm en miles
    saturan el softmax en el primer paso).
      1        termino independiente (sesgo)
      en       error normalizado        -> "hacia donde y cuanto"
      en^2     magnitud del error       -> permite frenar fuerte lejos
      raiz(en) con signo                -> sensibilidad fina cerca de cero
      den      variacion del error      -> amortiguamiento (parte derivativa)
      den^2
      pwm_n    salida actual            -> conoce si le queda margen arriba/abajo
    """
    en = max(-1.0, min(1.0, state["err"] / RPM_MAX))
    den = max(-1.0, min(1.0, state["derr"] / DERR_MAX))
    raiz = math.copysign(math.sqrt(abs(en)), en)
    return [1.0, en, en * en, raiz, den, den * den, state["pwm"] / 255.0]


NF = len(features({"err": 0.0, "derr": 0.0, "pwm": 0}))


# =============================================================================
# AGENTE REINFORCE
# =============================================================================

class ReinforceAgent:
    nombre = "REINFORCE"
    clave = "reinforce"
    kind = "policy"

    def __init__(self, disc: Discretizer = None, alpha=ALPHA, gamma=GAMMA,
                 entropy_beta=ENTROPY_BETA, baseline_lr=BASELINE_LR,
                 grad_clip=GRAD_CLIP, policy_path=POLICY_PATH):
        self.disc = disc            # solo para dibujar
        self.alpha, self.gamma = alpha, gamma
        self.entropy_beta = entropy_beta
        self.baseline_lr = baseline_lr
        self.grad_clip = grad_clip
        self.policy_path = policy_path
        # theta[a] = vector de pesos de la accion a.
        self.theta = [[0.0] * NF for _ in range(N_ACCIONES)]
        self.baseline = 0.0
        self._traj = []             # [(phi, a, r), ...] del episodio en curso

    # ---- politica softmax ----------------------------------------------------
    def _logits(self, phi):
        return [sum(w * x for w, x in zip(th, phi)) for th in self.theta]

    def _policy(self, phi):
        z = self._logits(phi)
        mz = max(z)
        ex = [math.exp(zi - mz) for zi in z]     # estable numericamente
        s = sum(ex)
        return [e / s for e in ex]

    # ---- interfaz del runner -------------------------------------------------
    def act(self, state, explore=False) -> int:
        p = self._policy(features(state))
        if explore:
            u, acc = random.random(), 0.0        # muestreo estocastico
            for i, pi in enumerate(p):
                acc += pi
                if u <= acc:
                    return i
            return N_ACCIONES - 1
        return max(range(N_ACCIONES), key=lambda i: p[i])   # greedy

    def observe(self, s, a, r, s2, a2, done):
        """Solo acumula: REINFORCE aprende al cerrar el episodio."""
        self._traj.append((features(s), a, r))

    def end_episode(self):
        """Gradiente de Monte Carlo sobre el episodio completo."""
        if not self._traj:
            return
        # Retornos descontados G_t, hacia atras.
        G = 0.0
        returns = [0.0] * len(self._traj)
        for t in range(len(self._traj) - 1, -1, -1):
            G = self._traj[t][2] + self.gamma * G
            returns[t] = G

        # Linea base: media movil de los retornos (reduce varianza).
        media_ep = sum(returns) / len(returns)
        self.baseline += self.baseline_lr * (media_ep - self.baseline)

        # Escala de la ventaja (evita que un episodio raro dispare los pesos).
        sigma = 1.0
        if NORMALIZAR_VENTAJA and len(returns) > 1:
            var = sum((g - media_ep) ** 2 for g in returns) / len(returns)
            sigma = max(1e-3, math.sqrt(var))

        alpha, beta, clip = self.alpha, self.entropy_beta, self.grad_clip
        for (phi, a, _), Gt in zip(self._traj, returns):
            av = alpha * (Gt - self.baseline) / sigma   # alpha*ventaja, 1 vez
            p = self._policy(phi)
            logp = [math.log(pi + 1e-12) for pi in p]   # se reutiliza abajo
            H = -sum(pi * lp for pi, lp in zip(p, logp))
            for c in range(N_ACCIONES):
                grad_logpi = (1.0 if a == c else 0.0) - p[c]
                # Bonus de entropia: dH/dtheta_c = -phi * p[c] * (log p[c] + H).
                coef = av * grad_logpi - alpha * beta * p[c] * (logp[c] + H)
                if coef > clip:
                    coef = clip
                elif coef < -clip:
                    coef = -clip
                if coef != 0.0:
                    th = self.theta[c]
                    for i in range(NF):
                        th[i] += coef * phi[i]
        self._traj = []

    def on_reset(self):
        self._traj = []

    # ---- instantanea para la GUI ---------------------------------------------
    def valores(self, key):
        """Probabilidades pi(.|s) en el centro de la celda (para el mapa)."""
        if self.disc is None:
            return None
        err, derr = self.disc.centro(*key)
        s = hacer_estado(0.0, err, 0.0, err - derr, 128)   # pwm medio de referencia
        return self._policy(features(s))

    def grid(self, disc):
        val, act = [], []
        for db in range(disc.derr_bins):
            fv, fa = [], []
            for eb in range(disc.err_bins):
                p = self.valores((eb, db))
                fv.append(max(p))                                  # confianza 0..1
                fa.append(max(range(N_ACCIONES), key=lambda i: p[i]))
            val.append(fv); act.append(fa)
        return {"algo": self.nombre, "kind": self.kind,
                "err_bins": disc.err_bins, "derr_bins": disc.derr_bins,
                "err_edges": [round(x, 1) for x in disc.bordes_err()],
                "derr_edges": [round(x, 1) for x in disc.bordes_derr()],
                "acciones": list(ACCIONES),
                "value": val, "action": act,
                "visits": [[1] * disc.err_bins for _ in range(disc.derr_bins)]}

    # ---- persistencia --------------------------------------------------------
    def save(self, path=None):
        path = path or self.policy_path
        data = {"theta": self.theta, "baseline": self.baseline,
                "nf": NF, "n_acciones": N_ACCIONES}
        guardar_json_atomico(path, data)   # nunca deja la politica corrupta

    def load(self, path=None):
        path = path or self.policy_path
        if not os.path.exists(path):
            return False
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if data.get("nf") != NF or data.get("n_acciones") != N_ACCIONES:
            print(f"  [{self.nombre}] El archivo tiene otra dimension de features "
                  f"o de acciones; se ignora y se entrena de nuevo.")
            return False
        self.theta = data["theta"]
        self.baseline = data.get("baseline", 0.0)
        print(f"  [{self.nombre}] Politica cargada de {path} "
              f"(baseline={self.baseline:.2f}).")
        return True


# =============================================================================
# MAIN
# =============================================================================

def main():
    disc = Discretizer(err_bins=ERR_BINS, derr_bins=DERR_BINS)
    preparar(lambda: ReinforceAgent(disc),
             "Control de velocidad por REINFORCE (policy gradient).", disc)


if __name__ == "__main__":
    main()
