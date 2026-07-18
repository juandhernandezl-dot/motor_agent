#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rl_motor_sarsa_lambda.py  -  ESTRATEGIA 2: SARSA(lambda) (TD, on-policy, trazas)
================================================================================

Value-based, ON-POLICY, con TRAZAS DE ELEGIBILIDAD (TD(lambda)).

Idea central
------------
SARSA aprende Q(s,a) siguiendo la MISMA politica que ejecuta: usa el valor de la
accion que REALMENTE se toma en el siguiente estado, no el maximo:

    delta = r + gamma * Q(s', a') - Q(s, a)        (a' = accion realmente tomada)

Las TRAZAS reparten ese error hacia atras, a los estados-accion recientes:

    e(s,a) <- e(s,a) + 1            (traza acumulativa en la celda visitada)
    Q      <- Q + alpha * delta * e (para todas las celdas)
    e      <- gamma * lambda * e    (decaimiento de todas las trazas)

Esto importa MUCHO en el motor: el efecto de subir el PWM no se ve en la muestra
siguiente sino varias despues (inercia, tau ~ decimas de segundo). Las trazas
propagan el credito a toda la rafaga de decisiones que causo la aceleracion, en
vez de aprender solo del ultimo paso.

Caracter: mas CONSERVADOR y SUAVE que Q-learning (al ser on-policy valora el
riesgo de su propia exploracion, asi que no se acerca al limite de rpm confiando
en frenar justo a tiempo) y converge antes gracias a las trazas. PWM mas limpio.

Uso
---
    python3 motor_bus.py --sim
    python3 rl_motor_sarsa_lambda.py
    python3 rl_motor_sarsa_lambda.py --reentrenar --acelerado --sp 1500
"""

import json
import os
import random

from rl_motor_common import (Discretizer, GridMixin, N_ACCIONES,
                             argmax_aleatorio, guardar_json_atomico, preparar)


# =============================================================================
# MANIFIESTO PARA LA GUI (ver controller_registry.py). Sin esto, el controlador
# no aparecia en el lanzador de motor_gui.py: solo se podia usar por consola.
# =============================================================================

CONTROLLER = {
    "clave": "sarsa_lambda",
    "nombre": "SARSA(lambda)",
    "familia": "RL",
    "kind": "q",
    "descripcion": ("TD on-policy con trazas de elegibilidad. Mas conservador y suave que Q-learning; las trazas ayudan con la inercia del motor."),
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

ALPHA = 0.15          # tasa de aprendizaje TD
GAMMA = 0.95            # factor de descuento
LAMBDA_ = 0.75          # decaimiento de trazas: 0=TD(0), 1=Monte Carlo
EPSILON_START = 1.0
EPSILON_END = 0.1
EPSILON_DECAY = 0.99985
EPSILON_ONLINE = 0.1
TRAZA_ACUMULATIVA = True   # True=acumulativa (e+=1); False=reemplazante (e=1)
TRAZA_MIN = 1e-3           # se descartan trazas por debajo de esto (eficiencia)

ERR_BINS = 27
DERR_BINS = 17

QTABLE_PATH = "qtable_motor_sarsa_lambda.json"


# =============================================================================
# AGENTE SARSA(lambda)
# =============================================================================

class SarsaLambdaAgent(GridMixin):
    nombre = "SARSA(lambda)"
    clave = "sarsa_lambda"
    kind = "q"

    def __init__(self, disc: Discretizer, alpha=ALPHA, gamma=GAMMA, lambda_=LAMBDA_,
                 epsilon_start=EPSILON_START, epsilon_end=EPSILON_END,
                 epsilon_decay=EPSILON_DECAY, epsilon_online=EPSILON_ONLINE,
                 traza_acumulativa=TRAZA_ACUMULATIVA, traza_min=TRAZA_MIN,
                 qtable_path=QTABLE_PATH):
        self.disc = disc
        self.alpha, self.gamma, self.lambda_ = alpha, gamma, lambda_
        self.epsilon = epsilon_start
        self.epsilon_end = epsilon_end
        self.epsilon_decay = epsilon_decay
        self.epsilon_online = epsilon_online
        self.traza_acumulativa = traza_acumulativa
        self.traza_min = traza_min
        self.qtable_path = qtable_path
        self.q = {}          # {(eb, db): [Q por accion]}
        self.e = {}          # trazas de elegibilidad, misma forma que q
        self.visitas = {}

    def _row(self, key):
        if key not in self.q:
            self.q[key] = [0.0] * N_ACCIONES
        return self.q[key]

    def valores(self, key):
        return self.q.get(key)

    # ---- interfaz del runner -------------------------------------------------
    def act(self, state, explore=False) -> int:
        row = self._row(self.disc.key(state))
        if explore and random.random() < self.epsilon:
            return random.randrange(N_ACCIONES)
        return argmax_aleatorio(row)   # desempate aleatorio (ver Q-learning)

    def observe(self, s, a, r, s2, a2, done):
        """
        SARSA(lambda) ON-POLICY: usa Q(s2, a2) con a2 = accion que el runner va a
        aplicar de verdad (por eso el runner elige a2 antes de llamar aqui).
        """
        key, key2 = self.disc.key(s), self.disc.key(s2)
        row, row2 = self._row(key), self._row(key2)
        q_sa = row[a]
        q_s2a2 = 0.0 if done else row2[a2]
        delta = r + self.gamma * q_s2a2 - q_sa
        self.visitas[key] = self.visitas.get(key, 0) + 1

        # Traza de la celda visitada (acumulativa o reemplazante).
        er = self.e.setdefault(key, [0.0] * N_ACCIONES)
        if self.traza_acumulativa:
            er[a] += 1.0
        else:
            er[a] = 1.0

        # Propaga el error TD segun la traza de cada celda y decae las trazas.
        # Optimizado: alpha*delta precalculado, maximo de la traza en la misma
        # pasada (evita un max() extra por celda) y acceso directo a self.q
        # (la celda existe seguro si tiene traza).
        gl = self.gamma * self.lambda_
        ad = self.alpha * delta
        traza_min = self.traza_min
        muertas = []
        for k, ev in self.e.items():
            qrow = self.q[k] if k in self.q else self._row(k)
            emax = 0.0
            for j in range(N_ACCIONES):
                e_j = ev[j]
                if e_j != 0.0:
                    qrow[j] += ad * e_j
                    e_j *= gl
                    ev[j] = e_j
                    if e_j > emax:
                        emax = e_j
            if emax < traza_min:
                muertas.append(k)
        for k in muertas:
            del self.e[k]

        if self.epsilon > self.epsilon_end:
            self.epsilon *= self.epsilon_decay
        if self.epsilon < self.epsilon_online:
            self.epsilon = self.epsilon_online

    def end_episode(self):
        pass    # el reinicio real de trazas ocurre en on_reset.

    def on_reset(self):
        self.e = {}   # las trazas no cruzan de un episodio al siguiente.

    # ---- persistencia --------------------------------------------------------
    def save(self, path=None):
        path = path or self.qtable_path
        data = {"q": {f"{k[0]},{k[1]}": v for k, v in self.q.items()},
                "visitas": {f"{k[0]},{k[1]}": v for k, v in self.visitas.items()},
                "n_acciones": N_ACCIONES,
                "bins": [self.disc.err_bins, self.disc.derr_bins]}
        guardar_json_atomico(path, data)   # nunca deja una tabla corrupta

    def load(self, path=None):
        path = path or self.qtable_path
        if not os.path.exists(path):
            return False
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if data.get("n_acciones", N_ACCIONES) != N_ACCIONES:
            print(f"  [{self.nombre}] La tabla guardada tiene otro numero de "
                  f"acciones; se ignora y se entrena de nuevo.")
            return False
        self.q = {}
        for k, v in (data.get("q") or {}).items():
            eb, db = k.split(",")
            self.q[(int(eb), int(db))] = v
        self.visitas = {}
        for k, v in (data.get("visitas") or {}).items():
            eb, db = k.split(",")
            self.visitas[(int(eb), int(db))] = v
        print(f"  [{self.nombre}] Tabla Q cargada de {path} ({len(self.q)} estados).")
        return True


# =============================================================================
# MAIN
# =============================================================================

def main():
    disc = Discretizer(err_bins=ERR_BINS, derr_bins=DERR_BINS)
    preparar(lambda: SarsaLambdaAgent(disc),
             "Control de velocidad por SARSA(lambda) (RL).", disc)


if __name__ == "__main__":
    main()
