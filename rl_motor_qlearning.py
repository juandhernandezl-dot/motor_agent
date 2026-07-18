#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rl_motor_qlearning.py  -  ESTRATEGIA 1: Q-LEARNING (TD, off-policy, tabular)
============================================================================

Value-based, off-policy, diferencia temporal (TD) de un paso.

Idea central
------------
Aprende Q(s,a) = retorno esperado de aplicar el incremento de PWM 'a' en el
estado s = (error de velocidad, variacion del error) y luego actuar
OPTIMAMENTE. La actualizacion usa el MEJOR valor del siguiente estado, sin
importar que accion se tome de verdad (por eso es OFF-POLICY):

    Q(s,a) <- Q(s,a) + alpha * [ r + gamma * max_a' Q(s',a') - Q(s,a) ]

Exploracion: epsilon-greedy decreciente durante el pre-entrenamiento; en tiempo
real se mantiene un minimo (epsilon_online) para seguir adaptandose a la carga.

Caracter en el motor: agresivo y eficiente en datos; llega rapido a una
respuesta veloz, pero tiende a un PWM mas nervioso (mas microcorrecciones) que
SARSA, porque persigue el optimo aunque el camino sea ruidoso.

Uso
---
    python3 motor_bus.py --sim             # 1) bus (o --serial /dev/ttyACM0)
    python3 motor_gui.py                   # 2) GUI: consigna + ver la tabla
    python3 rl_motor_qlearning.py          # 3) entrena (si hace falta) y controla
    python3 rl_motor_qlearning.py --reentrenar --acelerado --sp 1500
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
    "clave": "qlearning",
    "nombre": "Q-learning",
    "familia": "RL",
    "kind": "q",
    "descripcion": ("TD off-policy tabular. Agresivo y eficiente en datos: respuesta rapida pero PWM algo mas nervioso que SARSA."),
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

ALPHA = 0.20            # tasa de aprendizaje TD
GAMMA = 0.95            # factor de descuento (cuanto pesa el futuro)
EPSILON_START = 1.0     # exploracion inicial (pre-entrenamiento)
EPSILON_END = 0.05      # exploracion minima al final del decaimiento
EPSILON_DECAY = 0.99985  # factor multiplicativo por paso
EPSILON_ONLINE = 0.05   # exploracion minima persistente en tiempo real

ERR_BINS = 25           # resolucion del error de velocidad en la tabla
DERR_BINS = 13          # resolucion de la variacion del error

QTABLE_PATH = "qtable_motor_qlearning.json"


# =============================================================================
# AGENTE Q-LEARNING
# =============================================================================

class QLearningAgent(GridMixin):
    nombre = "Q-learning"
    clave = "qlearning"          # nombre con el que toma el mando en el bus
    kind = "q"

    def __init__(self, disc: Discretizer, alpha=ALPHA, gamma=GAMMA,
                 epsilon_start=EPSILON_START, epsilon_end=EPSILON_END,
                 epsilon_decay=EPSILON_DECAY, epsilon_online=EPSILON_ONLINE,
                 qtable_path=QTABLE_PATH):
        self.disc = disc
        self.alpha, self.gamma = alpha, gamma
        self.epsilon = epsilon_start
        self.epsilon_end = epsilon_end
        self.epsilon_decay = epsilon_decay
        self.epsilon_online = epsilon_online
        self.qtable_path = qtable_path
        self.q = {}          # {(eb, db): [Q por cada accion]}
        self.visitas = {}    # {(eb, db): n} solo informativo (GUI)

    def _row(self, key):
        if key not in self.q:
            self.q[key] = [0.0] * N_ACCIONES
        return self.q[key]

    def valores(self, key):
        return self.q.get(key)          # None si nunca se visito -> celda gris

    # ---- interfaz del runner -------------------------------------------------
    def act(self, state, explore=False) -> int:
        row = self._row(self.disc.key(state))
        if explore and random.random() < self.epsilon:
            return random.randrange(N_ACCIONES)
        # Desempate aleatorio: sin el, toda celda nueva elegiria ACCIONES[0]
        # (el frenazo de -60) por puro orden de indices.
        return argmax_aleatorio(row)

    def observe(self, s, a, r, s2, a2, done):
        """Actualizacion Q-learning (off-policy: usa max_a' Q(s2,a'))."""
        key = self.disc.key(s)
        row = self._row(key)
        row2 = self._row(self.disc.key(s2))
        td_target = r + (0.0 if done else self.gamma * max(row2))
        row[a] += self.alpha * (td_target - row[a])
        self.visitas[key] = self.visitas.get(key, 0) + 1
        # Decaimiento de epsilon, con piso 'epsilon_online'.
        if self.epsilon > self.epsilon_end:
            self.epsilon *= self.epsilon_decay
        if self.epsilon < self.epsilon_online:
            self.epsilon = self.epsilon_online

    def end_episode(self):
        pass    # Q-learning aprende paso a paso; no necesita cierre de episodio.

    def on_reset(self):
        pass    # no hay estado por-episodio que reiniciar.

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
        raw = data.get("q", {})
        self.q = {}
        for k, v in raw.items():
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
    preparar(lambda: QLearningAgent(disc),
             "Control de velocidad por Q-learning (RL).", disc)


if __name__ == "__main__":
    main()
