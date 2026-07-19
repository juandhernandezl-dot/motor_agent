#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rl_motor_common.py
==================

INFRAESTRUCTURA COMPARTIDA de los tres controladores por aprendizaje por
refuerzo del MOTOR (equivalente a rl_common.py del vermicompostaje).

Cambia la planta, NO la arquitectura: el agente sigue cumpliendo exactamente la
misma interfaz minima, asi que rl_motor_qlearning.py / rl_motor_sarsa_lambda.py /
rl_motor_reinforce.py son casi copia de los originales.

QUE CAMBIA RESPECTO AL VERMICOMPOSTAJE
--------------------------------------
  humedad/temperatura  ->  error de velocidad (e) y su variacion (de)
  valvula {abrir,cerrar} (2 acciones) -> INCREMENTO de PWM (7 acciones)
  banda de humedad     ->  banda de tolerancia alrededor de la consigna
  cliente del simulador -> cliente del BUS (motor_client.BusClient)

POR QUE ACCIONES INCREMENTALES (du) Y NO PWM ABSOLUTO
----------------------------------------------------
El PWM que sostiene 1500 rpm depende de la carga, la bateria y la friccion: si
el agente aprendiera "consigna 1500 -> PWM 137" quedaria invalidado al primer
cambio de carga. Aprendiendo el INCREMENTO (u += du) el agente adquiere accion
integral: el estado (e, de) basta y la tabla se mantiene valida aunque cambie el
punto de operacion. Por eso la tabla es 2D y se puede dibujar como mapa de calor.

INTERFAZ QUE DEBE CUMPLIR UN AGENTE (identica a la de rl_common.py)
    act(state, explore) -> int            # indice en ACCIONES
    observe(s, a, r, s2, a2, done)
    end_episode()  /  on_reset()
    save(path) / load(path) -> bool
    grid(disc) -> dict                    # instantanea para la GUI
    nombre : str

USO DIRECTO (identificacion de la planta para pre-entrenar):
    python3 rl_motor_common.py --identificar
"""

import argparse
import json
import math
import os
import random
import time
from collections import deque
from dataclasses import dataclass

from motor_bus import MotorPlant, RPM_FS
from motor_client import BusClient, conectar_o_avisar

# =============================================================================
# PLANTA, ACCIONES Y OBJETIVOS POR DEFECTO
# =============================================================================

RPM_MAX = RPM_FS               # fondo de escala del tacometro (5 V)
RPM_LIMITE = 0.95 * RPM_FS     # interlock local del agente (el bus tiene el suyo)
SP_MIN, SP_MAX = 300.0, 0.80 * RPM_FS   # consignas usadas al entrenar

TOLERANCIA = 40.0              # +/- rpm que se consideran "en banda" (objetivo)
DERR_MAX = 300.0               # variacion de error por muestra tipica (para escalar)

# Accion = incremento de PWM (cuentas de 0..255) aplicado a la salida actual.
# Escala geometrica: pasos finos para afinar y gruesos para arrancar/frenar.
ACCIONES = (-30, -10, -5, -2, 0, +2, +5, +10, +30)  # PWM increment (du) applied to the current output
N_ACCIONES = len(ACCIONES)
DU_MAX = max(abs(a) for a in ACCIONES)

# --- recompensa (mismo espiritu que en el vermicompostaje) -------------------
PESO_EXCESO = 20.0          # pasarse de vueltas es peor que quedarse corto
PESO_EXCESO_LIN = 30.0       # ... y hasta los rebases pequenos deben doler
PESO_DEFECTO = 15.0
PENAL_ZONA_RIESGO = 10.0    # castigo extra por pisar el limite de seguridad
PENAL_ESFUERZO = 0.3   # castigo por mover mucho el PWM (suavidad, desgaste).
                       # 0 = desactivado. Un valor pequenno (0.2-0.5) reduce el
                       # "nerviosismo" del PWM en regimen sin frenar la subida.

PLANT_PATH = "plant_motor.json"


# =============================================================================
# UTILIDADES COMPARTIDAS
# =============================================================================

def argmax_aleatorio(vals):
    """
    Indice del maximo, rompiendo EMPATES AL AZAR.

    Importante: con Q inicializada a 0, el clasico max(range(n), key=...)
    devuelve SIEMPRE el indice 0 en celdas nuevas o empatadas, y ACCIONES[0]
    es el frenazo de -60 PWM. Ese sesgo sistematico hacia "frenar a tope"
    ensucia el arranque del aprendizaje; con desempate aleatorio desaparece.
    """
    mejor = max(vals)
    cand = [i for i, v in enumerate(vals) if v == mejor]
    return cand[0] if len(cand) == 1 else random.choice(cand)


def guardar_json_atomico(ruta, data):
    """
    Escritura ATOMICA: se vuelca a un temporal y se renombra con os.replace.
    Sin esto, un Ctrl+C (o un cuelgue) a mitad de json.dump deja la tabla
    corrupta y se pierde todo el entrenamiento. os.replace es atomico en
    POSIX y en Windows.
    """
    tmp = ruta + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, separators=(",", ":"))
    os.replace(tmp, ruta)


# =============================================================================
# ESTADO Y DISCRETIZACION
# =============================================================================

def hacer_estado(rpm, sp, rpm_prev, sp_prev, pwm, t=0.0):
    """
    Estado del agente. Contiene lo minimo con sentido fisico:
      err  = consigna - medida          (cuanto falta)
      derr = err - err_anterior         (hacia donde va: da amortiguamiento)
      pwm  = salida actual              (no entra en la tabla; sirve al limitar)
    """
    err = sp - rpm
    derr = err - (sp_prev - rpm_prev)
    return {"rpm": float(rpm), "sp": float(sp), "err": float(err),
            "derr": float(derr), "pwm": int(pwm), "t": float(t)}


class Discretizer:
    """
    (err, derr) continuos -> celda (eb, db).

    Rejilla NO uniforme: se aplica una deformacion sign(x)*sqrt(|x|/xmax) antes
    de dividir en bins iguales. Resultado: MUCHA resolucion cerca de cero (donde
    hay que afinar para quedarse en la banda de +/-40 rpm) y poca en los
    extremos (donde da igual el detalle: hay que acelerar/frenar a tope).
    Los valores fuera de rango caen en las celdas del borde (clamping).
    """

    def __init__(self, err_max=RPM_MAX, err_bins=25,
                 derr_max=DERR_MAX, derr_bins=13, warp=0.5):
        self.err_max, self.err_bins = err_max, err_bins
        self.derr_max, self.derr_bins = derr_max, derr_bins
        self.warp = warp

    def _w(self, x, xmax):
        """[-xmax, xmax] -> [-1, 1] con mas detalle cerca de 0."""
        u = max(-1.0, min(1.0, x / xmax))
        return math.copysign(abs(u) ** self.warp, u)

    def _wi(self, y, xmax):
        """Inversa de _w (para saber que rpm representa cada celda)."""
        return math.copysign(abs(y) ** (1.0 / self.warp), y) * xmax

    @staticmethod
    def _bin(y, nbins):
        i = int((y + 1.0) * 0.5 * nbins)
        return max(0, min(nbins - 1, i))

    def key(self, state):
        eb = self._bin(self._w(state["err"], self.err_max), self.err_bins)
        db = self._bin(self._w(state["derr"], self.derr_max), self.derr_bins)
        return (eb, db)

    def centro(self, eb, db):
        """Valores (err, derr) representativos del centro de la celda."""
        ye = (eb + 0.5) / self.err_bins * 2.0 - 1.0
        yd = (db + 0.5) / self.derr_bins * 2.0 - 1.0
        return self._wi(ye, self.err_max), self._wi(yd, self.derr_max)

    def bordes_err(self):
        return [self._wi(i / self.err_bins * 2.0 - 1.0, self.err_max)
                for i in range(self.err_bins + 1)]

    def bordes_derr(self):
        return [self._wi(i / self.derr_bins * 2.0 - 1.0, self.derr_max)
                for i in range(self.derr_bins + 1)]

    @property
    def n_states(self):
        return self.err_bins * self.derr_bins


# =============================================================================
# RECOMPENSA, BANDA Y SEGURIDAD
# =============================================================================

def en_banda(state, tol=TOLERANCIA) -> bool:
    return abs(state["err"]) <= tol


def reward(state_next, du, tol=TOLERANCIA, rpm_lim=RPM_LIMITE,
           peso_exceso=PESO_EXCESO, peso_exceso_lin=PESO_EXCESO_LIN,
           peso_defecto=PESO_DEFECTO, penal_riesgo=PENAL_ZONA_RIESGO,
           penal_esfuerzo=PENAL_ESFUERZO, escala=RPM_MAX) -> float:
    """
    Recompensa unica de los tres controladores (misma forma que la del
    vermicompostaje, con velocidad en lugar de humedad):

      +1                                    si |e| <= tol   (dentro de banda)
      -peso_defecto*(e/escala)^2            si falta velocidad (castigo suave)
      -peso_exceso*(x)^2 - peso_exceso_lin*x  si sobra velocidad (x = exceso
                                            normalizado): cuadratico + LINEAL.
      -penal_riesgo                         si se pisa el limite de seguridad.
      -penal_esfuerzo*|du|/DU_MAX           por mover el PWM (suavidad).

    Asimetria a proposito: pasarse de vueltas castiga a la mecanica y cuesta
    frenar; quedarse corto solo pierde rendimiento.
    """
    e = state_next["err"]                 # >0 falta velocidad, <0 sobra
    if abs(e) <= tol:
        r = 1.0
    elif e > 0:
        r = -((e / escala) ** 2) * peso_defecto
    else:
        over = -e / escala
        r = -(over * over * peso_exceso + over * peso_exceso_lin)
    if state_next["rpm"] >= rpm_lim:
        r -= penal_riesgo
    r -= penal_esfuerzo * abs(du) / DU_MAX
    return r


# =============================================================================
# METRICAS EN VENTANA MOVIL
# =============================================================================

class MetricasMoviles:
    """% de tiempo en banda, % en zona de riesgo y RMSE sobre ventana movil."""

    def __init__(self, ventana=100, tol=TOLERANCIA, rpm_lim=RPM_LIMITE):
        self.banda = deque(maxlen=ventana)
        self.sat = deque(maxlen=ventana)
        self.err2 = deque(maxlen=ventana)
        self.tol, self.rpm_lim = tol, rpm_lim

    def push(self, state):
        self.banda.append(1 if abs(state["err"]) <= self.tol else 0)
        self.sat.append(1 if state["rpm"] >= self.rpm_lim else 0)
        self.err2.append(state["err"] ** 2)

    @property
    def pct_banda(self):
        return 100.0 * sum(self.banda) / len(self.banda) if self.banda else float("nan")

    @property
    def pct_sat(self):
        return 100.0 * sum(self.sat) / len(self.sat) if self.sat else float("nan")

    @property
    def rmse(self):
        return math.sqrt(sum(self.err2) / len(self.err2)) if self.err2 else float("nan")

    @property
    def n(self):
        return len(self.banda)


# =============================================================================
# INSTANTANEA DE LA TABLA PARA LA GUI (formato unico para los 3 algoritmos)
# =============================================================================

class GridMixin:
    """
    Convierte la tabla/politica en una rejilla uniforme que la GUI dibuja igual
    para los tres algoritmos:
        value[db][eb]  -> escalar (max_a Q, o max_a pi para REINFORCE)
        action[db][eb] -> indice de la accion greedy (o None si no visitado)
        visits[db][eb] -> veces que se ha actualizado la celda
    """
    def valores(self, key):
        """Devuelve la lista de N_ACCIONES valores de la celda, o None."""
        raise NotImplementedError

    def grid(self, disc):
        val, act, vis = [], [], []
        for db in range(disc.derr_bins):
            fv, fa, fn = [], [], []
            for eb in range(disc.err_bins):
                v = self.valores((eb, db))
                if v is None:
                    fv.append(None); fa.append(None); fn.append(0)
                else:
                    fv.append(max(v))
                    fa.append(max(range(len(v)), key=lambda i: v[i]))
                    fn.append(int(getattr(self, "visitas", {}).get((eb, db), 0)))
            val.append(fv); act.append(fa); vis.append(fn)
        return {
            "algo": self.nombre, "kind": getattr(self, "kind", "q"),
            "err_bins": disc.err_bins, "derr_bins": disc.derr_bins,
            "err_edges": [round(x, 1) for x in disc.bordes_err()],
            "derr_edges": [round(x, 1) for x in disc.bordes_derr()],
            "acciones": list(ACCIONES),
            "value": val, "action": act, "visits": vis,
        }


# =============================================================================
# CONFIGURACION DEL LAZO
# =============================================================================

@dataclass
class RunConfig:
    # --- conexion ---
    host: str = "127.0.0.1"
    port: int = 8770
    # --- objetivo y seguridad ---
    tolerancia: float = TOLERANCIA
    rpm_limite: float = RPM_LIMITE
    consigna_defecto: float = 1200.0
    consigna_desde_bus: bool = True   # tomar la consigna del topico 'setpoint'
                                      # (la mueve la GUI) EN TIEMPO REAL
    # --- pre-entrenamiento acelerado (sobre el modelo de planta) ---
    usar_modelo: bool = True          # si hay plant_motor.json, se usa
    episodios_min: int = 40
    episodios_max: int = 10000
    evaluar_cada: int = 100
    pasos_por_episodio: int = 100
    objetivo_pct: float = 0.9        # % en banda para dar por convergido
    pasos_evaluacion: int = 100
    # --- tiempo real ---
    mini_episodio: int = 5          # pasos por "episodio" online (para MC)
    guardar_cada: int = 100         # antes 5: volcar TODA la tabla a disco 4
                                    # veces/segundo (Ts=50ms) es I/O inutil y
                                    # acelera el desgaste si hay SD/eMMC. El
                                    # runner ya guarda siempre al salir.
    publicar_cada: int = 5           # cada cuantas decisiones se publica la tabla
    ventana_metrica: int = 25
    imprimir_cada: int = 5
    # --- recompensa ---
    peso_exceso: float = PESO_EXCESO
    peso_exceso_lin: float = PESO_EXCESO_LIN
    peso_defecto: float = PESO_DEFECTO
    penal_riesgo: float = PENAL_ZONA_RIESGO
    penal_esfuerzo: float = PENAL_ESFUERZO


# =============================================================================
# IDENTIFICACION DE LA PLANTA (para pre-entrenar sin machacar el motor)
# =============================================================================

def identificar_planta(bus: BusClient, ruta=PLANT_PATH, hold_s=1.5, verbose=True):
    """
    Ensayo automatico sobre el motor REAL:
      1) ESCALERA de PWM (0,32,...,255) -> curva estatica rpm(duty).
         Ajuste por minimos cuadrados -> ganancia K y ZONA MUERTA.
      2) ESCALON 0 -> 60% -> constante de tiempo tau (tiempo al 63% del final).
    Guarda {K, tau, zona_muerta} en plant_motor.json. El pre-entrenamiento
    acelerado usa ese modelo; luego el agente afina contra el motor real.
    Dura ~25 s y deja el motor parado.
    """
    def media_final(pwm, seg):
        bus.set_pwm(pwm)
        t0 = time.time()
        vals = []
        while time.time() - t0 < seg:
            m = bus.next_sample()
            if time.time() - t0 > seg * 0.66:
                vals.append(m["rpm"])
        return sum(vals) / len(vals) if vals else 0.0

    bus.tomar_mando()
    bus.set_enable(True)
    bus.set_pwm(0)
    bus.drain()
    if verbose:
        print("[ident] escalera estatica ...")
    puntos = []
    for pwm in range(0, 256, 32):
        rpm = media_final(pwm, hold_s)
        puntos.append((pwm / 255.0, rpm))
        if verbose:
            print(f"   pwm={pwm:3d} ({pwm/2.55:5.1f}%)  ->  {rpm:7.1f} rpm")

    # Regresion rpm = a*duty + b usando solo los puntos ya en movimiento.
    ptos = [(d, r) for d, r in puntos if r > 0.05 * RPM_MAX]
    if len(ptos) < 3:
        print("[ident] el motor casi no se movio: revisa driver/tacometro.")
        bus.set_pwm(0); bus.set_enable(False); bus.soltar_mando()
        return None
    n = len(ptos)
    sx = sum(d for d, _ in ptos); sy = sum(r for _, r in ptos)
    sxx = sum(d * d for d, _ in ptos); sxy = sum(d * r for d, r in ptos)
    den = n * sxx - sx * sx
    a = (n * sxy - sx * sy) / den
    b = (sy - a * sx) / n
    zm = max(0.0, min(0.6, -b / a)) if a > 0 else 0.1
    K = a * (1.0 - zm)

    if verbose:
        print(f"[ident] K={K:.0f} rpm  zona_muerta={zm*100:.1f}% de duty")
        print("[ident] escalon 0 -> 60% para tau ...")
    bus.set_pwm(0)
    time.sleep(1.0)
    bus.drain()
    t0 = time.time()
    bus.set_pwm(153)
    traza = []
    while time.time() - t0 < 3.0:
        m = bus.next_sample()
        traza.append((time.time() - t0, m["rpm"]))
    final = sum(r for _, r in traza[-8:]) / max(1, len(traza[-8:]))
    objetivo = 0.632 * final
    tau = 0.35
    for t, r in traza:
        if r >= objetivo:
            tau = max(0.02, t)
            break
    bus.set_pwm(0)
    bus.set_enable(False)
    bus.soltar_mando()

    datos = {"K": round(K, 1), "tau": round(tau, 3), "zona_muerta": round(zm, 3),
             "rpm_ss_60pct": round(final, 1), "curva": puntos, "fecha": time.time()}
    with open(ruta, "w", encoding="utf-8") as f:
        json.dump(datos, f, indent=2)
    if verbose:
        print(f"[ident] tau={tau:.2f} s  (rpm a 60% = {final:.0f})")
        print(f"[ident] modelo guardado en {ruta}")
    return datos


def cargar_planta(ruta=PLANT_PATH, verbose=True):
    """Modelo para el pre-entrenamiento: el identificado si existe, o uno tipico."""
    if os.path.exists(ruta):
        try:
            with open(ruta, "r", encoding="utf-8") as f:
                d = json.load(f)
            if verbose:
                print(f"  [modelo] {ruta}: K={d['K']:.0f} tau={d['tau']:.2f}s "
                      f"zm={d['zona_muerta']*100:.0f}%")
            return MotorPlant(K=d["K"], tau=d["tau"], zona_muerta=d["zona_muerta"],
                              rpm_max=RPM_MAX)
        except (KeyError, ValueError) as e:
            print(f"  [modelo] {ruta} ilegible ({e}); se usa uno generico.")
    if verbose:
        print("  [modelo] sin identificacion previa: se usa un motor generico. "
              "Ejecuta 'python3 rl_motor_common.py --identificar' para afinarlo.")
    return MotorPlant(rpm_max=RPM_MAX)


# =============================================================================
# RUNNER DE DOS FASES (pre-entrenamiento acelerado + tiempo real)
# =============================================================================

class MotorRunner:
    """
    Orquesta el ciclo de vida del control, independiente del algoritmo.

    FASE 1 (opcional, acelerada): entrena contra el MODELO de la planta a la
      MISMA cadencia Ts que usara de verdad, pero tan rapido como pueda la CPU.
      Miles de episodios en segundos y sin castigar el motor.
    FASE 2 (tiempo real): se conecta al bus, toma el mando y decide UNA vez por
      cada muestra que llega del Arduino (sincronizacion por 'next_sample()').
      Sigue aprendiendo, asi que se adapta a la carga real.

    El runner aplica el INTERLOCK y calcula la RECOMPENSA; el agente solo aporta
    su regla de aprendizaje.
    """

    def __init__(self, bus: BusClient, agent, cfg: RunConfig, disc: Discretizer = None):
        self.bus, self.agent, self.cfg = bus, agent, cfg
        self.disc = disc
        self.ts = float(bus.info.get("ts_ms", 50)) / 1000.0
        self.pwm = 0
        self._n = 0
        self._sin_mando = False

    # ---- utilidades ----------------------------------------------------------
    def _aplicar(self, pwm_actual, a):
        """Convierte el indice de accion en un PWM 0..255 ya saturado."""
        return max(0, min(255, pwm_actual + ACCIONES[a]))

    def _safe_action(self, state, a):
        """Interlock: pisando el limite de velocidad se fuerza el mayor frenado."""
        if state["rpm"] >= self.cfg.rpm_limite:
            return 0                      # ACCIONES[0] es el decremento mayor
        return a

    def _set_pwm(self, pwm):
        """Aplica el PWM tolerando que la GUI nos haya quitado el mando: en ese
        caso el agente SIGUE aprendiendo (observa lo que hace el humano) pero no
        actua. Al devolverle el mando retoma sin reiniciar nada."""
        try:
            self.bus.set_pwm(pwm)
            if self._sin_mando:
                self._sin_mando = False
                print("  [mando] recuperado: el agente vuelve a actuar.")
            return True
        except Exception as e:
            if not self._sin_mando:
                self._sin_mando = True
                print(f"  [mando] sin control ({e}); se sigue observando.")
            return False

    def _reward(self, s2, du):
        c = self.cfg
        return reward(s2, du, tol=c.tolerancia, rpm_lim=c.rpm_limite,
                      peso_exceso=c.peso_exceso, peso_exceso_lin=c.peso_exceso_lin,
                      peso_defecto=c.peso_defecto, penal_riesgo=c.penal_riesgo,
                      penal_esfuerzo=c.penal_esfuerzo)

    def _publicar_tabla(self, forzar=False):
        if self.disc is None or not hasattr(self.agent, "grid"):
            return
        if not forzar and self._n % self.cfg.publicar_cada:
            return
        try:
            self.bus.publish("agent/table", self.agent.grid(self.disc))
        except Exception:
            pass

    def _publicar_estado(self, fase, met, extra=None):
        d = {"algo": self.agent.nombre, "phase": fase,
             "epsilon": round(float(getattr(self.agent, "epsilon", 0.0)), 4),
             "pct_banda": round(met.pct_banda, 1) if met.n else None,
             "rmse": round(met.rmse, 1) if met.n else None,
             "estados": len(getattr(self.agent, "q", {}) or {}),
             "pasos": self._n, "t": time.time()}
        if extra:
            d.update(extra)
        try:
            self.bus.publish("agent/status", d)
        except Exception:
            pass

    # ---- FASE 1: pre-entrenamiento sobre el modelo ---------------------------
    def _episodio_modelo(self, plant, aprender=True, pasos=None, sp_fijo=None):
        """Un episodio contra el modelo. Devuelve % de muestras en banda."""
        cfg = self.cfg
        pasos = pasos or cfg.pasos_por_episodio
        plant.reset(random.uniform(0, 0.3 * RPM_MAX))
        pwm = random.randint(0, 60)
        sp = sp_fijo if sp_fijo else random.uniform(SP_MIN, SP_MAX)
        rpm = plant.rpm
        rpm_prev, sp_prev = rpm, sp
        s = hacer_estado(rpm, sp, rpm_prev, sp_prev, pwm)
        a = self._safe_action(s, self.agent.act(s, explore=aprender))
        self.agent.on_reset()
        en_banda_n = 0

        for k in range(pasos):
            # Cambios de consigna a mitad de episodio: obliga a aprender a
            # SEGUIR referencias, no a memorizar un punto de operacion.
            if sp_fijo is None and k and k % 120 == 0:
                sp_prev, sp = sp, random.uniform(SP_MIN, SP_MAX)
            pwm_nuevo = self._aplicar(pwm, a)
            du = pwm_nuevo - pwm
            pwm = pwm_nuevo
            rpm_prev2 = rpm
            rpm = plant.step(pwm, self.ts)
            s2 = hacer_estado(rpm, sp, rpm_prev2, sp_prev, pwm)
            sp_prev = sp
            r = self._reward(s2, du)
            if abs(s2["err"]) <= cfg.tolerancia:
                en_banda_n += 1
            a2 = self._safe_action(s2, self.agent.act(s2, explore=aprender))
            if aprender:
                self.agent.observe(s, a, r, s2, a2, done=(k == pasos - 1))
            s, a = s2, a2
        if aprender:
            self.agent.end_episode()
        return 100.0 * en_banda_n / pasos

    def entrenar_offline(self):
        cfg = self.cfg
        plant = cargar_planta()
        print(f"\n--- PRE-ENTRENAMIENTO ACELERADO ({self.agent.nombre}) ---")
        print(f"  Ts={self.ts*1000:.0f} ms (la real del bus), "
              f"{cfg.pasos_por_episodio} pasos/episodio, meta {cfg.objetivo_pct*100:.0f}% en banda.")
        t0 = time.time()
        for ep in range(1, cfg.episodios_max + 1):
            self._episodio_modelo(plant, aprender=True)
            if ep % cfg.evaluar_cada == 0:
                pct = self._episodio_modelo(plant, aprender=False,
                                            pasos=cfg.pasos_evaluacion)
                eps = getattr(self.agent, "epsilon", None)
                extra = f"  eps={eps:.3f}" if eps is not None else ""
                print(f"  ep {ep:4d}  en banda={pct:5.1f}%{extra}")
                if ep >= cfg.episodios_min and pct >= cfg.objetivo_pct * 100:
                    print(f"  Convergido en {ep} episodios ({time.time()-t0:.1f} s).")
                    break
        self.agent.save()
        print(f"  Pre-entrenamiento terminado en {time.time()-t0:.1f} s.")

    # ---- FASE 2: tiempo real sobre el motor ---------------------------------
    def lazo_real(self, aprender=True):
        cfg = self.cfg
        bus = self.bus
        bus.tomar_mando()
        bus.set_enable(True)
        bus.set_pwm(0)
        bus.drain()
        self.pwm = 0

        m = bus.next_sample()
        sp = self._consigna(m)
        rpm_prev, sp_prev = m["rpm"], sp
        s = hacer_estado(m["rpm"], sp, rpm_prev, sp_prev, self.pwm, m["t_ms"] / 1000)
        a = self._safe_action(s, self.agent.act(s, explore=aprender))
        met = MetricasMoviles(cfg.ventana_metrica, cfg.tolerancia, cfg.rpm_limite)
        self.agent.on_reset()
        fase = "aprendiendo" if aprender else "controlando"
        print(f"\n--- {fase.upper()} EN TIEMPO REAL ({self.agent.nombre}) ---")
        print("  Ctrl+C para parar el motor y soltar el mando.\n")

        try:
            while True:
                # 1) actuar: la accion es un INCREMENTO sobre el PWM vigente
                pwm_nuevo = self._aplicar(self.pwm, a)
                du = pwm_nuevo - self.pwm
                self.pwm = pwm_nuevo
                if not self._set_pwm(self.pwm):
                    # sin mando: el PWM real lo pone otro; se lee de la muestra
                    pass

                # 2) esperar la SIGUIENTE muestra: aqui se sincroniza el lazo
                m = bus.next_sample(timeout=15.0)
                if self._sin_mando:
                    self.pwm = m["pwm"]          # seguir el PWM real ajeno
                sp_ant, sp = sp, self._consigna(m)
                s2 = hacer_estado(m["rpm"], sp, rpm_prev, sp_ant, self.pwm,
                                  m["t_ms"] / 1000)
                rpm_prev = m["rpm"]

                # 3) recompensa y aprendizaje
                r = self._reward(s2, du)
                met.push(s2)
                a2 = self._safe_action(s2, self.agent.act(s2, explore=aprender))
                self._n += 1
                if aprender:
                    fin = (self._n % cfg.mini_episodio == 0)
                    self.agent.observe(s, a, r, s2, a2, done=False)
                    if fin:
                        self.agent.end_episode()   # cierra "episodio" para MC
                        self.agent.on_reset()
                    if self._n % cfg.guardar_cada == 0:
                        self.agent.save()
                s, a = s2, a2

                # 4) publicar para la GUI e imprimir
                self._publicar_tabla()
                if self._n % cfg.publicar_cada == 0:
                    self._publicar_estado(fase, met, {"pwm": self.pwm,
                                                      "reward": round(r, 3)})
                if self._n % cfg.imprimir_cada == 0:
                    print(f"  t={s2['t']:7.1f}s  sp={sp:6.0f}  rpm={s2['rpm']:6.0f}  "
                          f"e={s2['err']:+6.0f}  pwm={self.pwm:3d}  "
                          f"banda(ult.{met.n})={met.pct_banda:5.1f}%  "
                          f"rmse={met.rmse:5.1f}"
                          + (f"  eps={self.agent.epsilon:.3f}"
                             if hasattr(self.agent, "epsilon") else ""))
        except KeyboardInterrupt:
            print(f"\n[{self.agent.nombre}] Detenido. "
                  f"banda(ult.{met.n})={met.pct_banda:.1f}%  rmse={met.rmse:.1f}")
        finally:
            try:
                bus.set_pwm(0)
                bus.set_enable(False)
                bus.soltar_mando()
            except Exception:
                pass
            if aprender:
                self.agent.save()
            self._publicar_tabla(forzar=True)
            self._publicar_estado("parado", met)

    def _consigna(self, muestra):
        """Consigna a seguir en ESTE paso del lazo real (no se usa en el
        pre-entrenamiento acelerado, que corre sobre el modelo de planta con
        su propia logica). 'consigna_defecto' es un ultimo recurso para el
        caso (raro en el bus real, que siempre fusiona "setpoint" en cada
        muestra) de que el campo directamente no venga en la muestra -- NO
        es para sustituir un 0 real: 0 es una consigna valida (parar el
        motor a proposito), igual que ya respeta ctl_pid.py. Antes se
        trataba cualquier "setpoint" <= 0 como "no hay consigna todavia" y
        se arrancaba igual con 1200 rpm sin que nadie lo pidiera.
        """
        if self.cfg.consigna_desde_bus:
            sp = muestra.get("setpoint")
            if sp is not None:
                return float(sp)
        return self.cfg.consigna_defecto

    # ---- ciclo completo ------------------------------------------------------
    def run(self, modo="auto", acelerado=None):
        """
        modo: 'controlar' | 'entrenar' | 'reentrenar' | 'auto'
        acelerado: True = pre-entrena contra el modelo antes de ir al motor.
        """
        print(f"[{self.agent.nombre}] Ts={self.ts*1000:.0f} ms -> "
              f"{1/self.ts:.0f} decisiones/s. Acciones dPWM={list(ACCIONES)}.")
        hay_previo = hay_entrenamiento_previo(self.agent)
        if modo == "auto":
            modo, acc = preguntar_modo(self.agent, hay_previo)
            if acelerado is None:
                acelerado = acc
        if acelerado is None:
            acelerado = True

        if modo == "controlar":
            if not hay_previo or not self.agent.load():
                print(f"[{self.agent.nombre}] No hay entrenamiento previo utilizable; "
                      f"no se puede controlar sin entrenar.")
                return
            self._publicar_tabla(forzar=True)
            self.lazo_real(aprender=False)
            return
        if modo == "entrenar" and hay_previo:
            self.agent.load()
            print(f"[{self.agent.nombre}] Se continua sobre lo guardado.")
        elif modo == "reentrenar":
            print(f"[{self.agent.nombre}] Entrenamiento desde cero.")

        if acelerado:
            self.entrenar_offline()
            self._publicar_tabla(forzar=True)
        self.lazo_real(aprender=True)


# =============================================================================
# ARRANQUE COMUN A LOS TRES SCRIPTS
# =============================================================================

def agregar_args_modo(ap):
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--controlar", action="store_true",
                   help="Solo controlar con lo aprendido (sin aprender).")
    g.add_argument("--entrenar", action="store_true",
                   help="Entrenar continuando sobre lo guardado.")
    g.add_argument("--reentrenar", "--train", dest="reentrenar", action="store_true",
                   help="Entrenar desde cero (ignora lo guardado).")
    r = ap.add_mutually_exclusive_group()
    r.add_argument("--acelerado", action="store_true",
                   help="Pre-entrenar contra el modelo antes de tocar el motor.")
    r.add_argument("--directo", action="store_true",
                   help="Ir directo al motor real (sin pre-entrenamiento).")
    ap.add_argument("--sp", type=float, default=None,
                    help="Publica esta consigna de rpm al arrancar.")
    return ap


def modo_desde_args(args):
    if getattr(args, "controlar", False):
        modo = "controlar"
    elif getattr(args, "entrenar", False):
        modo = "entrenar"
    elif getattr(args, "reentrenar", False):
        modo = "reentrenar"
    else:
        modo = "auto"
    if getattr(args, "acelerado", False):
        acc = True
    elif getattr(args, "directo", False):
        acc = False
    else:
        acc = None
    return {"modo": modo, "acelerado": acc}


def ruta_guardado(agent):
    return getattr(agent, "qtable_path", None) or getattr(agent, "policy_path", None)


def hay_entrenamiento_previo(agent) -> bool:
    p = ruta_guardado(agent)
    return bool(p) and os.path.exists(p)


def _leer_opcion(prompt, validas, defecto):
    try:
        resp = input(prompt).strip()
    except (EOFError, KeyboardInterrupt):
        print(f"(sin entrada interactiva -> se usa opcion '{defecto}')")
        return defecto
    return resp if resp in validas else defecto


def preguntar_modo(agent, hay_previo):
    ruta = ruta_guardado(agent)
    if hay_previo:
        print(f"\nHay entrenamiento previo de {agent.nombre} ({ruta}).")
        print("  [1] Controlar   - usar lo aprendido (sin seguir aprendiendo)")
        print("  [2] Entrenar    - seguir mejorando sobre lo guardado")
        print("  [3] Reentrenar  - descartar y empezar de cero")
        op = _leer_opcion("Opcion [1/2/3] (Enter=1): ", {"1", "2", "3"}, "1")
        modo = {"1": "controlar", "2": "entrenar", "3": "reentrenar"}[op]
        if modo == "controlar":
            return modo, False
    else:
        print(f"\nNo hay entrenamiento previo de {agent.nombre}: se entrena de cero.")
        modo = "reentrenar"
    print("\nRitmo:")
    print("  [1] Acelerado - pre-entrena contra el modelo y luego pasa al motor (recomendado)")
    print("  [2] Directo   - aprende desde cero sobre el motor real (lento y brusco)")
    op = _leer_opcion("Opcion [1/2] (Enter=1): ", {"1", "2"}, "1")
    return modo, (op == "1")


def preparar(agent_factory, descripcion, disc=None):
    """Arranque comun: parsea args, conecta al bus, monta runner y corre."""
    ap = argparse.ArgumentParser(description=descripcion)
    agregar_args_modo(ap)
    args = ap.parse_args()
    agent = agent_factory()
    bus = conectar_o_avisar(getattr(agent, "clave", agent.nombre))
    if bus is None:
        return
    with bus:
        bus.subscribe("telemetry")     # imprescindible: alimenta next_sample()
        if args.sp is not None:
            bus.set_setpoint(args.sp)
        cfg = RunConfig()
        runner = MotorRunner(bus, agent, cfg, disc)
        runner.run(**modo_desde_args(args))


# =============================================================================
# MAIN (utilidades de la infraestructura)
# =============================================================================

def main():
    ap = argparse.ArgumentParser(description="Utilidades de la infraestructura RL del motor.")
    ap.add_argument("--identificar", action="store_true",
                    help="Ensayo de escalera+escalon: ajusta el modelo de la planta.")
    args = ap.parse_args()
    if args.identificar:
        bus = conectar_o_avisar("identificacion")
        if bus is None:
            return
        with bus:
            bus.subscribe("telemetry")
            identificar_planta(bus)
    else:
        ap.print_help()


if __name__ == "__main__":
    main()
