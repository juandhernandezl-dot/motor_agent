#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ctl_pid.py  -  CONTROLADOR CLASICO: PID discreto (referencia de comparacion)
============================================================================

No aprende nada: es la vara de medir. Si tus agentes de RL no le ganan a esto en
seguimiento, sobreimpulso o suavidad del PWM, todavia les falta entrenamiento.

Se conecta al MISMO bus, toma el mando igual, decide UNA vez por muestra igual y
publica 'agent/status' igual. Es el ejemplo de que la arquitectura no es
"para RL": es para cualquier controlador.

Que lleva dentro (todo lo que un PID de laboratorio necesita para no dar guerra)
-------------------------------------------------------------------------------
  * DERIVADA SOBRE LA MEDIDA, no sobre el error: un salto de consigna no produce
    un pico de PWM ("derivative kick").
  * FILTRO de la derivada (constante Tf): sin el, el ruido del tacometro entra
    amplificado por Kd y el motor tiembla.
  * ANTI-WINDUP por integracion condicional: con el PWM saturado (0 o 255) la
    integral deja de crecer; si no, al bajar la consigna el motor tardaria
    segundos en reaccionar mientras se "descarga".
  * FEEDFORWARD de zona muerta: suma de golpe el PWM que el motor necesita solo
    para empezar a girar. Sin esto el integrador tiene que "descubrirlo" en cada
    arranque, y eso es sobreimpulso seguro.
  * TRANSFERENCIA SIN SALTO (bumpless): mientras no tiene el mando, ajusta su
    integral al PWM que aplica el otro. Al recuperarlo, continua sin escalon.

Autoajuste
----------
  --auto usa el modelo identificado (plant_motor.json, de
  'python3 rl_motor_common.py --identificar') y aplica sintonia IMC para planta
  de primer orden:   Kc = tau / (Kp_planta * lambda),  Ti = tau,  Kd = 0
  con lambda = max(tau/2, 2*Ts). lambda es la agresividad: subelo para una
  respuesta mas suave, bajalo para una mas rapida (y mas nerviosa).

Uso
---
    python3 ctl_pid.py --auto --sp 1500
    python3 ctl_pid.py --kp 0.06 --ki 0.18 --kd 0.0 --sp 1500
"""

import argparse
import time

from motor_client import conectar_o_avisar
from rl_motor_common import (MetricasMoviles, TOLERANCIA, RPM_LIMITE, RPM_MAX,
                             cargar_planta, PLANT_PATH)

# =============================================================================
# MANIFIESTO (esto es lo unico que la GUI necesita para ofrecerlo)
# =============================================================================

CONTROLLER = {
    "clave": "pid",
    "nombre": "PID clasico",
    "familia": "clasico",
    "kind": "none",
    "descripcion": ("Control PID discreto con anti-windup, filtro derivativo y "
                    "feedforward de zona muerta. No aprende: sirve de referencia "
                    "para comparar contra los agentes de RL. Con --auto se "
                    "sintoniza solo a partir de plant_motor.json (IMC)."),
    "params": [
        {"tipo": "bool", "etiqueta": "Autoajuste IMC (usa plant_motor.json)",
         "flag": "--auto", "default": True},
        {"tipo": "float", "etiqueta": "Kp (pwm/rpm)", "flag": "--kp",
         "default": 0.05, "ayuda": "ignorado si --auto"},
        {"tipo": "float", "etiqueta": "Ki (pwm/rpm/s)", "flag": "--ki",
         "default": 0.15, "ayuda": "ignorado si --auto"},
        {"tipo": "float", "etiqueta": "Kd (pwm·s/rpm)", "flag": "--kd",
         "default": 0.0},
        {"tipo": "float", "etiqueta": "Tf filtro derivada (s)", "flag": "--tf",
         "default": 0.05},
        {"tipo": "float", "etiqueta": "lambda IMC (agresividad)", "flag": "--lam",
         "default": None, "ayuda": "vacio = tau/2. Subir = mas suave"},
        {"tipo": "float", "etiqueta": "Consigna (rpm)", "flag": "--sp",
         "default": None, "ayuda": "vacio = la de la GUI"},
        {"tipo": "bool", "etiqueta": "Sin feedforward de zona muerta",
         "flag": "--sin-ff", "default": False},
    ],
}


# =============================================================================
# PID
# =============================================================================

class PID:
    def __init__(self, kp, ki, kd, ts, tf=0.05, umin=0.0, umax=255.0, ff=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.ts, self.tf = ts, tf
        self.umin, self.umax = umin, umax
        self.ff = ff
        self.i = 0.0
        self.d = 0.0
        self.y_prev = None

    def reset(self, u_actual=0.0, y=0.0):
        self.i = max(self.umin, min(self.umax, u_actual)) - self.ff
        self.d = 0.0
        self.y_prev = y

    def paso(self, sp, y, ts=None):
        ts = ts or self.ts
        if self.y_prev is None:
            self.y_prev = y
        e = sp - y
        P = self.kp * e
        # Derivada SOBRE LA MEDIDA (con signo cambiado) y filtrada.
        dy = (y - self.y_prev) / ts
        self.y_prev = y
        if self.kd > 0:
            alpha = ts / (self.tf + ts)
            self.d += alpha * (-self.kd * dy - self.d)
        else:
            self.d = 0.0
        ff = self.ff if sp > 1.0 else 0.0
        u_sin_sat = P + self.i + self.d + ff
        u = max(self.umin, min(self.umax, u_sin_sat))
        # Anti-windup: solo integra si eso NO empuja mas contra la saturacion.
        saturado_arriba = u_sin_sat > self.umax and e > 0
        saturado_abajo = u_sin_sat < self.umin and e < 0
        if not (saturado_arriba or saturado_abajo):
            self.i += self.ki * e * ts
            self.i = max(self.umin - 40, min(self.umax + 40, self.i))
        return u, P, self.i, self.d

    def bumpless(self, u_real, y):
        """Mientras no manda: coloca la integral donde toca para no dar salto."""
        self.y_prev = y
        self.i = u_real - self.ff


def sintonia_imc(ts, lam=None, ruta=PLANT_PATH):
    """Kp, Ki, Kd (PI) y feedforward a partir del modelo identificado."""
    planta = cargar_planta(ruta)
    K, tau, zm = planta.K, planta.tau, planta.zm
    kp_planta = K / 255.0                      # rpm por cuenta de PWM
    if kp_planta <= 0:
        return 0.05, 0.15, 0.0, 0.0
    lam = lam if lam else max(tau / 2.0, 2.0 * ts)
    kc = tau / (kp_planta * lam)
    ti = tau
    ff = zm * 255.0
    print(f"  [autoajuste IMC] K={K:.0f} rpm  tau={tau:.2f}s  zm={zm*100:.0f}%  "
          f"lambda={lam:.2f}s")
    print(f"  [autoajuste IMC] Kp={kc:.4f}  Ki={kc/ti:.4f}  Kd=0  ff={ff:.0f} pwm")
    return kc, kc / ti, 0.0, ff


# =============================================================================
# LAZO
# =============================================================================

def main():
    ap = argparse.ArgumentParser(description="Control de velocidad por PID clasico.")
    ap.add_argument("--kp", type=float, default=0.05)
    ap.add_argument("--ki", type=float, default=0.15)
    ap.add_argument("--kd", type=float, default=0.0)
    ap.add_argument("--tf", type=float, default=0.05)
    ap.add_argument("--lam", type=float, default=None)
    ap.add_argument("--auto", action="store_true", help="sintonia IMC automatica")
    ap.add_argument("--sin-ff", dest="sin_ff", action="store_true")
    ap.add_argument("--sp", type=float, default=None)
    args = ap.parse_args()

    bus = conectar_o_avisar("pid")
    if bus is None:
        return
    with bus:
        bus.subscribe("telemetry")
        ts = float(bus.info.get("ts_ms", 50)) / 1000.0
        # Ts en caliente por SUSCRIPCION: antes se hacia bus.get("config") en
        # CADA muestra (una ida y vuelta TCP sincrona por paso de control, que
        # ademas compite por el lock del cliente). Ahora el bus nos empuja el
        # cambio solo cuando ocurre y aqui se lee de una variable local.
        cfg_ts = {"ms": ts * 1000.0}
        bus.subscribe("config",
                      lambda d: cfg_ts.__setitem__("ms", float(d.get("ts_ms",
                                                                cfg_ts["ms"]))))
        if args.sp is not None:
            bus.set_setpoint(args.sp)

        if args.auto:
            kp, ki, kd, ff = sintonia_imc(ts, args.lam)
        else:
            kp, ki, kd = args.kp, args.ki, args.kd
            ff = 0.0 if args.sin_ff else cargar_planta(verbose=False).zm * 255.0
        if args.sin_ff:
            ff = 0.0

        pid = PID(kp, ki, kd, ts, args.tf, 0.0, 255.0, ff)
        met = MetricasMoviles(100, TOLERANCIA, RPM_LIMITE)

        bus.tomar_mando()
        bus.set_enable(True)
        bus.drain()
        m = bus.next_sample()
        pid.reset(0.0, m["rpm"])
        sin_mando = False
        n = 0
        print(f"\n--- PID EN MARCHA ---  Kp={kp:.4f} Ki={ki:.4f} Kd={kd:.4f} "
              f"Tf={args.tf:.3f}s ff={ff:.0f}  Ts={ts*1000:.0f} ms")
        print("  Ctrl+C para parar el motor y soltar el mando.\n")
        t_ini = time.time()
        try:
            while True:
                m = bus.next_sample(timeout=15.0)
                # Ts en caliente: si la GUI lo cambia, el PID se entera solo
                # (via la suscripcion a 'config'; cero trafico extra por paso).
                ts_nuevo = cfg_ts["ms"] / 1000.0
                if abs(ts_nuevo - ts) > 1e-9:
                    print(f"  [config] Ts {ts*1000:.0f} -> {ts_nuevo*1000:.0f} ms "
                          f"(el PID reescala la integral y la derivada solo)")
                    ts = ts_nuevo
                    pid.ts = ts
                sp = float(m.get("setpoint") or 0.0)
                y = m["rpm"]
                u, P, I, D = pid.paso(sp, y, ts)
                if sp <= 0:
                    u, pid.i = 0.0, 0.0
                try:
                    bus.set_pwm(int(round(u)))
                    if sin_mando:
                        sin_mando = False
                        print("  [mando] recuperado (transferencia sin salto).")
                except Exception as e:
                    if not sin_mando:
                        sin_mando = True
                        print(f"  [mando] sin control ({e}); sigo en bumpless.")
                    pid.bumpless(m["pwm"], y)
                met.push({"err": sp - y, "rpm": y})
                n += 1
                if n % 10 == 0:
                    bus.publish("agent/status", {
                        "algo": "PID", "kind": "none", "phase": "controlando",
                        "pasos": n, "pwm": int(round(u)),
                        "p": round(P, 1), "i": round(I, 1), "d": round(D, 1),
                        "pct_banda": round(met.pct_banda, 1),
                        "rmse": round(met.rmse, 1), "t": time.time()})
                if n % 20 == 0:
                    print(f"  t={time.time()-t_ini:6.1f}s  sp={sp:6.0f}  "
                          f"rpm={y:6.0f}  e={sp-y:+6.0f}  pwm={u:5.1f}  "
                          f"[P={P:+6.1f} I={I:+6.1f} D={D:+5.1f}]  "
                          f"banda={met.pct_banda:5.1f}%  rmse={met.rmse:5.1f}")
        except KeyboardInterrupt:
            print(f"\n[PID] Detenido. banda(ult.{met.n})={met.pct_banda:.1f}%  "
                  f"rmse={met.rmse:.1f}")
        finally:
            try:
                bus.set_pwm(0)
                bus.set_enable(False)
                bus.soltar_mando()
            except Exception:
                pass


if __name__ == "__main__":
    main()
