#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
motor_bus.py  -  BUS DE COMUNICACION (broker publicador/suscriptor)
===================================================================

Es el UNICO proceso que toca el puerto serie del Arduino. Todo lo demas (GUI,
Q-learning, SARSA(lambda), REINFORCE, scripts de prueba) se conecta al bus por
TCP/JSON y se SUSCRIBE / PUBLICA en topicos. Asi varios programas comparten el
motor sin pelearse por el puerto.

        Arduino Mega  <--USB serie-->  motor_bus.py  <--TCP 127.0.0.1:8770-->  N clientes
                                            |                                    |
                                    senal medida (rpm)                     motor_gui.py
                                    senal control (pwm)                    rl_motor_qlearning.py
                                                                           rl_motor_sarsa_lambda.py
                                                                           rl_motor_reinforce.py

TOPICOS (todos con "retencion": el ultimo valor se entrega al suscribirse)
--------------------------------------------------------------------------
  telemetry    <- publicado por el BUS a cada muestra (cadencia del Arduino)
                  {"t_ms","seq","pwm","adc","volts","rpm","enabled","t_pc"}
  control      <- publicado por QUIEN MANDA. {"pwm":0..255,"src":"<nombre>"}
                  El bus lo traduce a la orden serie 'P<pwm>'.
  enable       <- {"on":true|false,"src":...} -> 'E<0|1>'
  setpoint     <- {"rpm":float,"src":...}  consigna de velocidad (la fija la GUI)
  mode         <- {"owner":"manual"|"qlearning"|"sarsa_lambda"|"reinforce"|"free"}
                  ARBITRAJE: el bus solo acepta 'control' cuyo src == owner
                  (o si owner == "free"). Evita que dos algoritmos peleen.
  agent/table  <- publicado por los algoritmos: instantanea de su tabla/politica
                  para que la GUI la dibuje (ver rl_motor_common.snapshot()).
  agent/status <- {"algo","phase","episode","epsilon","reward","states",...}
  log          <- mensajes crudos del Arduino (#ok / #err / #info)

PROTOCOLO TCP (una linea JSON por mensaje, UTF-8)
------------------------------------------------
  Cliente -> bus:
    {"cmd":"hello","name":"qlearning"}
    {"cmd":"subscribe","topics":["telemetry","setpoint"]}
    {"cmd":"unsubscribe","topics":[...]}
    {"cmd":"publish","topic":"control","data":{"pwm":120}}
    {"cmd":"get","topic":"telemetry"}          -> {"ok":true,"data":{...}}
    {"cmd":"info"}                             -> estado del bus
  Bus -> cliente:
    {"ok":true, ...}                           respuesta a un cmd
    {"type":"event","topic":"telemetry","data":{...},"seq":N}   asincrono

SEGURIDAD
---------
  * keepalive: el bus reenvia el PWM vigente cada 500 ms; si el bus muere, el
    watchdog del Arduino (1.5 s) corta el motor solo.
  * limite de RPM (--rpm-max): si la medida lo supera, el bus fuerza PWM=0 y
    publica un aviso. Es un interlock por encima de cualquier politica.
  * --sim: modelo de motor de primer orden con zona muerta y ruido, para
    desarrollar y entrenar SIN hardware. Misma cadencia y mismos topicos.

USO
---
    python3 motor_bus.py --sim                       # sin hardware
    python3 motor_bus.py --serial /dev/ttyACM0       # Linux
    python3 motor_bus.py --serial COM5               # Windows
    python3 motor_bus.py --list                      # ver puertos

Solo libreria estandar (pyserial solo si NO usas --sim).
"""

import argparse
import json
import math
import random
import socket
import sys
import threading
import time

HOST = "127.0.0.1"
PORT = 8770
BAUD = 9600

# Debe coincidir con RPM_FS_DEFAULT del firmware (rpm a 5 V del tacometro).
RPM_FS = 3000.0

TOPICS_RETENIDOS = ("telemetry", "control", "enable", "setpoint", "mode",
                    "agent/table", "agent/status", "log")


# =============================================================================
# MODELO DE MOTOR (solo para --sim y para el pre-entrenamiento acelerado)
# =============================================================================

class MotorPlant:
    """
    Motor DC de primer orden visto desde el PWM:

        duty = pwm/255
        u    = 0                       si duty <= zona_muerta
               (duty - zm) / (1 - zm)  si no          (normalizado 0..1)
        rpm' = (K*u - rpm) / tau                       (+ carga + ruido)

    Con K = ganancia estatica (rpm a duty=1), tau = constante de tiempo (s) y
    zona muerta (friccion estatica). Es un modelo POBRE pero suficiente: el RL
    lo usa solo para pre-entrenar y luego afina contra el motor real.
    """

    def __init__(self, K=2850.0, tau=0.35, zona_muerta=0.12, ruido=8.0,
                 carga=0.0, rpm_max=RPM_FS):
        self.K, self.tau, self.zm = K, tau, zona_muerta
        self.ruido, self.carga, self.rpm_max = ruido, carga, rpm_max
        self.rpm = 0.0

    def reset(self, rpm=0.0):
        self.rpm = rpm
        return self.rpm

    def step(self, pwm, dt):
        duty = max(0.0, min(1.0, pwm / 255.0))
        u = 0.0 if duty <= self.zm else (duty - self.zm) / (1.0 - self.zm)
        objetivo = self.K * u - self.carga
        self.rpm += (dt / self.tau) * (objetivo - self.rpm)
        self.rpm = max(0.0, min(self.rpm_max * 1.05, self.rpm))
        # El ruido es de MEDIDA (tacometro + ADC), no del eje: no se integra en
        # el estado, se anade a la lectura. Si se integrara, el motor haria un
        # paseo aleatorio que no existe en la realidad.
        medida = self.rpm + (random.gauss(0.0, self.ruido) if self.ruido else 0.0)
        return max(0.0, medida)


# =============================================================================
# ENLACES CON LA PLANTA (real o simulada). Misma interfaz para el broker.
# =============================================================================

class BaseLink(threading.Thread):
    """Interfaz comun: produce muestras -> on_sample(dict); acepta ordenes."""

    def __init__(self, on_sample, on_log):
        super().__init__(daemon=True)
        self.on_sample = on_sample
        self.on_log = on_log
        self._stop = threading.Event()
        self.pwm = 0
        self.enabled = False

    def set_pwm(self, v):
        raise NotImplementedError

    def set_enable(self, on):
        raise NotImplementedError

    def stop(self):
        self._stop.set()


class SerialLink(BaseLink):
    """Puente con el Arduino real por USB."""

    def __init__(self, port, baud, ts_ms, rpm_fs, on_sample, on_log):
        super().__init__(on_sample, on_log)
        import serial  # import perezoso: solo se necesita con hardware
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.lock = threading.Lock()
        self.ts_ms = ts_ms
        time.sleep(2.0)          # el Mega se reinicia al abrir el puerto
        self.ser.reset_input_buffer()
        self._write(f"T{int(ts_ms)}")
        self._write(f"K{float(rpm_fs):.1f}")
        self._write("Z")

    def _write(self, txt):
        with self.lock:
            self.ser.write((txt + "\n").encode("ascii"))

    def set_pwm(self, v):
        self.pwm = int(v)
        self._write(f"P{self.pwm}")

    def set_enable(self, on):
        self.enabled = bool(on)
        self._write(f"E{1 if on else 0}")

    def run(self):
        seq = 0
        while not self._stop.is_set():
            try:
                raw = self.ser.readline()
            except Exception as e:
                self.on_log(f"serie caido: {e}")
                break
            if not raw:
                continue
            linea = raw.decode("ascii", "replace").strip()
            if not linea:
                continue
            if linea.startswith("D,"):
                p = linea.split(",")
                if len(p) != 7:
                    continue
                try:
                    seq += 1
                    self.on_sample({
                        "t_ms": int(p[1]), "seq": seq, "pwm": int(p[2]),
                        "adc": int(p[3]), "volts": int(p[4]) / 1000.0,
                        "rpm": float(p[5]), "enabled": bool(int(p[6])),
                        "t_pc": time.time(),
                    })
                except ValueError:
                    continue
            else:
                self.on_log(linea)
        try:
            self.ser.write(b"Z\n")
            self.ser.close()
        except Exception:
            pass


class SimLink(BaseLink):
    """Motor simulado con la MISMA cadencia y los mismos mensajes."""

    def __init__(self, ts_ms, rpm_fs, on_sample, on_log, plant=None):
        super().__init__(on_sample, on_log)
        self.ts = ts_ms / 1000.0
        self.rpm_fs = rpm_fs
        self.plant = plant or MotorPlant(rpm_max=rpm_fs)

    def set_pwm(self, v):
        self.pwm = int(max(0, min(255, v)))

    def set_enable(self, on):
        self.enabled = bool(on)

    def run(self):
        self.on_log("#hello motor-rl 1.0 (SIMULADO)")
        t0 = time.time()
        seq = 0
        siguiente = t0
        while not self._stop.is_set():
            siguiente += self.ts
            dormir = siguiente - time.time()
            if dormir > 0:
                time.sleep(dormir)
            else:
                siguiente = time.time()
            pwm_ef = self.pwm if self.enabled else 0
            rpm = self.plant.step(pwm_ef, self.ts)
            volts = max(0.0, min(5.0, rpm / self.rpm_fs * 5.0))
            seq += 1
            self.on_sample({
                "t_ms": int((time.time() - t0) * 1000), "seq": seq,
                "pwm": pwm_ef, "adc": int(volts / 5.0 * 1023),
                "volts": round(volts, 4), "rpm": round(rpm, 1),
                "enabled": self.enabled, "t_pc": time.time(),
            })


# =============================================================================
# BROKER PUB/SUB
# =============================================================================

class Broker:
    def __init__(self):
        self.lock = threading.RLock()
        self.retenido = {}          # topico -> data
        self.clientes = []          # lista de ClientConn
        self.seq = 0

    def registrar(self, c):
        with self.lock:
            self.clientes.append(c)

    def quitar(self, c):
        with self.lock:
            if c in self.clientes:
                self.clientes.remove(c)

    def publicar(self, topico, data):
        with self.lock:
            self.seq += 1
            seq = self.seq
            if topico in TOPICS_RETENIDOS:
                self.retenido[topico] = data
            destinos = [c for c in self.clientes if topico in c.topicos]
        msg = {"type": "event", "topic": topico, "data": data, "seq": seq}
        for c in destinos:
            c.enviar(msg)

    def ultimo(self, topico):
        with self.lock:
            return self.retenido.get(topico)


class ClientConn(threading.Thread):
    """Una conexion TCP: lee comandos y empuja eventos."""

    def __init__(self, conn, addr, bus):
        super().__init__(daemon=True)
        self.conn, self.addr, self.bus = conn, addr, bus
        self.topicos = set()
        self.nombre = f"anon:{addr[1]}"
        self.wlock = threading.Lock()
        self.vivo = True

    def enviar(self, msg):
        if not self.vivo:
            return
        try:
            with self.wlock:
                self.conn.sendall((json.dumps(msg) + "\n").encode("utf-8"))
        except OSError:
            self.vivo = False

    def run(self):
        buf = b""
        try:
            while self.vivo:
                data = self.conn.recv(4096)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    linea, buf = buf.split(b"\n", 1)
                    if linea.strip():
                        self.enviar(self._procesar(linea))
        except (OSError, ConnectionError):
            pass
        finally:
            self.vivo = False
            self.bus.broker.quitar(self)
            self.bus.al_desconectar(self.nombre)
            try:
                self.conn.close()
            except OSError:
                pass

    def _procesar(self, linea):
        try:
            m = json.loads(linea.decode("utf-8"))
        except ValueError as e:
            return {"ok": False, "error": f"json invalido: {e}"}
        cmd = m.get("cmd")
        rid = m.get("id")
        r = self._despachar(cmd, m)
        if rid is not None:
            r["id"] = rid
        return r

    def _despachar(self, cmd, m):
        b = self.bus
        if cmd == "hello":
            self.nombre = str(m.get("name") or self.nombre)
            return {"ok": True, "name": self.nombre, "bus": b.descripcion()}
        if cmd == "subscribe":
            tops = m.get("topics") or []
            self.topicos.update(tops)
            # Entrega inmediata del ultimo valor retenido de cada topico nuevo.
            for t in tops:
                ult = b.broker.ultimo(t)
                if ult is not None:
                    self.enviar({"type": "event", "topic": t, "data": ult,
                                 "seq": 0, "retained": True})
            return {"ok": True, "topics": sorted(self.topicos)}
        if cmd == "unsubscribe":
            for t in (m.get("topics") or []):
                self.topicos.discard(t)
            return {"ok": True, "topics": sorted(self.topicos)}
        if cmd == "publish":
            return b.publicar_desde(self, m.get("topic"), m.get("data") or {})
        if cmd == "get":
            t = m.get("topic")
            return {"ok": True, "topic": t, "data": b.broker.ultimo(t)}
        if cmd == "info":
            return {"ok": True, **b.descripcion()}
        return {"ok": False, "error": f"cmd desconocido: {cmd}"}


# =============================================================================
# BUS (servidor + arbitraje + seguridad)
# =============================================================================

class MotorBus:
    def __init__(self, link_factory, host=HOST, port=PORT, ts_ms=50,
                 rpm_fs=RPM_FS, rpm_max=None, keepalive_s=0.5):
        self.broker = Broker()
        self.host, self.port = host, port
        self.ts_ms, self.rpm_fs = ts_ms, rpm_fs
        self.rpm_max = rpm_max if rpm_max else rpm_fs * 1.02
        self.keepalive_s = keepalive_s
        self.link = link_factory(self._on_sample, self._on_log)
        self.srv = None
        self._stop = threading.Event()
        self.n_muestras = 0
        self.interlock = False
        # Estado inicial de los topicos de mando.
        self.broker.retenido["mode"] = {"owner": "manual"}
        self.broker.retenido["control"] = {"pwm": 0, "src": "bus"}
        self.broker.retenido["enable"] = {"on": False, "src": "bus"}
        self.broker.retenido["setpoint"] = {"rpm": 0.0, "src": "bus"}

    # ---- planta -> bus -------------------------------------------------------
    def _on_sample(self, s):
        self.n_muestras += 1
        # INTERLOCK de sobrevelocidad: por encima de todo algoritmo.
        if s["rpm"] > self.rpm_max:
            if not self.interlock:
                self.interlock = True
                self._on_log(f"#err INTERLOCK: {s['rpm']:.0f} rpm > "
                             f"{self.rpm_max:.0f} rpm -> PWM=0")
            self.link.set_pwm(0)
            self.link.set_enable(False)
            self.broker.publicar("enable", {"on": False, "src": "interlock"})
            self.broker.publicar("control", {"pwm": 0, "src": "interlock"})
        elif self.interlock and s["rpm"] < self.rpm_max * 0.9:
            self.interlock = False
            self._on_log("#ok interlock liberado (habilita de nuevo para seguir)")
        s["setpoint"] = float((self.broker.ultimo("setpoint") or {}).get("rpm", 0.0))
        s["owner"] = (self.broker.ultimo("mode") or {}).get("owner", "free")
        self.broker.publicar("telemetry", s)

    def _on_log(self, txt):
        self.broker.publicar("log", {"t": time.time(), "msg": txt})

    # ---- clientes -> planta --------------------------------------------------
    def publicar_desde(self, cliente, topico, data):
        if not topico:
            return {"ok": False, "error": "falta 'topic'"}
        src = data.get("src") or cliente.nombre
        data["src"] = src

        if topico == "control":
            owner = (self.broker.ultimo("mode") or {}).get("owner", "free")
            if owner not in ("free", src):
                return {"ok": False, "error": f"el mando es de '{owner}', no de '{src}'",
                        "owner": owner}
            if self.interlock:
                return {"ok": False, "error": "interlock de sobrevelocidad activo"}
            try:
                pwm = int(round(float(data.get("pwm"))))
            except (TypeError, ValueError):
                return {"ok": False, "error": "pwm invalido"}
            pwm = max(0, min(255, pwm))
            data["pwm"] = pwm
            self.link.set_pwm(pwm)

        elif topico == "enable":
            on = bool(data.get("on"))
            if on:
                self.interlock = False
            self.link.set_enable(on)
            if not on:
                self.link.set_pwm(0)

        elif topico == "mode":
            owner = str(data.get("owner") or "free")
            # Al cambiar de mando se corta el PWM: transferencia limpia.
            self.link.set_pwm(0)
            data = {"owner": owner, "src": src}
            self.broker.publicar("control", {"pwm": 0, "src": "bus"})

        self.broker.publicar(topico, data)
        return {"ok": True, "topic": topico}

    def al_desconectar(self, nombre):
        """Si el que se cae es quien tenia el mando, se para el motor."""
        owner = (self.broker.ultimo("mode") or {}).get("owner")
        if owner and owner == nombre:
            self._on_log(f"#err '{nombre}' se desconecto teniendo el mando -> PARADA")
            self.link.set_pwm(0)
            self.link.set_enable(False)
            self.broker.publicar("enable", {"on": False, "src": "bus"})
            self.broker.publicar("control", {"pwm": 0, "src": "bus"})
            self.broker.publicar("mode", {"owner": "manual", "src": "bus"})

    def descripcion(self):
        ctrl = self.broker.ultimo("control") or {}
        return {"ts_ms": self.ts_ms, "rpm_fs": self.rpm_fs, "rpm_max": self.rpm_max,
                "sim": isinstance(self.link, SimLink), "muestras": self.n_muestras,
                "owner": (self.broker.ultimo("mode") or {}).get("owner"),
                "pwm": ctrl.get("pwm", 0), "interlock": self.interlock,
                "clientes": [c.nombre for c in list(self.broker.clientes)],
                "topics": list(TOPICS_RETENIDOS)}

    # ---- servidor ------------------------------------------------------------
    def _keepalive(self):
        """Refresca el PWM vigente: mantiene contento al watchdog del Arduino."""
        while not self._stop.is_set():
            time.sleep(self.keepalive_s)
            try:
                if isinstance(self.link, SerialLink):
                    self.link.set_pwm(self.link.pwm)
            except Exception:
                pass

    def serve_forever(self):
        self.link.start()
        threading.Thread(target=self._keepalive, daemon=True).start()
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind((self.host, self.port))
        self.srv.listen(8)
        print(f"[bus] escuchando en {self.host}:{self.port}  "
              f"({'SIMULADO' if isinstance(self.link, SimLink) else 'serie'}, "
              f"Ts={self.ts_ms} ms, rpm_fs={self.rpm_fs:.0f}, "
              f"interlock={self.rpm_max:.0f} rpm)")
        print("[bus] topicos:", ", ".join(TOPICS_RETENIDOS))
        try:
            while not self._stop.is_set():
                conn, addr = self.srv.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                c = ClientConn(conn, addr, self)
                self.broker.registrar(c)
                c.start()
        except KeyboardInterrupt:
            print("\n[bus] cerrando...")
        finally:
            self.detener()

    def detener(self):
        self._stop.set()
        try:
            self.link.set_pwm(0)
            self.link.set_enable(False)
        except Exception:
            pass
        self.link.stop()
        if self.srv:
            try:
                self.srv.close()
            except OSError:
                pass


# =============================================================================
# MAIN
# =============================================================================

def listar_puertos():
    try:
        from serial.tools import list_ports
    except ImportError:
        print("pyserial no esta instalado:  pip install pyserial")
        return
    for p in list_ports.comports():
        print(f"  {p.device:20s} {p.description}")


def main():
    ap = argparse.ArgumentParser(description="Bus pub/sub Arduino <-> algoritmos.")
    ap.add_argument("--serial", help="puerto serie del Arduino (/dev/ttyACM0, COM5)")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--sim", action="store_true", help="motor simulado (sin hardware)")
    ap.add_argument("--list", action="store_true", help="lista puertos serie y sale")
    ap.add_argument("--host", default=HOST)
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--ts", type=int, default=50, help="periodo de muestreo en ms")
    ap.add_argument("--rpm-fs", type=float, default=RPM_FS,
                    help="rpm a 5 V del tacometro (debe coincidir con el firmware)")
    ap.add_argument("--rpm-max", type=float, default=None,
                    help="interlock de sobrevelocidad (por defecto 1.02*rpm_fs)")
    args = ap.parse_args()

    if args.list:
        listar_puertos()
        return
    if not args.sim and not args.serial:
        print("Indica --serial <puerto> o --sim.  (--list para ver puertos)")
        return

    if args.sim:
        def factory(on_s, on_l):
            return SimLink(args.ts, args.rpm_fs, on_s, on_l)
    else:
        def factory(on_s, on_l):
            return SerialLink(args.serial, args.baud, args.ts, args.rpm_fs, on_s, on_l)

    try:
        bus = MotorBus(factory, args.host, args.port, args.ts, args.rpm_fs, args.rpm_max)
    except ImportError:
        print("Falta pyserial:  pip install pyserial   (o usa --sim)")
        return
    except Exception as e:
        print(f"No se pudo abrir la planta: {e}")
        return
    bus.serve_forever()


if __name__ == "__main__":
    main()
