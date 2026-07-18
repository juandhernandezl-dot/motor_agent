#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
motor_client.py  -  CLIENTE del bus (esto es lo que usan GUI y algoritmos)
==========================================================================

Envuelve el protocolo TCP/JSON de motor_bus.py en 4 operaciones:

    bus = BusClient("qlearning"); bus.connect()
    bus.subscribe("telemetry", callback)     # asincrono (hilo lector)
    bus.publish("control", {"pwm": 120})
    dato = bus.get("setpoint")               # ultimo valor retenido
    m = bus.next_sample()                    # bloquea hasta la siguiente muestra

'next_sample()' es la pieza clave para el control: SINCRONIZA el lazo con el
muestreo real del Arduino (igual que 'sample(since=...)' hacia con el simulador
de vermicompostaje). El agente decide exactamente una vez por muestra.

Uso directo como monitor:
    python3 motor_client.py                 # imprime la telemetria
    python3 motor_client.py --pwm 100       # toma el mando y aplica PWM=100
"""

import argparse
import json
import queue
import socket
import threading
import time

HOST, PORT = "127.0.0.1", 8770


class BusError(RuntimeError):
    pass


class BusClient:
    """Cliente pub/sub con hilo lector. Seguro para usar desde Tk o consola."""

    def __init__(self, name="anon", host=HOST, port=PORT, timeout=5.0):
        self.name, self.host, self.port, self.timeout = name, host, port, timeout
        self.sock = None
        self._buf = b""
        self._callbacks = {}          # topico -> [fn(data)]
        self._respuestas = queue.Queue()
        self._muestras = queue.Queue(maxsize=200)
        self._wlock = threading.Lock()
        self._rlock = threading.Lock()
        self._hilo = None
        self._vivo = False
        self._rid = 0                 # contador de ids de comando (ver _cmd)
        self.info = {}
        self.on_close = None          # callback opcional si se cae el bus

    # ---- conexion ------------------------------------------------------------
    def connect(self):
        self.sock = socket.create_connection((self.host, self.port), self.timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(None)
        self._vivo = True
        self._hilo = threading.Thread(target=self._leer, daemon=True)
        self._hilo.start()
        r = self._cmd({"cmd": "hello", "name": self.name})
        self.info = r.get("bus", {})
        return self

    def close(self):
        self._vivo = False
        try:
            self.sock.close()
        except (OSError, AttributeError):
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ---- hilo lector ---------------------------------------------------------
    def _leer(self):
        while self._vivo:
            try:
                data = self.sock.recv(8192)
            except OSError:
                break
            if not data:
                break
            self._buf += data
            while b"\n" in self._buf:
                linea, self._buf = self._buf.split(b"\n", 1)
                if not linea.strip():
                    continue
                try:
                    m = json.loads(linea.decode("utf-8"))
                except ValueError:
                    continue
                if m.get("type") == "event":
                    self._despachar(m)
                else:
                    self._respuestas.put(m)
        self._vivo = False
        if self.on_close:
            try:
                self.on_close()
            except Exception:
                pass

    def _despachar(self, m):
        topico, data = m.get("topic"), m.get("data")
        if topico == "telemetry":
            try:
                self._muestras.put_nowait(data)
            except queue.Full:
                # Si el consumidor se retrasa se tira la mas vieja: en control
                # importa el dato NUEVO, no la cola historica.
                try:
                    self._muestras.get_nowait()
                    self._muestras.put_nowait(data)
                except queue.Empty:
                    pass
        for fn in self._callbacks.get(topico, ()):
            try:
                fn(data)
            except Exception as e:      # un callback roto no tumba el hilo
                print(f"[bus:{self.name}] error en callback de {topico}: {e}")

    # ---- comandos ------------------------------------------------------------
    def _cmd(self, msg):
        """
        Cada comando lleva un 'id' que el bus devuelve en su respuesta (ya lo
        soportaba: ClientConn._procesar lo hace eco). Sin esto habia un fallo
        sutil: si un comando expiraba por timeout, su respuesta TARDIA quedaba
        en la cola y se emparejaba con el SIGUIENTE comando (respuestas
        cruzadas: un publish "contestado" con el resultado de un get anterior).
        Ahora las respuestas atrasadas se reconocen por id y se descartan.
        """
        if not self._vivo:
            raise BusError("no conectado al bus")
        with self._rlock:
            self._rid += 1
            rid = self._rid
            msg = dict(msg, id=rid)
            with self._wlock:
                try:
                    self.sock.sendall((json.dumps(msg) + "\n").encode("utf-8"))
                except OSError as e:
                    raise BusError(f"envio fallido: {e}")
            limite = time.time() + self.timeout
            while True:
                restante = limite - time.time()
                if restante <= 0:
                    raise BusError("el bus no respondio")
                try:
                    r = self._respuestas.get(timeout=restante)
                except queue.Empty:
                    raise BusError("el bus no respondio")
                if r.get("id") in (None, rid):   # None: bus antiguo sin eco
                    break
                # respuesta atrasada de un comando anterior -> descartada
        if not r.get("ok", False):
            raise BusError(r.get("error", "error desconocido"))
        return r

    # ---- API -----------------------------------------------------------------
    def subscribe(self, topico, callback=None):
        self._callbacks.setdefault(topico, [])
        if callback:
            self._callbacks[topico].append(callback)
        return self._cmd({"cmd": "subscribe", "topics": [topico]})

    def unsubscribe(self, topico):
        self._callbacks.pop(topico, None)
        return self._cmd({"cmd": "unsubscribe", "topics": [topico]})

    def publish(self, topico, data):
        return self._cmd({"cmd": "publish", "topic": topico, "data": data})

    def get(self, topico):
        return self._cmd({"cmd": "get", "topic": topico}).get("data")

    def bus_info(self):
        r = self._cmd({"cmd": "info"})
        r.pop("ok", None)
        return r

    # ---- atajos de mando -----------------------------------------------------
    def tomar_mando(self):
        """Se declara duenno del PWM (los demas quedan bloqueados por el bus)."""
        return self.publish("mode", {"owner": self.name})

    def soltar_mando(self):
        return self.publish("mode", {"owner": "manual"})

    def set_pwm(self, pwm):
        return self.publish("control", {"pwm": int(max(0, min(255, pwm)))})

    def set_enable(self, on):
        return self.publish("enable", {"on": bool(on)})

    def set_setpoint(self, rpm):
        return self.publish("setpoint", {"rpm": float(rpm)})

    # ---- sincronizacion con el muestreo --------------------------------------
    def next_sample(self, timeout=10.0, vaciar=False):
        """
        Bloquea hasta la SIGUIENTE muestra del motor y la devuelve.
        vaciar=True descarta las acumuladas y devuelve solo la mas reciente
        (util al retomar el lazo tras una pausa larga).
        """
        if not self._vivo:
            raise BusError("el bus se cerro")
        m = self._muestras.get(timeout=timeout)
        if vaciar:
            while True:
                try:
                    m = self._muestras.get_nowait()
                except queue.Empty:
                    break
        return m

    def drain(self):
        while True:
            try:
                self._muestras.get_nowait()
            except queue.Empty:
                return


def conectar_o_avisar(name, host=HOST, port=PORT):
    """Conecta e imprime la ayuda estandar si el bus no esta levantado."""
    print(f"Conectando al bus {host}:{port} como '{name}' ...")
    try:
        c = BusClient(name, host, port).connect()
    except OSError as e:
        print(f"No se pudo conectar: {e}")
        print("Levanta primero el bus:")
        print("   python3 motor_bus.py --sim                 (sin hardware)")
        print("   python3 motor_bus.py --serial /dev/ttyACM0 (con Arduino)")
        return None
    print(f"  Conectado. bus={c.info}")
    return c


def main():
    ap = argparse.ArgumentParser(description="Monitor / mando manual del bus.")
    ap.add_argument("--pwm", type=int, help="toma el mando y aplica este PWM (0-255)")
    ap.add_argument("--sp", type=float, help="publica una consigna de rpm")
    ap.add_argument("--n", type=int, default=0, help="numero de muestras (0=infinito)")
    args = ap.parse_args()

    c = conectar_o_avisar("monitor")
    if c is None:
        return
    with c:
        c.subscribe("telemetry")
        c.subscribe("log", lambda d: print(f"  [arduino] {d['msg']}"))
        if args.sp is not None:
            c.set_setpoint(args.sp)
        if args.pwm is not None:
            c.tomar_mando()
            c.set_enable(True)
            c.set_pwm(args.pwm)
            print(f"  mando tomado, PWM={args.pwm}. Ctrl+C para soltar.")
        i = 0
        try:
            while True:
                m = c.next_sample()
                i += 1
                print(f"  t={m['t_ms']/1000:7.2f}s  pwm={m['pwm']:3d}  "
                      f"{m['volts']:.3f}V  rpm={m['rpm']:7.1f}  "
                      f"sp={m.get('setpoint', 0):7.1f}  "
                      f"{'ON ' if m['enabled'] else 'OFF'}")
                if args.n and i >= args.n:
                    break
        except KeyboardInterrupt:
            pass
        finally:
            if args.pwm is not None:
                c.set_pwm(0)
                c.set_enable(False)
                c.soltar_mando()
                print("\n  motor parado, mando liberado.")


if __name__ == "__main__":
    main()
