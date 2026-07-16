"""
Suscriptor TCP -> MongoDB Atlas para el Quanser QET DC Motor Control Trainer.

Se conecta como CLIENTE al bus (motor_bus.py), que expone un protocolo
TCP/JSON de publicacion/suscripcion en 127.0.0.1:8770 (ver el docstring de
motor_bus.py para el detalle completo). Reemplaza a la version anterior,
que era cliente WebSocket de nodo_comunicacion.py -- ese nodo ya no existe,
motor_bus.py es ahora el UNICO proceso que toca el puerto serie.

Este script SOLO escucha los topicos "telemetry" y "mode" y los persiste;
no controla el motor, no publica nada, no decide setpoints ni modos.

Simplificacion importante frente a la version anterior: cada muestra de
"telemetry" ya llega con "setpoint" y "owner" (el modo de control vigente)
fusionados por el propio bus (ver MotorBus._on_sample en motor_bus.py). Ya
no hace falta guardar telemetria y control como documentos separados ni
reconciliarlos despues -- el bus ya resolvio esa correspondencia temporal.

Enruta cada evento segun su topico:
    "telemetry" -> motor_telemetria (time series). Trae t_ms/seq/pwm/adc/
                   volts/rpm/enabled/setpoint/owner, tal cual los publica
                   el bus.
    "mode"      -> control_eventos (evento discreto: cambio de modo entre
                   manual/qlearning/sarsa_lambda/reinforce/free, quien lo
                   pidio y cuando). Se ignora el valor "retenido" que el
                   bus entrega automaticamente al suscribirse -- ese no es
                   un cambio real, es solo el estado que ya habia.
    cualquier otro -> se ignora (control, enable, setpoint, agent/table,
                      agent/status, log: no se persisten por ahora).

Uso:
    python3 mongo_motor_subscriber.py \
        --bus-host 127.0.0.1 --bus-port 8770 \
        --mongo-uri "mongodb+srv://usuario:password@cluster.mongodb.net" \
        --db motor_qet

Requisitos:
    pip install "pymongo[srv]"
    (bus_client.py no usa librerias externas; websockets ya no se necesita)

Principios aplicados:
- Simplicity First: un hilo lector + un hilo de flush periodico, sin
  asyncio; el bus ya fusiona setpoint/owner por muestra, asi que este
  suscriptor no reconcilia nada, solo enruta y guarda.
- Surgical Changes: este script SOLO escucha y guarda.
"""

from __future__ import annotations

import argparse
import json
import logging
import threading
import time
from collections import deque
from datetime import datetime, timezone
from typing import Any, Deque, Dict, Optional, Tuple

from pymongo import MongoClient
from pymongo.errors import PyMongoError

from bus_client import BusClient

logging.basicConfig(format="%(asctime)s [%(levelname)s] %(message)s", level=logging.INFO)
log = logging.getLogger("mongo_motor_subscriber")

# Debe coincidir con los "owner" validos de motor_bus.py / rl_motor_*.py.
MODOS_VALIDOS = {"manual", "qlearning", "sarsa_lambda", "reinforce", "free"}


def _parse_timestamp(t_pc: Any) -> datetime:
    """El bus manda t_pc como epoch en segundos (time.time() de Python).
    Si falta o viene raro, se usa la hora de llegada en vez de fallar."""
    if isinstance(t_pc, (int, float)):
        return datetime.fromtimestamp(t_pc, tz=timezone.utc)
    return datetime.now(timezone.utc)


def procesar_evento(msg: Dict[str, Any]) -> Optional[Tuple[str, Dict[str, Any]]]:
    """Convierte un mensaje del bus (ya deserializado de JSON) en
    (nombre_coleccion, doc) listo para insertar. None si no debe guardarse:
    no es un evento (es la respuesta a un comando como hello/subscribe),
    el topico no nos interesa, o es el valor "retenido" de bienvenida de
    "mode" (no un cambio real).
    """
    if msg.get("type") != "event":
        return None

    topic = msg.get("topic")
    data = msg.get("data") or {}

    if topic == "telemetry":
        doc: Dict[str, Any] = {
            "timestamp": _parse_timestamp(data.get("t_pc")),
            "t_ms": data.get("t_ms"),
            "seq": data.get("seq"),
            "pwm": data.get("pwm"),
            "adc": data.get("adc"),
            "volts": data.get("volts"),
            "rpm": data.get("rpm"),
            "enabled": data.get("enabled"),
            "setpoint": data.get("setpoint"),
            "owner": data.get("owner"),
        }
        return "motor_telemetria", doc

    if topic == "mode":
        if msg.get("retained"):
            return None
        owner = data.get("owner")
        if owner not in MODOS_VALIDOS:
            log.warning("evento 'mode' con owner invalido o ausente: %.200s", json.dumps(msg))
            return None
        doc = {
            "timestamp": datetime.now(timezone.utc),
            "tipo": "comando_modo",
            "modo": owner,
            "origen": data.get("src", "desconocido"),
        }
        return "control_eventos", doc

    return None  # control/enable/setpoint/agent/*/log: no se persisten aun


class MongoBatchWriter:
    """Acumula documentos de motor_telemetria en memoria y los inserta por
    lotes (a Ts=50ms por defecto el motor reporta ~20 Hz). control_eventos,
    al ser poco frecuente y discreto, se inserta de inmediato, uno por uno.
    """

    def __init__(
        self, mongo_uri: str, db: str, telemetria_col: str, eventos_col: str,
        batch_size: int, batch_interval: float,
    ):
        self.client = MongoClient(mongo_uri, serverSelectionTimeoutMS=5000)
        self.db = self.client[db]
        self.telemetria = self.db[telemetria_col]
        self.eventos = self.db[eventos_col]
        self.batch_size = batch_size
        self.batch_interval = batch_interval
        self.buffer: Deque[Dict[str, Any]] = deque()
        self.lock = threading.Lock()
        self._stop = threading.Event()

        self.client.admin.command("ping")
        log.info("Conectado a MongoDB Atlas (%s.[%s, %s]).", db, telemetria_col, eventos_col)

    def add(self, coleccion: str, doc: Dict[str, Any]) -> None:
        if coleccion == "control_eventos":
            try:
                self.eventos.insert_one(doc)
                log.info("Evento de control guardado: modo=%s origen=%s", doc.get("modo"), doc.get("origen"))
            except PyMongoError as e:
                log.error("Fallo al insertar evento de control: %s", e)
            return

        with self.lock:
            self.buffer.append(doc)
            listo_para_flush = len(self.buffer) >= self.batch_size
        if listo_para_flush:
            self.flush()

    def flush(self) -> None:
        with self.lock:
            if not self.buffer:
                return
            docs = list(self.buffer)
            self.buffer.clear()
        try:
            self.telemetria.insert_many(docs, ordered=False)
            log.info("Insertados %d documentos en motor_telemetria.", len(docs))
        except PyMongoError as e:
            log.error("Fallo al insertar lote en Mongo (%d docs perdidos): %s", len(docs), e)

    def flush_periodico(self) -> None:
        while not self._stop.is_set():
            time.sleep(self.batch_interval)
            self.flush()

    def detener(self) -> None:
        self._stop.set()
        self.flush()  # no perder lo que quedo en el buffer al salir


def escuchar_bus(bus_host: str, bus_port: int, writer: MongoBatchWriter) -> None:
    """Se conecta al bus y reintenta con backoff exponencial si se cae la
    conexion (motor_bus.py puede reiniciarse sin que este proceso deba
    morir con el)."""
    espera = 1.0
    while True:
        try:
            client = BusClient(bus_host, bus_port, name="mongo_subscriber")
            client.subscribe(["telemetry", "mode"])
            log.info("Conectado al bus: %s:%s", bus_host, bus_port)
            espera = 1.0
            for msg in client.events():
                resultado = procesar_evento(msg)
                if resultado is not None:
                    coleccion, doc = resultado
                    writer.add(coleccion, doc)
        except (OSError, ConnectionError, json.JSONDecodeError) as e:
            log.warning("Bus desconectado (%s). Reintentando en %.1fs...", e, espera)
            time.sleep(espera)
            espera = min(espera * 2, 30.0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bus-host", default="127.0.0.1", help="Host del bus (motor_bus.py)")
    parser.add_argument("--bus-port", type=int, default=8770, help="Puerto TCP del bus")
    parser.add_argument("--mongo-uri", required=True, help='ej. "mongodb+srv://user:pass@cluster.mongodb.net"')
    parser.add_argument("--db", default="motor_qet")
    parser.add_argument("--telemetria-collection", default="motor_telemetria")
    parser.add_argument("--eventos-collection", default="control_eventos")
    parser.add_argument("--batch-size", type=int, default=50, help="Documentos por lote antes de insertar")
    parser.add_argument("--batch-interval", type=float, default=2.0, help="Segundos max. antes de forzar un flush")
    args = parser.parse_args()

    writer = MongoBatchWriter(
        mongo_uri=args.mongo_uri,
        db=args.db,
        telemetria_col=args.telemetria_collection,
        eventos_col=args.eventos_collection,
        batch_size=args.batch_size,
        batch_interval=args.batch_interval,
    )
    threading.Thread(target=writer.flush_periodico, daemon=True).start()

    try:
        escuchar_bus(args.bus_host, args.bus_port, writer)
    except KeyboardInterrupt:
        log.info("Detenido por el usuario.")
    finally:
        writer.detener()


if __name__ == "__main__":
    main()
