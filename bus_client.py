"""
Cliente minimo del BUS DE COMUNICACION (motor_bus.py).

motor_bus.py habla un protocolo TCP muy simple: una linea JSON por mensaje,
UTF-8, terminada en "\n" (ver el docstring de motor_bus.py para el detalle
completo de comandos y topicos). Este modulo SOLO envuelve ese protocolo en
una clase chica -- no reimplementa nada del bus ni valida reglas de
arbitraje (eso es responsabilidad exclusiva de motor_bus.py).

Se usa desde dos lugares con necesidades distintas:
  - mongo_motor_subscriber.py: conexion larga, se suscribe y lee eventos
    para siempre con BusClient.events().
  - bot.py: conexion corta, solo para publicar un cambio de modo puntual
    con BusClient.publish(...).

Principio Simplicity First: sin dependencias externas (solo socket/json de
la libreria estandar), sin reconexion automatica aqui -- eso lo maneja
quien use el cliente (ver escuchar_bus() en mongo_motor_subscriber.py),
porque solo el que escucha por mucho tiempo la necesita.
"""

from __future__ import annotations

import json
import socket
from typing import Any, Dict, Iterator, Iterable, Optional


class BusClient:
    """Conexion TCP a motor_bus.py con un saludo ("hello") ya hecho."""

    def __init__(self, host: str, port: int, name: str,
                 timeout: Optional[float] = None) -> None:
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self._buf = b""
        self.hello(name)

    def _send(self, msg: Dict[str, Any]) -> None:
        self.sock.sendall((json.dumps(msg) + "\n").encode("utf-8"))

    def _recv_line(self) -> Dict[str, Any]:
        """Bloquea hasta tener una linea completa y la devuelve parseada.
        Puede ser tanto la respuesta a un comando como un evento asincrono
        (el bus no garantiza el orden entre ambos, ver motor_bus.py)."""
        while b"\n" not in self._buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("el bus cerro la conexion")
            self._buf += chunk
        linea, self._buf = self._buf.split(b"\n", 1)
        return json.loads(linea.decode("utf-8"))

    def hello(self, name: str) -> Dict[str, Any]:
        self._send({"cmd": "hello", "name": name})
        return self._recv_line()

    def subscribe(self, topics: Iterable[str]) -> None:
        """Pide suscripcion a `topics`. No espera ni valida la respuesta:
        el bus puede mandar primero los valores retenidos de bienvenida y
        despues el "ok" del comando (o al reves); ambos se leen igual en
        events(), no vale la pena distinguirlos aqui."""
        self._send({"cmd": "subscribe", "topics": list(topics)})

    def publish(self, topic: str, data: Dict[str, Any]) -> Dict[str, Any]:
        """Publica y devuelve la respuesta directa del bus ({"ok": ...}).
        Solo pensado para uso de "publicacion corta" (ver bot.py); si hay
        eventos de otros topicos en vuelo se podria leer uno por error,
        pero en ese uso puntual no hay ninguna suscripcion activa."""
        self._send({"cmd": "publish", "topic": topic, "data": data})
        return self._recv_line()

    def get(self, topic: str) -> Dict[str, Any]:
        self._send({"cmd": "get", "topic": topic})
        return self._recv_line()

    def events(self) -> Iterator[Dict[str, Any]]:
        """Generador infinito de mensajes entrantes, para usar tras
        subscribe(). Termina lanzando ConnectionError si se cae el socket."""
        while True:
            yield self._recv_line()

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass
