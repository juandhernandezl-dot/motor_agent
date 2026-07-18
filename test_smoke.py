"""
Prueba de humo: valida bot.py y mongo_motor_subscriber.py sin necesitar
Telegram, Mongo Atlas real, motor_bus.py real, LM Studio real, ni los
modelos de Whisper/Piper descargados.
"""
import io
import json
import math
import os
import socket
import struct
import threading
import wave as wave_module
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, HTTPServer


def _puerto_libre() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


BUS_PORT_FALSO = _puerto_libre()

# Variables de entorno ANTES de importar bot.py (bot.py las lee al importarse)
os.environ["TELEGRAM_BOT_TOKEN"] = "dummy-token-para-test"
os.environ["LLM_BASE_URL"] = "http://localhost:8766/v1"
os.environ["LLM_MODEL"] = "qwen3.5-4b"
os.environ["BUS_HOST"] = "127.0.0.1"
os.environ["BUS_PORT"] = str(BUS_PORT_FALSO)
# Puerto que a propósito nunca responde: prueba que el bot no se caiga si
# Atlas no está disponible.
os.environ["MONGO_URI"] = "mongodb://localhost:1/"
# PIPER_VOICE_PATH se deja vacío a propósito: esta prueba NO carga Piper ni
# Whisper (requieren modelos descargados), solo la conversión de audio con
# ffmpeg, que es independiente de esos modelos.


class FakeLMStudioHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        json.loads(self.rfile.read(length))  # solo valida que sea JSON bien formado
        fake_reply = "RPM estable cerca del setpoint, todo en orden."
        response = {
            "choices": [{"message": {"role": "assistant", "content": fake_reply}, "finish_reason": "stop"}]
        }
        payload = json.dumps(response).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, format, *args):
        pass  # silenciar logs del servidor de prueba


def _fake_bus_server(port: int) -> None:
    """Bus falso: acepta UNA conexión, responde 'ok' a hello y a publish
    de 'mode'. Suficiente para probar bot._publicar_modo_sync sin correr
    motor_bus.py real (que necesita --sim o hardware)."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(1)
    conn, _ = srv.accept()
    with conn:
        buf = b""
        while b"\n" not in buf:
            buf += conn.recv(4096)
        linea, buf = buf.split(b"\n", 1)
        m = json.loads(linea.decode("utf-8"))
        assert m.get("cmd") == "hello"
        conn.sendall((json.dumps({"ok": True, "name": m.get("name")}) + "\n").encode("utf-8"))

        while b"\n" not in buf:
            buf += conn.recv(4096)
        linea, buf = buf.split(b"\n", 1)
        m = json.loads(linea.decode("utf-8"))
        assert m.get("cmd") == "publish" and m.get("topic") == "mode"
        conn.sendall((json.dumps({"ok": True, "topic": "mode"}) + "\n").encode("utf-8"))
    srv.close()


def main():
    lm_server = HTTPServer(("localhost", 8766), FakeLMStudioHandler)
    threading.Thread(target=lm_server.serve_forever, daemon=True).start()

    bus_thread = threading.Thread(target=_fake_bus_server, args=(BUS_PORT_FALSO,), daemon=True)
    bus_thread.start()

    import bot
    import mongo_motor_subscriber as sub

    print("--- detectar_comando_modo() ---")
    assert bot.detectar_comando_modo("cambia el modo a qlearning") == "qlearning"
    assert bot.detectar_comando_modo("pon el modo sarsa porfa") == "sarsa_lambda"
    assert bot.detectar_comando_modo("cambia a modo manual") == "manual"
    assert bot.detectar_comando_modo("cambia a modo pid") == "pid"
    assert bot.detectar_comando_modo("cambia a reinforce") == "reinforce"
    assert bot.detectar_comando_modo("déjalo en modo free") == "free"
    assert bot.detectar_comando_modo("¿cómo va el rpm?") is None
    # Ambigüedad a propósito: pide cambiar de modo pero no dice cuál de
    # los 3 algoritmos de RL -- el bot NO debe adivinar.
    assert bot.detectar_comando_modo("cambia el modo a aprendizaje por refuerzo") == "ambiguo"
    print("OK")

    print("\n--- format_status_for_llm() con Mongo Atlas caído (no debe lanzar excepción) ---")
    texto = bot.format_status_for_llm()
    print(texto)
    assert "No hay lecturas del motor" in texto

    print("\n--- ask_llm() contra el servidor LM Studio falso ---")
    reply = bot.ask_llm(chat_id=1, user_text="¿cómo va el motor?")
    print("Respuesta:", reply)
    assert reply

    print("\n--- enviar_comando_modo() contra el bus falso ---")
    bot._publicar_modo_sync("qlearning", chat_id=1)
    bus_thread.join(timeout=2)
    print("OK")

    print("\n--- procesar_mensaje_texto() (camino compartido texto/voz) ---")
    import asyncio

    respuesta_ambigua = asyncio.run(
        bot.procesar_mensaje_texto(chat_id=1, user_text="cambia el modo a aprendizaje por refuerzo")
    )
    assert "¿A cuál modo?" in respuesta_ambigua
    respuesta_pregunta = asyncio.run(
        bot.procesar_mensaje_texto(chat_id=1, user_text="¿cómo va el motor?")
    )
    assert respuesta_pregunta  # vino del LLM falso, ya probado arriba que responde algo
    print("OK")

    print("\n--- mongo_motor_subscriber.procesar_evento() ---")
    ahora = datetime.now(timezone.utc).timestamp()

    resultado = sub.procesar_evento({
        "type": "event", "topic": "telemetry", "seq": 1,
        "data": {
            "t_ms": 1000, "seq": 1, "pwm": 120, "adc": 512, "volts": 2.5,
            "rpm": 1450.3, "enabled": True, "t_pc": ahora,
            "setpoint": 1500.0, "owner": "qlearning",
        },
    })
    assert resultado is not None
    coleccion, doc = resultado
    assert coleccion == "motor_telemetria" and doc["rpm"] == 1450.3 and doc["owner"] == "qlearning"

    resultado = sub.procesar_evento({
        "type": "event", "topic": "mode", "seq": 2,
        "data": {"owner": "manual", "src": "telegram:1"},
    })
    assert resultado is not None
    coleccion, doc = resultado
    assert coleccion == "control_eventos" and doc["modo"] == "manual"

    resultado = sub.procesar_evento({
        "type": "event", "topic": "mode", "seq": 3,
        "data": {"owner": "pid", "src": "telegram:1"},
    })
    assert resultado is not None
    coleccion, doc = resultado
    assert coleccion == "control_eventos" and doc["modo"] == "pid"

    # El valor "retenido" que el bus entrega al suscribirse NO es un cambio
    # real de modo; no debe guardarse.
    assert sub.procesar_evento({
        "type": "event", "topic": "mode", "retained": True,
        "data": {"owner": "free", "src": "bus"},
    }) is None

    # owner inválido, tópico que no nos interesa, y respuesta a comando
    # (no evento): ninguno debe guardarse.
    assert sub.procesar_evento({"type": "event", "topic": "mode", "data": {"owner": "invalido"}}) is None
    assert sub.procesar_evento({"type": "event", "topic": "log", "data": {"msg": "#ok"}}) is None
    assert sub.procesar_evento({"ok": True, "name": "mongo_subscriber"}) is None
    print("OK")

    print("\n--- Conversión de audio WAV -> OGG/Opus (usada para responder con voz) ---")
    sample_rate = 22050
    wav_buf = io.BytesIO()
    with wave_module.open(wav_buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sample_rate)
        frames = bytearray()
        for i in range(sample_rate):
            val = int(3000 * math.sin(2 * math.pi * 440 * i / sample_rate))
            frames += struct.pack("<h", val)
        w.writeframes(bytes(frames))

    ogg_bytes = bot.wav_bytes_to_ogg_opus(wav_buf.getvalue())
    assert ogg_bytes[:4] == b"OggS", "La salida no tiene la firma OGG esperada"
    print(f"WAV sintético ({len(wav_buf.getvalue())} bytes) -> OGG/Opus ({len(ogg_bytes)} bytes) ✅")

    print(
        "\nNota: no se prueban aquí Whisper ni Piper de extremo a extremo "
        "(transcribir o sintetizar audio real) porque requieren los modelos "
        "descargados. Para probar la voz de verdad: correr bot.py y mandarle "
        "una nota de voz real por Telegram."
    )

    print("\n✅ Smoke test OK: detección de comandos (incluida la ambigüedad "
          "intencional de RL), estado con Mongo caído, ask_llm, camino "
          "compartido texto/voz, publicación de modo contra el bus falso, "
          "enrutamiento del suscriptor, y conversión de audio funcionan.")
    lm_server.shutdown()
    # Nota: los hilos de monitoreo que pymongo abre tras el intento fallido
    # a MONGO_URI (a propósito, sin Atlas real) son daemon threads, así que
    # no impiden que el proceso termine solo aquí.


if __name__ == "__main__":
    main()
