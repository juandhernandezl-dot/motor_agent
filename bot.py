"""
Bot de Telegram / agente para el sistema de control del motor DC (Quanser
QET DC Motor Control Trainer, trabajo de grado).

Dos tipos de interaccion, en texto o en voz:
  1) Preguntas sobre el estado del motor ("¿cómo va el rpm?", "¿cuál es el
     setpoint?") -> se responden con un LLM local (LM Studio / Qwen3.5-4B),
     usando el estado real leido de MongoDB Atlas como contexto.
  2) Comandos de control ("cambia el modo a qlearning") -> se detectan con
     una regla simple (sin LLM de por medio) y se publican directo al bus
     (motor_bus.py) por TCP, topico "mode".

Voz (portada de huerto-bot/bot.py, mismo patron, Surgical Changes: solo se
copian las piezas de audio, la logica de dominio -- deteccion de modo,
bus, Mongo -- es la que ya tenia este bot):
- Entrada: notas de voz de Telegram (.oga/Opus) -> faster-whisper -> texto.
- Ese texto pasa por el MISMO camino que un mensaje escrito (deteccion de
  comando de modo primero, LLM si no es un comando).
- Salida: la respuesta -> Piper -> WAV -> ffmpeg -> .ogg/Opus -> Telegram.
  Si la sintesis de voz falla, se responde en texto como respaldo (nunca
  se deja al usuario sin respuesta).

Arquitectura real del bus (ver motor_bus.py):
  Arduino Mega <--serie--> motor_bus.py <--TCP 127.0.0.1:8770--> N clientes
  Este bot es un cliente mas: publica {"cmd":"publish","topic":"mode",
  "data":{"owner":...}} cuando alguien pide cambiar de modo.

Los "owner" validos del bus son: manual, pid, qlearning, sarsa_lambda,
reinforce, free (el motivo por el que se agrego "pid": ctl_pid.py toma el
mando con owner="pid", tal como los tres algoritmos de RL usan su propia
clave; faltaba en esta lista y el bot no podia cambiar a ese modo).

Principios de Karpathy:
- Think Before Coding: si piden "modo" sin decir cual de los 5, el bot NO
  adivina; se le pregunta al usuario cual quiere (aplica igual si la
  pregunta llega por voz).
- Simplicity First: la deteccion de comando sigue siendo un match de
  palabras clave; la voz reusa exactamente las funciones de audio de
  huerto-bot (STT/TTS/ffmpeg), sin reescribirlas "mejor" sin necesidad.
- Surgical Changes: unico cambio real para agregar voz es (a) las
  funciones de audio, (b) un handler nuevo `handle_voice`, y (c) que la
  logica de "que responder" ahora vive en una funcion compartida
  (`procesar_mensaje_texto`) para no duplicarla entre texto y voz.

Pendiente (fuera de este cambio, a proposito):
- Confirmacion de comandos: se deja para cuando haya pruebas reales.
"""

from __future__ import annotations

import asyncio
import io
import logging
import os
import shutil
import subprocess
import tempfile
import wave
from collections import defaultdict, deque
from datetime import datetime, timezone
from typing import Deque, Dict, Optional

from dotenv import load_dotenv
from faster_whisper import WhisperModel
from openai import OpenAI
from piper import PiperVoice
from pymongo import MongoClient
from pymongo.errors import PyMongoError
from telegram import Update
from telegram.ext import (
    Application,
    CommandHandler,
    ContextTypes,
    MessageHandler,
    filters,
)

from bus_client import BusClient

# Carga el .env directamente (busca en el directorio actual y en los padres).
# Antes había que hacer `export $(grep -v '^#' .env | xargs)` a mano en cada
# terminal nueva; si se te olvidaba tras editar el .env, el bot arrancaba
# con variables viejas o vacías (eso fue lo que pasó con PIPER_VOICE_PATH:
# el bot corrió sin la variable, no porque el código de síntesis fallara).
# Por default no pisa variables ya exportadas a mano en esa terminal.
load_dotenv()

# --------------------------------------------------------------------------
# Configuración (todo por variables de entorno; ver .env.example)
# --------------------------------------------------------------------------
TELEGRAM_BOT_TOKEN = os.environ["TELEGRAM_BOT_TOKEN"]  # obligatorio, sin default
LLM_BASE_URL = os.environ.get("LLM_BASE_URL", "http://localhost:1234/v1")
LLM_MODEL = os.environ.get("LLM_MODEL", "qwen3.5-4b")
LLM_API_KEY = os.environ.get("LLM_API_KEY", "lm-studio")  # LM Studio ignora el valor
MAX_HISTORY_TURNS = int(os.environ.get("MAX_HISTORY_TURNS", "4"))
REQUEST_TIMEOUT_S = int(os.environ.get("REQUEST_TIMEOUT_S", "60"))

# Bus de comunicación (motor_bus.py) -- TCP/JSON, no WebSocket.
BUS_HOST = os.environ.get("BUS_HOST", "127.0.0.1")
BUS_PORT = int(os.environ.get("BUS_PORT", "8770"))
BUS_TIMEOUT_S = float(os.environ.get("BUS_TIMEOUT_S", "3"))

# --------------------------------------------------------------------------
# Configuración de voz (STT: faster-whisper / TTS: Piper) -- igual que
# huerto-bot/bot.py, mismos nombres de variable de entorno.
# --------------------------------------------------------------------------
WHISPER_MODEL_SIZE = os.environ.get("WHISPER_MODEL_SIZE", "small")
WHISPER_COMPUTE_TYPE = os.environ.get("WHISPER_COMPUTE_TYPE", "int8")  # int8 = rápido en CPU
WHISPER_LANGUAGE = os.environ.get("WHISPER_LANGUAGE", "es")

# Ruta al modelo .onnx de Piper (el .onnx.json debe estar en la misma carpeta).
# En este repo ya está en piper-voices/es_MX-claude-high.onnx.
PIPER_VOICE_PATH = os.environ.get("PIPER_VOICE_PATH", "")

# --------------------------------------------------------------------------
# Configuración de MongoDB Atlas (telemetría real del motor)
# --------------------------------------------------------------------------
MONGO_URI = os.environ["MONGO_URI"]  # mongodb+srv://..., obligatorio, sin default
MONGO_DB = os.environ.get("MONGO_DB", "motor_qet")
MONGO_COLLECTION = os.environ.get("MONGO_COLLECTION", "motor_telemetria")
# El motor reporta a decenas de Hz: "desactualizado" se mide en segundos.
TELEMETRY_STALE_SECONDS = int(os.environ.get("TELEMETRY_STALE_SECONDS", "10"))

# Debe coincidir con los "owner" válidos de motor_bus.py / rl_motor_*.py.
MODOS_VALIDOS = ("manual", "pid", "qlearning", "sarsa_lambda", "reinforce", "free")
# Sinónimos en español para detectar el comando sin ambigüedad artificial.
_SINONIMOS_MODO = {
    "manual": ("manual",),
    "pid": ("pid",),
    "qlearning": ("qlearning", "q-learning", "q learning"),
    "sarsa_lambda": ("sarsa",),
    "reinforce": ("reinforce",),
    "free": ("free", "libre", "sin dueño", "sin dueno", "sin mando"),
}

SYSTEM_PROMPT = (
    "Eres el asistente del sistema de control del motor DC (Quanser QET "
    "Motor Control Trainer) de un trabajo de grado. Respondes en español, "
    "de forma clara y breve (máximo 3 oraciones), a alguien que conoce el "
    "proyecto y quiere una lectura rápida del estado del motor sin mirar "
    "el dashboard. No inventes valores que no te den; si falta un dato, dilo. "
    "Recibirás el estado ACTUAL del motor (rpm medido, pwm aplicado, "
    "voltaje del tacómetro, setpoint) y el modo de control vigente (uno de "
    "manual, pid, qlearning, sarsa_lambda, reinforce o free). Para cambiar el "
    "modo de control el usuario debe pedirlo explícitamente y decir cuál "
    "('cambia a modo qlearning'); eso NO lo decides tú ni lo simulas en la "
    "respuesta, ya lo maneja el sistema por fuera de ti."
)

logging.basicConfig(
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    level=logging.INFO,
)
log = logging.getLogger("motor-bot")

llm_client = OpenAI(base_url=LLM_BASE_URL, api_key=LLM_API_KEY)

if shutil.which("ffmpeg") is None:
    log.warning(
        "ffmpeg no está instalado (sudo apt install ffmpeg): las respuestas "
        "de voz no se podrán convertir al formato que exige Telegram."
    )

# Los modelos de voz se cargan de forma perezosa (solo la primera vez que
# llegue una nota de voz), no al arrancar -- mismo motivo que en
# huerto-bot: el modo texto no debe depender de ellos ni pagar el costo de
# cargar ~500MB de Whisper si nunca se usa voz.
_whisper_model: Optional[WhisperModel] = None
_piper_voice: Optional[PiperVoice] = None


def get_whisper_model() -> WhisperModel:
    global _whisper_model
    if _whisper_model is None:
        log.info("Cargando modelo de voz a texto (faster-whisper, %s)...", WHISPER_MODEL_SIZE)
        _whisper_model = WhisperModel(
            WHISPER_MODEL_SIZE, device="cpu", compute_type=WHISPER_COMPUTE_TYPE
        )
    return _whisper_model


def get_piper_voice() -> PiperVoice:
    global _piper_voice
    if _piper_voice is None:
        if not PIPER_VOICE_PATH:
            raise RuntimeError("PIPER_VOICE_PATH no está configurado en .env")
        log.info("Cargando voz de Piper desde %s...", PIPER_VOICE_PATH)
        _piper_voice = PiperVoice.load(PIPER_VOICE_PATH)
    return _piper_voice


_mongo_client: Optional[MongoClient] = None


def get_mongo_client() -> MongoClient:
    global _mongo_client
    if _mongo_client is None:
        # Timeout corto: si Atlas no responde, el bot no debe quedarse
        # colgado -- mejor seguir sin datos que trabar la respuesta.
        _mongo_client = MongoClient(MONGO_URI, serverSelectionTimeoutMS=3000)
    return _mongo_client


# Historial corto por chat (en memoria; se pierde al reiniciar el bot).
_history: Dict[int, Deque[dict]] = defaultdict(
    lambda: deque(maxlen=MAX_HISTORY_TURNS * 2)
)


# --------------------------------------------------------------------------
# Lectura de estado (MongoDB Atlas)
# --------------------------------------------------------------------------
def get_latest_telemetry() -> Optional[dict]:
    """Último documento de motor_telemetria. Ya viene con setpoint y owner
    (modo vigente) fusionados por el bus (ver motor_bus.py), así que no
    hace falta una segunda consulta para el modo o el setpoint. None si
    Mongo no responde o no hay datos todavía."""
    try:
        client = get_mongo_client()
        return client[MONGO_DB][MONGO_COLLECTION].find_one({}, sort=[("timestamp", -1)])
    except PyMongoError as e:
        log.warning("No se pudo leer telemetría de MongoDB: %s", e)
        return None


def format_status_for_llm() -> str:
    """Convierte el estado crudo de Mongo en una descripción en texto que
    el LLM pueda usar de contexto (no se le manda el JSON crudo)."""
    doc = get_latest_telemetry()
    if doc is None:
        return "No hay lecturas del motor disponibles en este momento."

    edad_s = (
        datetime.now(timezone.utc) - doc["timestamp"].replace(tzinfo=timezone.utc)
    ).total_seconds()

    partes = [
        f"Hace {edad_s:.1f}s: rpm medido {doc.get('rpm')}, pwm aplicado "
        f"{doc.get('pwm')}, voltaje del tacómetro {doc.get('volts')}, "
        f"habilitado: {doc.get('enabled')}."
    ]
    if edad_s > TELEMETRY_STALE_SECONDS:
        partes.append(
            f"AVISO: este dato tiene más de {TELEMETRY_STALE_SECONDS}s, podría estar desactualizado."
        )

    setpoint = doc.get("setpoint")
    partes.append(
        f"Setpoint vigente: {setpoint} rpm." if setpoint is not None
        else "No hay setpoint registrado todavía."
    )

    partes.append(f"Modo de control vigente: {doc.get('owner') or 'desconocido'}.")

    return " ".join(partes)


# --------------------------------------------------------------------------
# LLM
# --------------------------------------------------------------------------
def ask_llm(chat_id: int, user_text: str) -> str:
    """Envía el mensaje del usuario + historial corto + estado actual del
    motor al LLM local. El estado se refresca en cada llamada y NO se guarda
    en el historial (para que el siguiente turno vea siempre el dato nuevo).
    """
    status_context = format_status_for_llm()

    # El estado va DESPUES del historial, justo antes de la pregunta nueva
    # (no al principio): con un modelo chico como Qwen3.5-4B, lo que esta
    # mas cerca de la pregunta pesa mas que un system message de varios
    # turnos atras. Si el estado quedara arriba, una respuesta vieja del
    # historial (ej. "no tengo datos", de un turno anterior sin Mongo)
    # queda mas cerca de la pregunta que el dato fresco, y el modelo repite
    # la negativa vieja en vez de leer el estado actual.
    messages = [{"role": "system", "content": SYSTEM_PROMPT}]
    messages.extend(_history[chat_id])
    messages.append({"role": "system", "content": f"Estado actual: {status_context}"})
    messages.append({"role": "user", "content": user_text})

    response = llm_client.chat.completions.create(
        model=LLM_MODEL,
        messages=messages,
        temperature=0.5,
        top_p=0.8,
        max_tokens=250,
        timeout=REQUEST_TIMEOUT_S,
    )
    reply = response.choices[0].message.content.strip()

    _history[chat_id].append({"role": "user", "content": user_text})
    _history[chat_id].append({"role": "assistant", "content": reply})

    return reply


# --------------------------------------------------------------------------
# Comandos de control (cambio de modo) -- deterministas, sin LLM de por medio
# --------------------------------------------------------------------------
def detectar_comando_modo(texto: str) -> Optional[str]:
    """Devuelve uno de MODOS_VALIDOS, "ambiguo" si el texto pide un cambio
    de modo pero no dice a cuál de los 5, o None si no pide cambiar de modo.

    Regla simple a propósito (ver docstring del módulo): controlar hardware
    real no debe depender de que un LLM interprete bien la intención. Y a
    propósito NO se adivina en caso de ambigüedad: hay tres algoritmos de
    RL distintos (qlearning/sarsa_lambda/reinforce) y adivinar mal
    arrancaría el que no es. Aplica igual si el texto viene de una nota de
    voz transcrita.
    """
    t = texto.lower()
    pide_cambio = "modo" in t or "cambia" in t or "cambiar" in t or "pon" in t
    if not pide_cambio:
        return None
    for modo, sinonimos in _SINONIMOS_MODO.items():
        if any(s in t for s in sinonimos):
            return modo
    return "ambiguo"


def _publicar_modo_sync(modo: str, chat_id: int) -> None:
    """Conexión de una sola vez (no persistente) al bus: estos comandos son
    esporádicos, no de alta frecuencia, así que no vale la pena mantener
    una conexión abierta todo el tiempo. Bloqueante a propósito -- se llama
    siempre desde un hilo aparte (ver enviar_comando_modo)."""
    client = BusClient(BUS_HOST, BUS_PORT, name=f"telegram:{chat_id}", timeout=BUS_TIMEOUT_S)
    try:
        respuesta = client.publish("mode", {"owner": modo})
        if not respuesta.get("ok", False):
            raise RuntimeError(respuesta.get("error", "el bus rechazó el comando"))
    finally:
        client.close()


async def enviar_comando_modo(modo: str, chat_id: int) -> None:
    """Publica el cambio de modo en el bus sin bloquear el loop de asyncio
    de python-telegram-bot (BusClient usa sockets bloqueantes a propósito,
    ver bus_client.py -- Simplicity First: no hace falta un cliente TCP
    asíncrono para un comando esporádico de una sola línea)."""
    await asyncio.to_thread(_publicar_modo_sync, modo, chat_id)


# --------------------------------------------------------------------------
# Lógica compartida entre texto y voz: qué responder a un mensaje del
# usuario, ya sea escrito o transcrito de una nota de voz.
# --------------------------------------------------------------------------
async def procesar_mensaje_texto(chat_id: int, user_text: str) -> str:
    """Decide si `user_text` es un comando de cambio de modo o una
    pregunta para el LLM, actúa en consecuencia, y devuelve la respuesta
    en texto (que el handler de texto o de voz se encarga de entregar en
    el formato que corresponda)."""
    modo_pedido = detectar_comando_modo(user_text)

    if modo_pedido == "ambiguo":
        return "¿A cuál modo? Puedo cambiar a: manual, pid, qlearning, sarsa_lambda, reinforce o free."

    if modo_pedido is not None:
        try:
            await enviar_comando_modo(modo_pedido, chat_id)
            return f"Listo, mandé la orden de cambiar a modo {modo_pedido}."
        except Exception:
            log.exception("Fallo al publicar 'mode' en el bus")
            return "No pude comunicarme con el bus para cambiar el modo. ¿Está corriendo motor_bus.py?"

    try:
        return ask_llm(chat_id, user_text)
    except Exception:
        log.exception("Fallo al consultar el LLM local (¿está LM Studio encendido?)")
        return "No pude pensar bien la respuesta en este momento. Intenta de nuevo."


# --------------------------------------------------------------------------
# Voz: STT (faster-whisper) y TTS (Piper + ffmpeg) -- portado tal cual de
# huerto-bot/bot.py, son funciones genéricas de audio, sin nada específico
# del dominio del huerto ni del motor.
# --------------------------------------------------------------------------
def transcribe_audio(audio_path: str) -> str:
    """Transcribe un archivo de audio (cualquier formato que ffmpeg soporte,
    incluido el .oga/Opus de las notas de voz de Telegram) a texto en español.
    """
    segments, _info = get_whisper_model().transcribe(
        audio_path, language=WHISPER_LANGUAGE, beam_size=1
    )
    # segments es un generador: hay que consumirlo para que se genere el texto.
    return " ".join(segment.text for segment in segments).strip()


def wav_bytes_to_ogg_opus(wav_bytes: bytes) -> bytes:
    """Convierte WAV a .ogg/Opus, el único formato que Telegram acepta para que
    un mensaje se muestre como nota de voz (send_voice) en vez de como archivo.
    """
    proc = subprocess.run(
        [
            "ffmpeg", "-y",
            "-i", "pipe:0",
            "-c:a", "libopus",
            "-b:a", "32k",
            "-ar", "48000",
            "-f", "ogg",
            "pipe:1",
        ],
        input=wav_bytes,
        capture_output=True,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"ffmpeg falló: {proc.stderr.decode(errors='ignore')[-300:]}")
    return proc.stdout


def synthesize_speech_ogg(text: str) -> bytes:
    """Convierte texto a una nota de voz .ogg/Opus lista para enviar por Telegram
    (Piper genera WAV; wav_bytes_to_ogg_opus hace la conversión final)."""
    voice = get_piper_voice()

    wav_buffer = io.BytesIO()
    with wave.open(wav_buffer, "wb") as wav_file:
        voice.synthesize_wav(text, wav_file)

    return wav_bytes_to_ogg_opus(wav_buffer.getvalue())


# --------------------------------------------------------------------------
# Handlers de Telegram
# --------------------------------------------------------------------------
async def start(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    await update.message.reply_text(
        "¡Hola! Soy el agente del motor DC (QET). Pregúntame por el estado "
        "(rpm, pwm, setpoint) o pídeme cambiar el modo de control, por "
        "ejemplo: 'cambia el modo a qlearning' (también: manual, "
        "pid, sarsa_lambda, reinforce, free). Puedes escribirme o mandarme una "
        "nota de voz."
    )


async def handle_text(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    chat_id = update.effective_chat.id
    reply = await procesar_mensaje_texto(chat_id, update.message.text)
    await update.message.reply_text(reply)


async def handle_voice(update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    chat_id = update.effective_chat.id
    tmp_path = None

    try:
        with tempfile.NamedTemporaryFile(suffix=".oga", delete=False) as tmp:
            tmp_path = tmp.name

        tg_file = await update.message.voice.get_file()
        await tg_file.download_to_drive(tmp_path)

        user_text = transcribe_audio(tmp_path)
        if not user_text:
            await update.message.reply_text(
                "No logré entender el audio. ¿Puedes intentar de nuevo, por favor?"
            )
            return

        reply = await procesar_mensaje_texto(chat_id, user_text)

    except Exception:
        log.exception("Fallo al procesar la nota de voz")
        await update.message.reply_text(
            "No pude procesar el audio. Intenta de nuevo o escribe tu pregunta."
        )
        return
    finally:
        if tmp_path and os.path.exists(tmp_path):
            os.remove(tmp_path)

    # Responder en voz (mismo canal que usó el usuario); si la síntesis
    # falla, el usuario igual recibe la respuesta en texto en vez de
    # quedarse sin nada (incluye el caso de un comando de cambio de modo:
    # también se confirma por voz).
    try:
        ogg_bytes = synthesize_speech_ogg(reply)
        await update.message.reply_voice(voice=io.BytesIO(ogg_bytes))
    except Exception:
        log.exception("Falló la síntesis de voz; respondo en texto como respaldo")
        await update.message.reply_text(reply)


def main() -> None:
    app = Application.builder().token(TELEGRAM_BOT_TOKEN).build()
    app.add_handler(CommandHandler("start", start))
    app.add_handler(MessageHandler(filters.TEXT & ~filters.COMMAND, handle_text))
    app.add_handler(MessageHandler(filters.VOICE, handle_voice))

    log.info("Bot iniciado. Escuchando mensajes de Telegram...")
    app.run_polling()


if __name__ == "__main__":
    main()
