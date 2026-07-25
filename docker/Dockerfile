# Imagen base para los 4 servicios de motor_agent (bot, bus, subscriber, gui).
# A propósito NO copia el código fuente aquí -- el código y los archivos que
# el proyecto genera en caliente (qtable_*.json, .env) se montan como bind
# mount en docker-compose.yml. Así:
#   - Editar un .py en la Pi no exige rebuild, solo reiniciar el contenedor.
#   - Las tablas Q aprendidas sobreviven a un `docker compose down` /
#     rebuild de imagen, porque viven en el host.
#   - La imagen nunca contiene tu .env (secretos), porque nunca se copia.
# El modelo de Whisper, en cambio, SÍ vive horneado dentro de la imagen
# (ver el ARG WHISPER_MODEL_SIZE más abajo) -- es voluntario, no un
# archivo que el proyecto genere, así que no tiene sentido dejarlo
# depender de una descarga en caliente cada vez.
#
# debian:trixie-slim en vez de la imagen oficial python:3.13-slim: así
# coincide exactamente con el Python 3.13.5 que ya usas en la Pi, y apt
# resuelve python3-tk de verdad (necesario para motor_gui.py) -- la imagen
# oficial de Docker Hub para python no trae soporte de Tk compilado.
FROM debian:trixie-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-venv \
    python3-tk \
    ffmpeg \
    build-essential \
    ca-certificates \
    && update-ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Solo requirements.txt entra a la imagen -- así el layer de pip install
# (el mas lento) solo se invalida cuando cambian las dependencias, no cada
# vez que editas un .py.
COPY requirements.txt .

# El venv va en /opt/venv, NO en /app/.venv -- esto es a propósito y es
# crítico: docker-compose.yml monta el código del host sobre /app
# (volumes: - ..:/app), y ese bind mount REEMPLAZA por completo lo que
# haya ahí adentro de la imagen. Un venv en /app/.venv quedaría tapado
# por el bind mount en cuanto el contenedor arranca -- exactamente el
# ModuleNotFoundError que se vio en la práctica. /opt/venv está fuera de
# la ruta montada, así que sobrevive.
RUN python3 -m venv /opt/venv \
    && /opt/venv/bin/pip install --upgrade pip \
    && /opt/venv/bin/pip install -r requirements.txt python-dotenv

ENV PATH="/opt/venv/bin:$PATH"
# hf_xet (el backend acelerado de descargas de Hugging Face, que
# faster-whisper usa por debajo) se cuelga indefinidamente en 0 bytes en
# ciertas redes -- bug conocido y documentado, no algo de este proyecto.
# La metadata (config.json, tokenizer.json) baja bien porque no pasa por
# Xet; el peso real del modelo sí, y ahí se cuelga sin error. Forzamos el
# fallback al HTTP clásico de huggingface_hub, mas lento pero confiable.
ENV HF_HUB_DISABLE_XET=1
# /opt/hf_cache, NO /app/.hf_cache -- mismo motivo que el venv en
# /opt/venv: /app se tapa por completo con el bind mount del host en
# runtime, así que cualquier cosa horneada en la imagen bajo /app
# desaparece en cuanto arranca el contenedor.
ENV HF_HOME="/opt/hf_cache"

# Hornea el modelo de Whisper en la imagen -- así la primera nota de voz
# real no depende de la red ni del bug de hf_xet en tiempo de ejecución.
# Ajustable por máquina sin tocar este archivo, ej. en el escritorio:
#   docker compose build --build-arg WHISPER_MODEL_SIZE=medium
# IMPORTANTE: debe coincidir con WHISPER_MODEL_SIZE en el .env de esa
# misma máquina -- si no coinciden, bot.py en tiempo de ejecución pide un
# modelo distinto al horneado aquí, y lo vuelve a descargar en caliente
# (funciona igual, solo pierde la ventaja de tenerlo ya listo).
ARG WHISPER_MODEL_SIZE=small
ARG WHISPER_COMPUTE_TYPE=int8
RUN python3 -c "from faster_whisper import WhisperModel; WhisperModel('$WHISPER_MODEL_SIZE', device='cpu', compute_type='$WHISPER_COMPUTE_TYPE')"

# Sin CMD fijo: cada servicio en docker-compose.yml define su propio
# 'command' (bot.py, motor_bus.py, mongo_motor_subscriber.py, motor_gui.py).
