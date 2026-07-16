# Motor QET — Agente de control (RL/manual) + Telegram + MongoDB Atlas

## Arquitectura de carpetas

```
~/Documents/motor/
├── motor_bus.py                # Bus TCP/JSON <-> puerto serial del Arduino (ya lo tienes)
├── bus_client.py                # Cliente TCP mínimo del bus (compartido por los dos de abajo)
├── mongo_motor_subscriber.py   # Cliente del bus -> persiste en MongoDB Atlas
├── mongo_setup_motor.js        # Crea las colecciones en Atlas (una sola vez)
├── bot.py                      # Bot/agente de Telegram (consultas + cambio de modo + voz)
├── requirements.txt
├── .env.example                 # copiar a .env y completar
├── test_smoke.py
├── README.md
├── piper-voices/
│   ├── es_MX-claude-high.onnx   # voz de Piper (ya la tienes, copiada del huerto)
│   └── es_MX-claude-high.onnx.json
└── venv/                        # (no versionar)

# Scripts que TÚ ya tienes / vas a escribir aparte (no incluidos aquí):
#   - rl_motor_qlearning.py / rl_motor_sarsa_lambda.py / rl_motor_reinforce.py
#     (o control manual): corren en la misma laptop, se conectan a
#     motor_bus.py como clientes TCP.
#   - firmware Arduino Mega 2560 (.ino)
#   - motor_gui.py (GUI de control manual)
```


## Flujo de datos

```
Arduino Mega 2560 ──(serie, protocolo del firmware)──► motor_bus.py
                                                             │  (TCP/JSON, 127.0.0.1:8770)
                        ┌────────────────────────────────────┼────────────────────────────────────┐
                        │                                    │                                    │
                        ▼                                    ▼                                    ▼
        rl_motor_qlearning.py / sarsa_lambda /      mongo_motor_subscriber.py            bot.py (agente)
        reinforce.py / motor_gui.py (manual)        (persiste todo en Atlas)        (lee Atlas, publica
        (topico "control": pwm)                                                     "mode" en el bus)
                                                             │
                                                             ▼
                                                     MongoDB Atlas
                                            motor_telemetria (time series)
                                            control_eventos (cambios de modo)
```

Todo cliente conectado al bus puede **suscribirse** a tópicos (recibe el
último valor retenido al conectarse, y luego cada evento nuevo) y
**publicar** en ellos. El bus arbitra quién puede publicar en `control`
(solo el dueño actual del modo, o cualquiera si el modo es `free`) y aplica
un interlock de sobrevelocidad por encima de cualquier algoritmo.

## Tópicos del bus (ver el docstring de `motor_bus.py` para el detalle completo)

| Tópico         | Quién publica                          | Campos                                            |
|----------------|------------------------------------------|-----------------------------------------------------|
| `telemetry`    | el bus, a cada muestra del Arduino/sim  | `t_ms, seq, pwm, adc, volts, rpm, enabled, setpoint, owner` (setpoint y owner ya fusionados por el bus) |
| `control`      | quien tenga el mando (`owner`)          | `pwm` (0–255), `src`                                |
| `enable`       | quien tenga el mando                     | `on` (bool), `src`                                  |
| `setpoint`     | la GUI / el algoritmo                    | `rpm`, `src`                                        |
| `mode`         | `bot.py` (o la GUI)                      | `owner`: `manual` \| `qlearning` \| `sarsa_lambda` \| `reinforce` \| `free`, `src` |
| `agent/table`  | los algoritmos de RL                     | instantánea de su tabla/política                    |
| `agent/status` | los algoritmos de RL                     | `algo, phase, episode, epsilon, reward, states, ...` |
| `log`          | el bus                                   | mensajes crudos del Arduino                          |

`mongo_motor_subscriber.py` solo persiste `telemetry` (en `motor_telemetria`,
time series) y `mode` (en `control_eventos`, como evento discreto). El resto
de tópicos no se guardan por ahora.

## Voz en el bot (texto y notas de voz)

`bot.py` acepta tanto mensajes de texto como **notas de voz** de Telegram,
con el mismo patrón que `huerto-bot`:

- Entrada: nota de voz (.oga/Opus) → **faster-whisper** → texto.
- Ese texto pasa por la misma lógica que un mensaje escrito (detección de
  comando de cambio de modo primero, LLM si no es un comando).
- Salida: la respuesta → **Piper** → WAV → **ffmpeg** → .ogg/Opus → nota de
  voz de Telegram. Si la síntesis falla, responde en texto como respaldo.

Requisito de sistema (no de Python):

```bash
sudo apt install ffmpeg
```

La voz de Piper (`es_MX-claude-high.onnx` + `.onnx.json`) ya está en
`piper-voices/` en este repo (copiada del proyecto huerto). Si
`PIPER_VOICE_PATH` queda vacío en tu `.env`, el bot igual transcribe notas
de voz de entrada, pero responde en texto en vez de audio.

La primera vez que llegue una nota de voz, `faster-whisper` descarga el
modelo (`WHISPER_MODEL_SIZE=small` ≈ 500 MB) desde Hugging Face; luego
queda en caché. Los modelos de voz cargan de forma perezosa: el modo texto
no paga ese costo si nadie manda audio.

## Puesta en marcha

```bash
mkdir -p ~/Documents/motor
# copia aquí motor_bus.py, bus_client.py, mongo_motor_subscriber.py,
# mongo_setup_motor.js, bot.py, requirements.txt, .env.example, test_smoke.py,
# y la carpeta piper-voices/

cd ~/Documents/motor
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

cp .env.example .env
# completa TELEGRAM_BOT_TOKEN y MONGO_URI (mongodb+srv://... de Atlas)
# y, para la voz: WHISPER_MODEL_SIZE, WHISPER_COMPUTE_TYPE, WHISPER_LANGUAGE,
# PIPER_VOICE_PATH=piper-voices/es_MX-claude-high.onnx
```

**1. Crear las colecciones en Atlas (una sola vez):**
```bash
mongosh "mongodb+srv://usuario:password@cluster.mongodb.net/motor_qet" mongo_setup_motor.js
```

**2. Smoke test (sin hardware, sin Atlas, sin Telegram, sin bus real, sin
descargar Whisper/Piper):**
```bash
python3 test_smoke.py
```

**3. Arrancar el bus** (con hardware o simulado, para probar sin Arduino):
```bash
python3 motor_bus.py --sim                     # sin hardware
python3 motor_bus.py --serial /dev/ttyACM0     # con el Arduino real
```

**4. Arrancar el suscriptor de Mongo:**
```bash
export $(grep -v '^#' .env | xargs)
python3 mongo_motor_subscriber.py --mongo-uri "$MONGO_URI" --bus-host "$BUS_HOST" --bus-port "$BUS_PORT"
```

**5. Arrancar tu script de control (qlearning/sarsa_lambda/reinforce/manual)**
conectándose al bus en `BUS_HOST:BUS_PORT`.

**6. Arrancar el bot:**
```bash
export $(grep -v '^#' .env | xargs)
python3 bot.py
```

## Prueba rápida sin el Arduino

Con `motor_bus.py --sim` corriendo, ya tienes telemetría fluyendo (el
simulador reporta a la misma cadencia que el hardware real). Pregúntale al
bot por Telegram "¿cómo va el motor?" (por texto o por nota de voz) y
debería responder con el rpm y setpoint simulados.

Para probar el cambio de modo, dile al bot algo como "cambia el modo a
qlearning". Si en cambio dices solo "cambia el modo" o "modo de
aprendizaje por refuerzo" sin decir cuál algoritmo, el bot te va a
preguntar cuál (a propósito no adivina entre los tres algoritmos de RL) —
esto aplica igual si lo dices por voz.

## Probar la voz una vez corriendo

1. Corre `python3 bot.py` como en el paso 6.
2. En Telegram, en vez de escribir, **manda una nota de voz** preguntando
   algo, por ejemplo: *"¿cómo va el rpm?"*.
3. Deberías recibir de vuelta otra nota de voz con la respuesta.
4. Si en cambio recibes texto, revisa los logs: lo más común es que
   `PIPER_VOICE_PATH` esté vacío o mal escrito, o que falte `ffmpeg`.

## Pendiente / fuera de este entregable

- **Borrar los datos de prueba** insertados en `motor_telemetria` /
  `control_eventos` durante pruebas en vivo contra Atlas (no hay todavía un
  script tipo `--clear`; se puede hacer con `mongosh` directo mientras
  tanto).
- **GUI de control manual** (`motor_gui.py`): no incluida — falta definir
  framework (Tkinter/PyQt/web) y qué controles necesita antes de generarla.
- **`rl_motor_qlearning.py` / `rl_motor_sarsa_lambda.py` /
  `rl_motor_reinforce.py` / firmware del Arduino**: fuera de alcance según
  lo pedido; solo se ajustó el resto de piezas para hablar el protocolo
  real de `motor_bus.py` y los campos/modos que esos scripts van a publicar.
