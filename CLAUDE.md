# CLAUDE.md — Proyecto motor QET (trabajo de grado)

Contexto para Claude Code sobre `~/Documents/motor/`. Lee esto antes de tocar
código en este repo.

## Qué es esto

Control (manual y RL: Q-learning, SARSA(λ), REINFORCE) de un Quanser QET DC
Motor Control Trainer, con Arduino Mega 2560, un bus de comunicación en
Python, persistencia en MongoDB Atlas, y un bot de Telegram (texto y voz)
como agente de consulta/control. Todo corre en una laptop Ubuntu 22.04, en
VS Code, con `venv`.

Proyecto hermano (otro repo, mismo autor): `~/Documents/huerto-ml` y
`~/Documents/huerto-bot` (LSTM de riego + bot de Telegram). Este proyecto
copió su patrón de arquitectura (LLM local + Mongo + historial en memoria,
y ahora también voz Whisper/Piper) pero es un dominio distinto; no
compartir código directamente sin revisar que aplique.

## Arquitectura real (NO la primera versión con WebSocket)

```
Arduino Mega 2560 ──(serie)──► motor_bus.py ──(TCP/JSON, 127.0.0.1:8770)──►
    ├── rl_motor_qlearning.py / sarsa_lambda / reinforce.py / motor_gui.py
    │     (publican "control": pwm; consumen "telemetry")
    ├── mongo_motor_subscriber.py  (persiste todo en Atlas)
    └── bot.py  (lee Atlas, publica "mode" en el bus; texto y voz)
```

`motor_bus.py` es el **único** proceso que toca el puerto serie. Es un
broker publicador/suscriptor: los clientes hablan TCP con una línea JSON
por mensaje (`{"cmd":...}` → respuesta; eventos asíncronos `{"type":"event",
"topic":...,"data":...}`).

**Ya NO existe** `nodo_comunicacion.py` (versión anterior por WebSocket).
Si aparece código o docs que lo mencionen, están obsoletos.

### Tópicos del bus

| Tópico | Quién publica | Campos |
|---|---|---|
| `telemetry` | el bus, a cada muestra | `t_ms,seq,pwm,adc,volts,rpm,enabled,setpoint,owner` (setpoint y owner ya fusionados por el bus) |
| `control` | el dueño actual (`owner`) | `pwm` (0-255), `src` |
| `enable` | el dueño actual | `on` (bool), `src` |
| `setpoint` | GUI o algoritmo | `rpm`, `src` |
| `mode` | `bot.py` o la GUI | `owner`: uno de los modos válidos, `src` |
| `agent/table`, `agent/status`, `log` | los algoritmos / el bus | ver docstring de `motor_bus.py` |

**Modos válidos (`owner`)**: `manual`, `qlearning`, `sarsa_lambda`,
`reinforce`, `free`. No son genéricos tipo "rl"/"pid" — hay tres algoritmos
de RL distintos y el modo debe decir cuál.

Arbitraje: el bus solo acepta `control` cuyo `src == owner` (o si
`owner == "free"`). Interlock de sobrevelocidad (`--rpm-max`) por encima de
cualquier algoritmo. Keepalive cada 500ms; watchdog del Arduino corta el
motor a los 1.5s sin señal. `--sim` simula el motor sin hardware.

## Esquema de MongoDB Atlas

DB `motor_qet`:
- `motor_telemetria` (time series, `timeField: timestamp`, granularidad
  `seconds`): un doc por muestra de `telemetry`, campos tal cual arriba.
- `control_eventos`: un doc por cambio de modo real (no por el valor
  "retenido" que el bus manda al suscribirse). Campos: `timestamp, tipo:
  "comando_modo", modo, origen` (ej. `"telegram:<chat_id>"`).

`mongo_setup_motor.js` crea ambas colecciones (correr una sola vez contra
Atlas). `mongo_motor_subscriber.py` es el único escritor.

## Archivos del repo y su rol

- `motor_bus.py` — el bus (dado, no generado en esta conversación).
- `bus_client.py` — cliente TCP mínimo del bus (solo stdlib), usado por
  `mongo_motor_subscriber.py` (conexión larga, `events()`) y `bot.py`
  (conexión corta, `publish()`).
- `mongo_motor_subscriber.py` — cliente del bus → Mongo Atlas. Enruta
  `telemetry` → `motor_telemetria` (por lotes) y `mode` → `control_eventos`
  (uno por uno, ignorando el retenido). Todo lo demás se ignora por ahora.
- `bot.py` — bot de Telegram, texto **y voz**. Función compartida
  `procesar_mensaje_texto(chat_id, texto)` decide entre dos caminos: (1)
  preguntas de estado → LLM local (LM Studio/Qwen) con contexto real leído
  de Mongo; (2) comandos de cambio de modo → regla determinista (sin LLM),
  publicado directo al bus. Si pide cambiar de modo sin decir cuál de los
  3 algoritmos de RL, el bot pregunta en vez de adivinar (a propósito).
  `handle_text` y `handle_voice` llaman a esa misma función; `handle_voice`
  además transcribe con faster-whisper y sintetiza la respuesta con Piper +
  ffmpeg (con texto como respaldo si la síntesis falla).
- `mongo_setup_motor.js` — crea las colecciones en Atlas.
- `test_smoke.py` — prueba sin Telegram/Atlas/bus/LM Studio reales ni
  modelos de Whisper/Piper descargados (bus y LLM falsos levantados en el
  propio test). Cubre detección de comandos (incluida la ambigüedad
  intencional), lectura de estado con Mongo caído, `ask_llm`,
  `procesar_mensaje_texto` (camino compartido texto/voz), publicación de
  modo, `procesar_evento` del suscriptor, y la conversión de audio
  WAV→OGG/Opus (no prueba Whisper/Piper de punta a punta: eso requiere
  correr el bot real con una nota de voz).
- `requirements.txt` — incluye `faster-whisper` y `piper-tts` además de las
  dependencias base.
- `README.md` — incluye instrucciones de voz (`ffmpeg`, variables de
  entorno, cómo probarla).
- `piper-voices/es_MX-claude-high.onnx(.json)` — voz de Piper (copiada del
  proyecto huerto), ya en uso por `bot.py`.

## Variables de entorno (`.env`) que usa `bot.py`

Obligatorias: `TELEGRAM_BOT_TOKEN`, `MONGO_URI` (Atlas, `mongodb+srv://...`).

Con default razonable si se omiten: `LLM_BASE_URL`, `LLM_MODEL`,
`LLM_API_KEY`, `MAX_HISTORY_TURNS`, `REQUEST_TIMEOUT_S`, `BUS_HOST`,
`BUS_PORT`, `BUS_TIMEOUT_S`, `MONGO_DB`, `MONGO_COLLECTION`,
`TELEMETRY_STALE_SECONDS`.

De voz (mismos nombres que en `huerto-bot`): `WHISPER_MODEL_SIZE` (default
`small`), `WHISPER_COMPUTE_TYPE` (default `int8`), `WHISPER_LANGUAGE`
(default `es`), `PIPER_VOICE_PATH` (vacío por defecto → responde en texto
aunque la entrada sea voz).

## Fuera de alcance en este repo (explícitamente pedido así)

- `rl_motor_qlearning.py`, `rl_motor_sarsa_lambda.py`, `rl_motor_reinforce.py`
  — el usuario los escribe aparte.
- Firmware del Arduino Mega 2560 (`.ino`).
- `motor_gui.py` — pendiente, falta decidir framework (Tkinter/PyQt/web).

## Estado probado en la laptop real (Ubuntu 22.04)

- `test_smoke.py` corre OK (versión sin voz).
- `mongo_motor_subscriber.py` insertando en Atlas en vivo contra el bus en
  modo `--sim` (docs de `motor_telemetria` confirmados: `timestamp, volts,
  pwm, enabled, setpoint, rpm, seq, t_ms, owner, adc`).
- Cambio de modo por Telegram confirmado insertando en `control_eventos`
  (`tipo: "comando_modo", modo, origen: "telegram:<chat_id>"`).
- Bot de Telegram creado en BotFather como **@QED Motor Control**
  (username terminó en algo distinto de `motor_bot`, que estaba tomado;
  confirmar el username real al configurar `.env`).
- `pip install -r requirements.txt` con `faster-whisper` y `piper-tts` ya
  corrido con éxito en el `venv` real (Python 3.10); quedó un warning
  inofensivo de `pymongo[srv]` (el extra `srv` ya viene incluido en
  pymongo≥4.9, no hace falta el corchete) y conflictos de dependencias
  ajenos al proyecto (`generate-parameter-library-py`, de otro entorno
  ROS/robot en la misma laptop).
- **Aún sin correr en la laptop real**: la versión de `bot.py` con voz
  (`handle_voice`, `procesar_mensaje_texto`) y el `test_smoke.py` /
  `README.md` regenerados con la sección de voz — recién generados en esta
  conversación, pendientes de copiar y probar.

## Pendiente inmediato

1. **Copiar a la laptop y correr `test_smoke.py`** con el `bot.py` nuevo
   (con voz) para confirmar que sigue pasando.
2. **Probar la voz de punta a punta**: correr `bot.py` real y mandarle una
   nota de voz por Telegram (el smoke test no cubre Whisper/Piper reales).
3. **Borrar los datos de prueba insertados** en `motor_telemetria` /
   `control_eventos` durante las pruebas en vivo contra Atlas (no hay
   todavía un script tipo `--clear` como en `seed_historical_data.py` del
   proyecto huerto; se puede hacer con `mongosh` directo mientras tanto).

## Principios de Karpathy aplicados en todo el repo

1. **Think Before Coding** — no adivinar; ver ejemplo del modo ambiguo en
   `detectar_comando_modo`, y el pedir el código fuente real de
   `huerto-bot/bot.py` antes de portar la voz en vez de reinventarla.
2. **Simplicity First** — sin abstracciones especulativas (ej. `bus_client.py`
   sin reconexión propia, sin async donde no hace falta; modelos de voz
   cargan perezosos para no pagar su costo si nadie usa audio).
3. **Surgical Changes** — cada script toca solo lo suyo (el suscriptor no
   controla el motor; el bot no persiste telemetría). Al agregar voz, el
   único cambio estructural fue extraer `procesar_mensaje_texto()` para
   compartir la lógica entre texto y voz, sin duplicarla.
4. **Goal-Driven Execution** — los módulos documentan el criterio de éxito
   en el docstring, no una receta paso a paso.
