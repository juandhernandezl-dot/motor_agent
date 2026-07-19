# CLAUDE.md — Proyecto motor QET (trabajo de grado)

Contexto para Claude Code sobre `~/Documents/motor/`. Lee esto antes de tocar
código en este repo.

## Qué es esto

Control (manual, PID clásico, y RL: Q-learning, SARSA(λ), REINFORCE) de un
Quanser QET DC Motor Control Trainer, con Arduino Mega 2560, un bus de
comunicación en Python, una GUI de Tkinter, persistencia en MongoDB Atlas, y
un bot de Telegram (texto y voz) como agente de consulta/control. Todo corre
en una laptop Ubuntu 22.04, en VS Code, con `venv`.

Proyecto hermano (otro repo, mismo autor): `~/Documents/huerto-ml` y
`~/Documents/huerto-bot` (LSTM de riego + bot de Telegram). Este proyecto
copió su patrón de arquitectura (LLM local + Mongo + historial en memoria,
voz Whisper/Piper) pero es un dominio distinto; no compartir código
directamente sin revisar que aplique.

## Arquitectura real

```
Arduino Mega 2560 ──(serie)──► motor_bus.py ──(TCP/JSON, 127.0.0.1:8770)──►
    ├── motor_gui.py   -- UNICO proceso que lanza/detiene subprocesos
    │     │  (Popen + Ctrl+C; una consola por controlador, descubiertos
    │     │   por controller_registry.py sin ejecutarlos)
    │     └─ lanza: ctl_pid.py / rl_motor_qlearning.py / sarsa_lambda /
    │               reinforce.py  (cada uno toma el mando con su propia
    │               clave: "pid"/"qlearning"/"sarsa_lambda"/"reinforce")
    ├── mongo_motor_subscriber.py  (persiste todo en Atlas)
    └── bot.py  (lee Atlas; pide cambios de controlador -- NO lanza
                 procesos el mismo, se los pide a motor_gui.py por el bus)
```

`motor_bus.py` es el **único** proceso que toca el puerto serie. Es un
broker publicador/suscriptor: los clientes hablan TCP con una línea JSON
por mensaje (`{"cmd":...}` → respuesta; eventos asíncronos `{"type":"event",
"topic":...,"data":...}`).

Hay **dos clientes de bus distintos, a propósito** (no es duplicación
accidental, ver conversación): `bus_client.py` es un cliente simple y
bloqueante, de conexión corta, para `bot.py` y `mongo_motor_subscriber.py`.
`motor_client.py` es un cliente con hilo lector y callbacks, para
`motor_gui.py` y los controladores (`ctl_pid.py`, `rl_motor_*.py`), que
necesitan recibir eventos sin bloquear Tkinter ni el lazo de control.

**Ya NO existe** `nodo_comunicacion.py` (versión anterior por WebSocket).
Si aparece código o docs que lo mencionen, están obsoletos.

### Tópicos del bus

| Tópico | Quién publica | Campos |
|---|---|---|
| `telemetry` | el bus, a cada muestra | `t_ms,seq,pwm,adc,volts,rpm,enabled,setpoint,owner` (setpoint y owner ya fusionados por el bus) |
| `control` | el dueño actual (`owner`) | `pwm` (0-255), `src` |
| `enable` | el dueño actual | `on` (bool), `src` |
| `setpoint` | GUI o algoritmo | `rpm`, `src` |
| `config` | GUI | `ts_ms` (periodo de muestreo en caliente) |
| `link` | el bus | estado del enlace serie con el Arduino |
| `mode` | quien tenga el mando o `motor_gui.py` | `owner`: uno de los modos válidos, `src` |
| `agent/table`, `agent/status`, `log` | los algoritmos / el bus | ver docstring de `motor_bus.py` |
| `controller/launch` | `bot.py` (o cualquier cliente) | `{"clave": <modo>, "src": <quien pide>}` — **pedido**, no confirmación |
| `controller/stop` | idem | `{"clave": <modo o None=todos>, "src": ...}` |
| `controller/ack` | **solo `motor_gui.py`** | `{"clave": <modo>, "ok": bool, "error"?: str}` — la confirmación real |

**Modos válidos (`owner`)**: `manual`, `pid`, `qlearning`, `sarsa_lambda`,
`reinforce`, `free`. No son genéricos tipo "rl" — hay tres algoritmos de RL
distintos y un PID, y el modo debe decir cuál.

Arbitraje: el bus solo acepta `control` cuyo `src == owner` (o si
`owner == "free"`). Interlock de sobrevelocidad (`--rpm-max`) por encima de
cualquier algoritmo. Keepalive cada 500ms; watchdog del Arduino corta el
motor a los 1.5s sin señal. `--sim` simula el motor sin hardware.

### Cambiar de controlador: `controller/launch` / `controller/ack` (IMPORTANTE)

**Publicar en `mode` NO arranca ni detiene ningún proceso** — solo cambia
quién *tiene permiso* de mandar. Si nadie con ese nombre está corriendo,
el motor queda a 0 pwm para siempre, sin avisar. Esto causó un bug real
(ver conversación): el bot cambiaba el `mode` a `sarsa_lambda` con éxito,
pero como ese proceso no estaba corriendo, el motor se quedaba quieto y el
controlador anterior (que había perdido el mando) seguía imprimiendo en su
consola sin controlar nada — comportamiento esperado del diseño (los
controladores no se auto-matan al perder el mando, solo `Ctrl+C` los para;
ver "Qué NO cambia" más abajo), pero confuso si nadie sabe que hace falta
lanzar el proceso a mano.

La solución: **`motor_gui.py` es el único que lanza/mata subprocesos**
(tiene `Proceso`, `controller_registry.descubrir/construir_cmd`, y una
consola por controlador). Cualquier cliente del bus — típicamente `bot.py`
— puede **pedirle** un cambio publicando en `controller/launch`, y
`motor_gui.py` hace lo mismo que si alguien hiciera clic en "Lanzar" en la
GUI: detiene con `Ctrl+C` cualquier controlador vivo, y lanza el pedido (o,
si es `manual`/`free`, que no tienen script propio, solo cambia `mode`).
Al terminar, publica la confirmación real en `controller/ack`.

`bot.py` (`_cambiar_controlador_sync` en `bot.py`) publica en
`controller/launch` y **espera** el evento `controller/ack` (con timeout
`CONTROLLER_ACK_TIMEOUT_S`, default 12s) antes de responderle al usuario —
nunca asume éxito solo porque el bus aceptó el publish. Si no llega
confirmación a tiempo, le avisa al usuario que revise si `motor_gui.py`
está abierta.

**Para lanzamientos remotos** (sin nadie sentado en la consola para
contestar un menú interactivo), `motor_gui.py._on_remote_launch` fuerza
`--controlar --directo` en los parámetros "Modo"/"Ritmo" de los tres
agentes de RL (en vez de heredar el default `""`, que abre un menú
interactivo esperando input por stdin — nadie va a contestarlo si el
pedido vino de Telegram). Ver `_REMOTO_OVERRIDES` en `motor_gui.py`.

**Qué NO cambia** (comportamiento previo, sigue igual, a propósito): un
controlador que pierde el mando (porque otro lo tomó) NO se detiene solo
— el PID queda en modo *bumpless* reintentando, y los agentes de RL siguen
observando/aprendiendo de la telemetría real aunque no controlen. Solo
`Ctrl+C` (mandado por `motor_gui.py`, ya sea por clic o por
`controller/launch`/`controller/stop` remoto) los detiene de verdad.

## Esquema de MongoDB Atlas

DB `motor_qet`:
- `motor_telemetria` (time series, `timeField: timestamp`, granularidad
  `seconds`): un doc por muestra de `telemetry`, campos tal cual arriba.
- `control_eventos`: un doc por cambio de modo real (no por el valor
  "retenido" que el bus manda al suscribirse). Campos: `timestamp, tipo:
  "comando_modo", modo, origen` (ej. `"telegram:<chat_id>"`).

`mongo_setup_motor.js` crea ambas colecciones (correr una sola vez contra
Atlas). `mongo_motor_subscriber.py` es el único escritor. Solo persiste
`telemetry` y `mode`; `controller/launch`, `controller/ack`, `config`,
`link`, etc. no se guardan por ahora.

## Archivos del repo y su rol

- `motor_bus.py` — el bus (dado, no generado en esta conversación).
- `motor_client.py` — cliente del bus con hilo lector + callbacks, para
  `motor_gui.py` y los controladores (`ctl_pid.py`, `rl_motor_*.py`).
  `BusClient(name, host, port).connect()`, `subscribe(topico, callback)`,
  `next_sample()` (sincroniza el lazo de control con el muestreo real).
- `bus_client.py` — cliente TCP mínimo y bloqueante (solo stdlib), para
  conexiones cortas: `mongo_motor_subscriber.py` (conexión larga,
  `events()`) y `bot.py` (conexión corta, `publish()`). Tiene
  `subscribe_and_ack()`/`next_event()` para esperar una confirmación
  puntual sin abrir un segundo cliente (ver `_cambiar_controlador_sync` en
  `bot.py`) — **cuidado**: `subscribe()` normal no consume su propia
  respuesta (por diseño, para no interferir con `events()`); si necesitas
  esperar un "ok" antes de seguir, usa `subscribe_and_ack()`, no
  `subscribe()` a secas, o el siguiente `publish()` leerá la línea
  equivocada.
- `controller_registry.py` — descubre controladores leyendo el dict
  `CONTROLLER` de cada `.py` con `ast` (sin ejecutar código), y arma el
  comando (`construir_cmd`) a partir de los parámetros.
- `motor_gui.py` — la GUI (Tkinter): consigna, Ts en caliente,
  habilitar/parar, PWM manual, gráficas, lanzador de controladores (una
  consola por proceso), mapa de calor de la tabla/política del agente
  activo, y — nuevo — el manejador de `controller/launch`/`controller/stop`
  (ver sección de arriba).
- `ctl_pid.py` — PID clásico con anti-windup, filtro derivativo,
  feedforward de zona muerta, autoajuste IMC (`--auto`, usa
  `plant_motor.json` si existe). Toma el mando como `owner="pid"`.
- `rl_motor_common.py` — utilidades compartidas por los tres agentes de
  RL: discretización del estado, identificación de la planta
  (`--identificar` → `plant_motor.json`), métricas móviles, el *runner*
  (`preparar()`) que maneja pre-entrenamiento acelerado + control en
  tiempo real + persistencia.
- `rl_motor_qlearning.py` / `rl_motor_sarsa_lambda.py` /
  `rl_motor_reinforce.py` — Q-learning (TD off-policy tabular), SARSA(λ)
  (TD on-policy con trazas), REINFORCE (policy gradient Monte Carlo, sin
  discretizar). Cada uno persiste su tabla/política en su propio JSON.
- `mongo_motor_subscriber.py` — cliente del bus → Mongo Atlas. Enruta
  `telemetry` → `motor_telemetria` (por lotes) y `mode` → `control_eventos`
  (uno por uno, ignorando el retenido).
- `bot.py` — bot de Telegram, texto **y voz**. `procesar_mensaje_texto`
  decide entre dos caminos: (1) preguntas de estado → LLM local (LM
  Studio/Qwen) con contexto real leído de Mongo, con el "Estado actual"
  puesto **después** del historial y justo antes de la pregunta (no al
  principio del prompt): con un modelo chico, lo que está más cerca de la
  pregunta pesa más que un system message de varios turnos atrás — si el
  estado quedara arriba, una respuesta vieja del historial (ej. "no tengo
  datos", de un turno sin Mongo) pesa más que el dato fresco y el modelo
  repite la negativa vieja; (2) cambio de controlador → detectado por
  regla determinista (sin LLM), pide el cambio real via
  `controller/launch` y espera `controller/ack` (ver sección de arriba).
  Si pide cambiar de modo sin decir cuál de los 3 algoritmos de RL, el bot
  pregunta en vez de adivinar. `handle_text` y `handle_voice` comparten
  `procesar_mensaje_texto`; `handle_voice` transcribe con faster-whisper y
  sintetiza con Piper + ffmpeg (texto como respaldo si falla la síntesis).
  Carga `.env` con `python-dotenv` al arrancar (antes había que hacer
  `export $(grep -v '^#' .env | xargs)` a mano en cada terminal nueva).
- `mongo_setup_motor.js` — crea las colecciones en Atlas.
- `test_smoke.py` — sin Telegram/Atlas/bus/GUI/LM Studio reales ni modelos
  de Whisper/Piper descargados. El bus falso simula el intercambio
  completo de `controller/launch` → `controller/ack` (como si
  `motor_gui.py` respondiera). No prueba Whisper/Piper de punta a punta ni
  `motor_gui.py` real.
- `requirements.txt` — incluye `python-dotenv`, `faster-whisper`,
  `piper-tts` además de las dependencias base. `motor_gui.py` necesita
  además `sudo apt install python3-tk` (paquete de sistema, no de pip).
- `piper-voices/es_MX-claude-high.onnx(.json)` — voz de Piper.

## Variables de entorno (`.env`) que usa `bot.py`

Obligatorias: `TELEGRAM_BOT_TOKEN`, `MONGO_URI` (Atlas, `mongodb+srv://...`).

Con default razonable si se omiten: `LLM_BASE_URL`, `LLM_MODEL`,
`LLM_API_KEY`, `MAX_HISTORY_TURNS`, `REQUEST_TIMEOUT_S`, `BUS_HOST`,
`BUS_PORT`, `BUS_TIMEOUT_S`, `CONTROLLER_ACK_TIMEOUT_S` (default 12s: más
largo que `BUS_TIMEOUT_S` porque detener+lanzar un proceso tarda más que
un publish suelto), `MONGO_DB`, `MONGO_COLLECTION`,
`TELEMETRY_STALE_SECONDS`.

De voz (mismos nombres que en `huerto-bot`): `WHISPER_MODEL_SIZE` (default
`small`), `WHISPER_COMPUTE_TYPE` (default `int8`), `WHISPER_LANGUAGE`
(default `es`), `PIPER_VOICE_PATH` (vacío por defecto → responde en texto
aunque la entrada sea voz).

## Fuera de alcance en este repo (explícitamente pedido así)

- Firmware del Arduino Mega 2560 (`.ino`).

## Pendiente inmediato

1. **Copiar `motor_gui.py`, `bus_client.py` y `bot.py` actualizados a la
   laptop y probar de punta a punta**: pedirle al bot por Telegram
   "cambia a modo qlearning" con `motor_gui.py` abierta, y confirmar que
   detiene el controlador anterior, aparece la consola de `qlearning`
   corriendo con `--controlar --directo`, y el bot responde solo cuando de
   verdad tiene el control (no antes).
2. **Probar el caso de falla**: pedir un cambio de modo con
   `motor_gui.py` cerrada — el bot debe avisar que nadie confirmó a
   tiempo, no fingir éxito.
3. **Borrar los datos de prueba insertados** en `motor_telemetria` /
   `control_eventos` durante pruebas en vivo contra Atlas (no hay todavía
   un script `--clear`; se puede hacer con `mongosh` directo mientras
   tanto).
4. Considerar sincronizar el combobox de la GUI (`self.cb_ctl`) cuando el
   lanzamiento viene de un pedido remoto — hoy es solo cosmético (el
   proceso se lanza bien, pero el combobox puede no reflejar la selección).

## Principios de Karpathy aplicados en todo el repo

1. **Think Before Coding** — no adivinar; ver ejemplo del modo ambiguo en
   `detectar_comando_modo`, y la decisión explícita de que sea
   `motor_gui.py` (no el bot) quien toque subprocesos, para no duplicar
   `Proceso`/`controller_registry` ni arriesgar dos copias de un mismo
   agente peleando por su qtable/policy JSON.
2. **Simplicity First** — sin abstracciones especulativas (`bus_client.py`
   sigue sin reconexión propia; modelos de voz cargan perezosos).
   `controller/launch`/`controller/ack` son dos tópicos más del mismo bus
   pub/sub que ya existía, no un protocolo nuevo.
3. **Surgical Changes** — cada script toca solo lo suyo. Al agregar el
   control remoto, el único cambio estructural en `motor_gui.py` fue
   generalizar "detener + lanzar" para aceptar una clave pedida por el bus
   además de la selección del combobox; `bot.py` solo cambió *cómo* pide
   el cambio de modo (y ahora espera confirmación real).
4. **Goal-Driven Execution** — criterio de éxito de este cambio: "pedir un
   cambio de modo por Telegram detiene de verdad el controlador anterior y
   dispara el nuevo, visible en su propia consola de la GUI, sin tocar el
   mouse" — no una receta paso a paso de qué botones simular.
