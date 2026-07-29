# motor_agent

Agente conversacional para el sistema de control del motor DC (Quanser QET
DC Motor Control Trainer), desarrollado como trabajo de grado. Permite
operar y consultar el motor por Telegram (texto o nota de voz), con tres
frentes:

1. **Control clásico y por refuerzo**: un controlador PID y tres
   algoritmos de RL (Q-learning, SARSA(λ), REINFORCE) compiten por el
   mando del motor a través de un bus TCP/JSON propio (`motor_bus.py`).
2. **Telemetría persistente**: cada muestra del motor se escribe en
   MongoDB (`mongo_motor_subscriber.py`), separando la operación en
   tiempo real (el bus) de su registro histórico (la base de datos). La
   base corre **local, dentro de Docker** (`mongo:7`); Atlas sigue
   siendo una opción, se elige con `MONGO_URI` en el `.env`.
3. **Interfaz conversacional**: un bot de Telegram (`bot.py`) que
   responde preguntas sobre el estado del motor usando un LLM local
   (Qwen3.5-4B vía LM Studio) y ejecuta comandos deterministas (cambio
   de modo, consigna, entrenamiento) sin pasar esos comandos por el LLM.

Para operar el stack día a día ver `GUIA_OPERACION.md`. Para subir
cambios a GitHub ver `SUBIR_A_GITHUB.md`.

## Despliegues activos

El proyecto corre en dos máquinas **independientes a propósito** — no
comparten datos ni están pensadas para operar el mismo motor a la vez.
Cada una tiene ahora su propia base local en Docker (volumen
`mongo_data`); los clusters de Atlas de cada una quedan como respaldo
histórico, no como destino activo:

| | Raspberry Pi 5 | Escritorio (ASUS TUF A15) |
|---|---|---|
| Rol | Despliegue con el Quanser real | Desarrollo / pruebas |
| Arquitectura | aarch64 | x86_64 |
| Base de datos | Local en Docker (`mongo_data`) | Local en Docker (`mongo_data`) |
| Cluster Atlas (respaldo) | `cluster0.boten8d.mongodb.net` | `motorcluster.qulsskv.mongodb.net` |
| `WHISPER_MODEL_SIZE` | `small` (CPU sin margen) | `medium` (CPU más holgada) |

Docker hace que el mismo `Dockerfile`/`docker-compose.yml` sirva para
ambas sin cambios — cada máquina personaliza lo que le corresponde vía
su propio `.env` (proyecto) y `docker/.env` (solo build args, ver más
abajo), nunca editando los archivos de Docker en sí.

## Por qué un LLM local y no una API en la nube

Decisión deliberada, no por defecto:
- **Costo**: el bot puede recibir preguntas constantes durante pruebas
  con usuarios sin generar factura por token.
- **Privacidad/autonomía**: el sistema de control (hardware real, datos
  del motor) no depende de que un tercero externo esté disponible para
  que el operador pueda preguntarle algo al bot.
- **Offline por diseño**: con la base de datos ya local (ver
  "Despliegues activos"), el sistema completo — lazo de control, LLM,
  voz y persistencia — corre sin internet. Lo único que sigue
  necesitando red es el propio Telegram, por ser un servicio externo por
  naturaleza.

El costo de esta decisión es real y está documentado: en hardware sin
GPU dedicada, la latencia del LLM (1-3 minutos en preguntas complejas en
la Pi) es un piso que no desaparece sin cambiar de hardware o de modelo.

## Por qué Docker

El objetivo explícito de esta etapa fue simplificar acceso, descarga y
despliegue del proyecto. Docker resuelve tres problemas concretos que
tenía el setup manual:
- **Reproducibilidad**: instalar el stack completo (`ffmpeg`,
  `python3-tk`, las dependencias de audio/ML) deja de depender de
  recordar cada paso manual.
- **Aislamiento sin perder simplicidad**: los 5 servicios (bot, bus,
  subscriber, gui, mongo) quedan separados y reiniciables de forma
  independiente, con `network_mode: host` -- deliberadamente la opción
  mas simple posible para un solo equipo, no un clúster.
- **Portabilidad real, ya probada**: el mismo `Dockerfile` corrió sin
  ningún cambio tanto en la Pi (arm64) como en el escritorio (x86_64) --
  Docker construye la imagen para la arquitectura de cada host
  automáticamente.

LM Studio queda **fuera** de Docker a propósito: es una app nativa, y
los contenedores le hablan por red exactamente igual que si corriera en
el mismo host sin Docker de por medio (gracias a `network_mode: host`).

## Estructura del repo

```
motor_agent/
  bot.py, motor_bus.py, ...          código
  .env                                 secretos + config de esta máquina (nunca al repo)
  .gitignore
  docker/
    Dockerfile
    docker-compose.yml                base: mongo + bus(--sim) + subscriber + gui + bot
    docker-compose.hardware.yml       override, Arduino real
    .env                               SOLO build args (WHISPER_MODEL_SIZE/COMPUTE_TYPE), nunca al repo
  GUIA_OPERACION.md
  SUBIR_A_GITHUB.md
  README.md                           este archivo
```

## Whisper: horneado en la imagen, no descargado en caliente

El modelo de voz a texto (faster-whisper) se descarga **durante el
build**, no la primera vez que llega una nota de voz -- así el
contenedor arranca listo, sin depender de la red en producción. La
talla es un `ARG` de build (`WHISPER_MODEL_SIZE`, default `small`),
leído del `docker/.env` de cada máquina (no del `.env` del proyecto):

```bash
# docker/.env, específico de esta máquina, no se sube al repo
WHISPER_MODEL_SIZE=medium
WHISPER_COMPUTE_TYPE=int8
```

**Debe coincidir con `WHISPER_MODEL_SIZE` en el `.env` del proyecto** --
si no coinciden, `bot.py` en runtime pide un modelo distinto al
horneado y lo vuelve a descargar en caliente (funciona igual, solo
pierde la ventaja de tenerlo ya listo).

## Principios de Karpathy aplicados (con ejemplos reales de este proyecto)

**1. Think Before Coding** -- no asumir, aclarar confusión, presentar
tradeoffs.
- Antes de implementar el cambio de setpoint/entrenamiento por Telegram,
  se preguntó explícitamente si el setpoint debía aplicarse en caliente
  o requería relanzar el controlador, en vez de asumir un diseño.
- Cuando el bot dejó de responder en el escritorio, no se asumió que
  era el mismo bug de dos instancias compitiendo por el token (ya
  descartado antes) -- se verificó con `getUpdates` crudo, sin pasar por
  el bot, hasta encontrar la causa real (Whisper `medium` colgado en la
  carga, no un problema de Telegram).

**2. Simplicity First** -- código mínimo, sin abstracciones
especulativas.
- El bus de comunicación es TCP/JSON hecho a mano, no un broker de
  mensajería de propósito general.
- La detección de comandos de modo/setpoint es un match de palabras
  clave determinista, no un parser de lenguaje natural ni una llamada a
  LLM -- decisión de seguridad tanto como de simplicidad.
- El despliegue con Docker usa `network_mode: host` en vez de una red
  bridge con DNS de servicios.

**3. Surgical Changes** -- solo tocar lo que la tarea requiere.
- Cada fix (el doble mensaje "system" que rompía la plantilla de
  Qwen3.5, el orden de detección de setpoint/modo, el venv movido de
  `/app/.venv` a `/opt/venv`, `HF_HUB_DISABLE_XET`) se aplicó como un
  diff mínimo y verificado, nunca como una reescritura del archivo.

**4. Goal-Driven Execution** -- criterios de éxito, no instrucciones
paso a paso.
- La comparación a ciegas entre Qwen3.5 (varias tallas) y Gemma 4 con
  usuarios reales usó como criterio "respuesta limpia, en español, sin
  fugas de razonamiento, manejable en CPU" -- eso definió la elección,
  no una instrucción previa de qué modelo usar.

## Decisiones y depuración -- cronología técnica real

- **Doble mensaje "system"**: la plantilla de chat de Qwen3.5 exige que
  el único mensaje `system` sea el primero de la lista; un segundo
  mensaje `system` (estado del motor cerca de la pregunta) rompía el
  parser de LM Studio con un 400. Fix: ese mensaje pasa como `user`.
- **Thinking mode por defecto**: Qwen3.5 genera un bloque de
  razonamiento oculto antes de cada respuesta salvo que se desactive
  explícitamente (`chat_template_kwargs: {"enable_thinking": false}`).
- **Prompt processing casi tan lento como la generación** en la Pi
  (CPU-only): ~4.5 t/s de prefill vs ~2.2 t/s de generación, muy lejos
  de la ventaja habitual con GPU. El historial de conversación tiene un
  costo real y creciente, no solo estético.
- **TLS `internal_error` contra MongoDB Atlas**: diagnosticado con
  `openssl s_client` hasta confirmar que la causa era una IP dinámica
  desactualizada en el Network Access de Atlas.
- **Bind mount tapando lo horneado en `/app`**: tanto el venv
  (`ModuleNotFoundError: pymongo`/`dotenv`) como, potencialmente, el
  caché de Whisper -- `docker-compose.yml` monta el código del host
  sobre `/app`, así que cualquier cosa instalada ahí durante el build
  desaparece al arrancar el contenedor. Fix: venv en `/opt/venv`, caché
  de HF en `/opt/hf_cache` -- ambos fuera de la ruta montada.
- **`hf_xet` colgado en 0 bytes**: bug conocido del backend acelerado de
  descargas de Hugging Face en ciertas redes -- la metadata baja bien
  (no pasa por Xet), el peso real del modelo se cuelga sin error.
  `HF_HUB_DISABLE_XET=1` fuerza el fallback HTTP clásico.
- **`docker compose restart` no relee `.env`**: solo un `up -d` recrea
  el contenedor con variables de entorno nuevas -- un cambio de
  `WHISPER_MODEL_SIZE` en `.env` seguido de `restart` no tiene ningún
  efecto, hay que usar `up -d`.

## Limitaciones conocidas

- El LLM local en la Pi (CPU-only) tiene un piso de latencia real
  (~1-3 minutos en preguntas complejas) -- decisión consciente de
  costo/latencia a cambio de privacidad y autonomía offline.
- `motor_gui.py` dentro de Docker depende de un servidor X activo en el
  host (no funciona headless, con o sin Docker de por medio).
- La base local de Docker corre **sin autenticación**, atada a
  `127.0.0.1` -- suficiente para un equipo de un solo usuario, pero
  habría que agregar credenciales antes de exponerla a la red.
- Las bases de las dos máquinas (Pi / escritorio) no se sincronizan
  entre sí -- es una decisión, no un descuido (ver "Despliegues
  activos"), pero implica que un análisis histórico completo requiere
  consultar ambas por separado.
- Si se vuelve a usar Atlas: la IP pública de cada máquina es dinámica y
  su Network Access requiere reautorizarse a mano cada vez que cambia
  (fue la causa raíz de varias caídas de conexión antes de migrar a
  local).
