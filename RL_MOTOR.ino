/* ===========================================================================
 * motor_rl.ino  -  Firmware Arduino MEGA 2560 para control de velocidad
 * ===========================================================================
 *
 * Papel del microcontrolador: ES SOLO EL PLANTA-INTERFAZ. No decide nada.
 *   - Aplica la señal de CONTROL  : PWM 0..255 sobre PIN_PWM.
 *   - Publica la señal MEDIDA     : tacometro analogico 0-5V en PIN_TACHO.
 *   - Habla por USB-serie con motor_bus.py (que reparte a los algoritmos).
 *
 * La inteligencia (Q-learning / SARSA(lambda) / REINFORCE) vive en el PC.
 * El Arduino solo garantiza: cadencia fija de muestreo, filtrado, rampa
 * opcional y PARADA SEGURA si el PC deja de hablar (watchdog).
 *
 * ---------------------------------------------------------------------------
 * CABLEADO
 * ---------------------------------------------------------------------------
 *   PIN 9  (PWM, OC2B) --> entrada PWM del driver (BTS7960 / L298N / IBT-2 /
 *                          ESC / opto + MOSFET). NUNCA el motor directo.
 *   PIN 8  (DIR)       --> direccion del driver (opcional, ver USE_DIR).
 *   A0     (TACHO)     --> salida 0-5V del tacometro.
 *                          Recomendado: 10k en serie + 100nF a GND (filtro RC
 *                          anti-alias ~1.6 Hz) y diodos de clamp a 5V/GND.
 *                          Si el tacometro pudiera pasar de 5V -> divisor.
 *   GND    <-> GND del driver Y del tacometro (masa comun OBLIGATORIA).
 *
 * ---------------------------------------------------------------------------
 * PROTOCOLO SERIE (115200 8N1, lineas terminadas en '\n')
 * ---------------------------------------------------------------------------
 *   PC -> Arduino:
 *     P<0..255>   fija el PWM (senal de control)          ej: P128
 *     E<0|1>      habilita/deshabilita la salida          ej: E1
 *     D<0|1>      direccion (si USE_DIR)                  ej: D0
 *     T<ms>       periodo de telemetria (10..1000)        ej: T50
 *     K<float>    RPM a fondo de escala (5.000 V)         ej: K3000
 *     F<float>    alpha del filtro EMA (0.01..1.0)        ej: F0.30
 *     S<0..255>   rampa: salto maximo de PWM por muestra  ej: S255 (sin rampa)
 *     Z           parada inmediata (PWM=0, E=0)
 *     ?           pide una linea de informacion
 *
 *   Arduino -> PC:
 *     D,<ms>,<pwm>,<adc>,<mV>,<rpm>,<en>    telemetria periodica (cada T ms)
 *     #ok <eco>          / #err <motivo>    / #info k=.. f=.. t=.. s=..
 *     #hello motor-rl 1.0
 *
 * El campo <ms> es millis() EN EL INSTANTE DEL MUESTREO: es la marca de tiempo
 * buena para el RL (equivale al 'sample_index' del simulador anterior).
 * =========================================================================== */

// ------------------------- CONFIGURACION (edita aqui) ----------------------
const uint8_t PIN_PWM   = 9;      // OC2B (Timer2) en el Mega
const uint8_t PIN_DIR   = 8;
const uint8_t PIN_TACHO = A0;

const bool    USE_DIR        = false;  // true si tu driver necesita pin DIR
const bool    FAST_PWM_31K   = true;   // true -> ~31.4 kHz (silencioso, mejor
                                       //   para motores DC). Afecta pines 9/10.
const uint16_t TS_MS_DEFAULT = 50;     // periodo de muestreo/telemetria (20 Hz)
const float   RPM_FS_DEFAULT = 3000.0; // RPM cuando el tacometro da 5.000 V
                                       //   <<< CALIBRA ESTO CON TU MOTOR >>>
const float   EMA_ALPHA_DEF  = 0.30;   // filtro de la medida (1.0 = sin filtro)
const uint8_t SLEW_DEFAULT   = 255;    // 255 = sin rampa; p.ej. 20 = suave
const uint8_t OVERSAMPLE     = 8;      // lecturas ADC promediadas por muestra
const uint16_t WATCHDOG_MS   = 1500;   // sin ordenes del PC -> PWM 0
const float   VREF           = 5.000;  // tension de referencia real del ADC
// ---------------------------------------------------------------------------

uint16_t ts_ms      = TS_MS_DEFAULT;
float    rpm_fs     = RPM_FS_DEFAULT;
float    ema_alpha  = EMA_ALPHA_DEF;
uint8_t  slew_max   = SLEW_DEFAULT;

uint8_t  pwm_target = 0;      // lo que pide el PC
uint8_t  pwm_out    = 0;      // lo que se aplica de verdad (tras rampa/enable)
bool     enabled    = false;
bool     dir_fwd    = true;

float    rpm_filt   = 0.0;
uint32_t t_last_tx  = 0;
uint32_t t_last_cmd = 0;
bool     wd_tripped = false;

char     buf[24];
uint8_t  buf_n = 0;

// --------------------------------------------------------------------------
void aplicarPWM() {
  uint8_t deseado = enabled ? pwm_target : 0;
  // Rampa (slew-rate limit): protege el driver y evita picos de corriente.
  int16_t delta = (int16_t)deseado - (int16_t)pwm_out;
  if (delta >  (int16_t)slew_max) delta =  (int16_t)slew_max;
  if (delta < -(int16_t)slew_max) delta = -(int16_t)slew_max;
  pwm_out = (uint8_t)((int16_t)pwm_out + delta);
  analogWrite(PIN_PWM, pwm_out);
  if (USE_DIR) digitalWrite(PIN_DIR, dir_fwd ? HIGH : LOW);
}

uint16_t leerTacho() {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < OVERSAMPLE; i++) acc += analogRead(PIN_TACHO);
  return (uint16_t)(acc / OVERSAMPLE);      // 0..1023 promediado
}

void info() {
  Serial.print(F("#info k=")); Serial.print(rpm_fs, 1);
  Serial.print(F(" f="));      Serial.print(ema_alpha, 3);
  Serial.print(F(" t="));      Serial.print(ts_ms);
  Serial.print(F(" s="));      Serial.print(slew_max);
  Serial.print(F(" en="));     Serial.print(enabled ? 1 : 0);
  Serial.print(F(" pwm="));    Serial.println(pwm_out);
}

void procesar(char *s) {
  char c = s[0];
  float v = atof(s + 1);
  switch (c) {
    case 'P': {
      int x = (int)v;
      if (x < 0 || x > 255) { Serial.println(F("#err pwm fuera de 0..255")); return; }
      pwm_target = (uint8_t)x;
      break;
    }
    case 'E': enabled = (v != 0); if (!enabled) pwm_target = 0; break;
    case 'D': dir_fwd = (v == 0); break;
    case 'T': if (v >= 10 && v <= 1000) ts_ms = (uint16_t)v;
              else { Serial.println(F("#err ts fuera de 10..1000")); return; } break;
    case 'K': if (v > 1) rpm_fs = v; else { Serial.println(F("#err k")); return; } break;
    case 'F': if (v > 0.005 && v <= 1.0) ema_alpha = v;
              else { Serial.println(F("#err alpha")); return; } break;
    case 'S': if (v >= 1 && v <= 255) slew_max = (uint8_t)v;
              else { Serial.println(F("#err slew")); return; } break;
    case 'Z': enabled = false; pwm_target = 0; pwm_out = 0; analogWrite(PIN_PWM, 0); break;
    case '?': info(); return;
    default:  Serial.println(F("#err cmd desconocido")); return;
  }
  Serial.print(F("#ok ")); Serial.println(s);
}

void leerSerie() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf_n > 0) {
        buf[buf_n] = '\0';
        t_last_cmd = millis();
        if (wd_tripped) { wd_tripped = false; Serial.println(F("#ok watchdog rearmado")); }
        procesar(buf);
        buf_n = 0;
      }
    } else if (buf_n < sizeof(buf) - 1) {
      buf[buf_n++] = c;
    } else {
      buf_n = 0;                       // linea demasiado larga: se descarta
      Serial.println(F("#err linea larga"));
    }
  }
}

// --------------------------------------------------------------------------
void setup() {
  pinMode(PIN_PWM, OUTPUT);
  if (USE_DIR) pinMode(PIN_DIR, OUTPUT);
  analogWrite(PIN_PWM, 0);

  if (FAST_PWM_31K) {
    // Timer2 (pines 9 y 10 del Mega) con prescaler 1 -> 31372.55 Hz.
    // Fuera del rango audible y con rizado de corriente mucho menor.
    TCCR2B = (TCCR2B & 0b11111000) | 0x01;
  }

  Serial.begin(115200);
  delay(50);
  analogRead(PIN_TACHO);                       // descarta la primera lectura
  rpm_filt = analogRead(PIN_TACHO) * (VREF / 1023.0) * (rpm_fs / VREF);
  t_last_cmd = millis();
  Serial.println(F("#hello motor-rl 1.0"));
  info();
}

void loop() {
  leerSerie();

  uint32_t now = millis();

  // Watchdog: si el PC calla (proceso caido, USB desconectado) -> parar.
  if (now - t_last_cmd > WATCHDOG_MS) {
    if (!wd_tripped) { wd_tripped = true; Serial.println(F("#err watchdog: PWM=0")); }
    enabled = false;
    pwm_target = 0;
  }

  if (now - t_last_tx >= ts_ms) {
    t_last_tx += ts_ms;                        // cadencia sin deriva acumulada
    if (now - t_last_tx > 5UL * ts_ms) t_last_tx = now;   // resincroniza si se atasco

    uint16_t adc = leerTacho();
    float volts  = adc * (VREF / 1023.0);
    float rpm_raw = volts * (  / VREF);
    rpm_filt += ema_alpha * (rpm_raw - rpm_filt);

    aplicarPWM();                              // control y medida en el MISMO tic

    // D,<ms>,<pwm>,<adc>,<mV>,<rpm>,<en>
    Serial.print(F("D,"));   Serial.print(now);
    Serial.print(',');       Serial.print(pwm_out);
    Serial.print(',');       Serial.print(adc);
    Serial.print(',');       Serial.print((int)(volts * 1000.0));
    Serial.print(',');       Serial.print(rpm_filt, 1);
    Serial.print(',');       Serial.println(enabled ? 1 : 0);
  }
}
