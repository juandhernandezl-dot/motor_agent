// Configuración inicial de la base "motor_qet" en MongoDB Atlas.
// Ejecutar una sola vez, conectado al cluster de Atlas:
//   mongosh "mongodb+srv://usuario:password@cluster.mongodb.net/motor_qet" mongo_setup_motor.js
//
// Crea dos colecciones con responsabilidades separadas, mismo patrón que
// se usó en el proyecto de huerto (telemetria vs eventos):
//   - motor_telemetria: Time Series Collection nativa. Cada documento es
//     una muestra de "telemetry" tal cual la publica motor_bus.py, YA con
//     setpoint y owner (modo vigente) fusionados por el bus:
//       t_ms, seq, pwm, adc, volts, rpm, enabled, setpoint, owner
//     (ver mongo_motor_subscriber.py: procesar_evento()). Ya no se guardan
//     "telemetria" y "control" como documentos separados como en la
//     primera versión (esa asumía nodo_comunicacion.py + WebSocket, sin
//     fusión); el bus resuelve esa correspondencia temporal por su cuenta.
//   - control_eventos: colección normal. Cambios discretos de modo de
//     control (topico "mode" del bus): manual, qlearning, sarsa_lambda,
//     reinforce o free -- quién lo pidió y cuándo.

db.createCollection("motor_telemetria", {
  timeseries: {
    timeField: "timestamp",
    granularity: "seconds"   // el motor reporta a decenas de Hz (Ts=50ms por defecto)
  }
});
db.motor_telemetria.createIndex({ timestamp: 1 });

db.createCollection("control_eventos");
db.control_eventos.createIndex({ timestamp: 1 });
db.control_eventos.createIndex({ modo: 1 });

print("Colecciones 'motor_telemetria' (time series) y 'control_eventos' creadas en 'motor_qet'.");
print("Colecciones existentes:");
db.getCollectionNames().forEach(name => print(" - " + name));
