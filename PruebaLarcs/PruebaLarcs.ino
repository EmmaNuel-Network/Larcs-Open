// ==========================================
// PESTAÑA PRINCIPAL: Robot-Larc.ino
// FSM única + modos de prueba incrementales
// Integra la lógica probada de PruebaIntegracionUltrasonidos.ino
// ==========================================
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>

Adafruit_MCP23X17 mcp;

// Tipo compartido con Motores.ino (debe quedar en esta pestaña, al inicio)
struct ControlPD {
  float setpoint = 0.0f;
  long pulsosAnteriores = 0;
  float errorPrev = 0.0f;
  float Kp = 1.2f;
  float Kd = 0.15f;
  int outputPWM = 0;
};

// ==========================================
// 1. ESTADOS DE LA FSM
// ==========================================
enum EstadoFSM {
  ESPERA_INICIO,
  IR_A_CULTIVO,
  ESQUIVAR_PISCINA,
  VERIFICAR_LIBERACION,   // Viene de la prueba funcional
  HOMING_Y_ESCUADRADO,
  RECOLECCION_MULTINIVEL,
  ALINEAR_Y_CAPTURAR,
  REGRESO_BENEFICIADERO,
  DESCARGA_PAYLOAD,
  GIRO_180,               // Solo PRUEBA_DEMO: media vuelta al ver la línea
  FIN_MISION,
  ESTADO_ERROR
};

// ==========================================
// 2. MODOS DE PRUEBA
// ==========================================
// Cada modo define en qué estado arranca y qué estados tiene permitidos.
// Si la FSM intenta entrar a un estado no permitido, el robot se detiene
// (FIN_MISION) y lo reporta por Serial.
enum ModoPrueba {
  PRUEBA_SENSORES,      // Robot quieto: imprime IR y ultrasonidos cada 200 ms
  PRUEBA_ESQUIVA,       // Idéntico a PruebaIntegracionUltrasonidos (sin IR)
  PRUEBA_IR_A_CULTIVO,  // Esquiva + se detiene al ver la línea del cultivo
  PRUEBA_HOMING,        // Arranca en homing (poner el robot sobre la línea)
  PRUEBA_RECOLECCION,   // Arranca en recolección (poner el robot frente al árbol)
  PRUEBA_REGRESO,       // Arranca en regreso + descarga
  PRUEBA_DEMO,          // Demostración: avanza, esquiva, gira 180° en la línea y repite
  PRUEBA_GIRO,          // Calibración del giro 180° por Serial ('g', '+', '-')
  COMPETENCIA           // Ronda completa, arranca con el botón
};

// >>>>>>>>>> CAMBIAR AQUÍ EL MODO Y VOLVER A SUBIR <<<<<<<<<<
const ModoPrueba MODO = PRUEBA_DEMO;

// Subsistemas que se activan según el modo
const bool LEER_IR        = (MODO != PRUEBA_ESQUIVA && MODO != PRUEBA_DEMO &&
                             MODO != PRUEBA_GIRO); // La demo usa solo el módulo track
const bool USAR_SERVO     = (MODO == PRUEBA_REGRESO || MODO == COMPETENCIA);
const bool USAR_ELEVADOR  = (MODO == PRUEBA_RECOLECCION || MODO == COMPETENCIA);

EstadoFSM estadoActual = ESPERA_INICIO;

// ==========================================
// 3. PINES Y ACTUADORES ADICIONALES
// ==========================================
const int pinBotonStart      = 13; // MCP PB5 (pendiente reasignar)
const int pinServoCompuerta  = 25;
Servo servoCompuerta;

const int pinStepElevador    = 14; // MCP PB6 (pendiente: el 28BYJ-48 usa IN1-IN4)
const int pinDirElevador     = 15; // MCP PB7
int nivelActualH             = 1;

// ==========================================
// 4. PARÁMETROS DE NAVEGACIÓN (valores de la prueba funcional)
// ==========================================
const int VELOCIDAD_BASE                    = 120;
const int UMBRAL_PISCINA_CM                 = 20;   // Detecta obstáculo
const int UMBRAL_LIBRE_CM                   = 35;   // Considera frente despejado
const unsigned long TIEMPO_CONFIRMAR_LIBRE_MS = 800;
const unsigned long VENTANA_IGNORAR_LINEA_MS  = 1500; // Evita leer líneas de la zona de salida (calibrar)

// --- Parámetros de PRUEBA_DEMO y PRUEBA_GIRO ---
const int pinTrackLinea              = 8;    // MCP PB0: módulo track con comparador
// Al encender, el robot debe estar sobre BLANCO: lee el módulo y asume que
// ese valor es blanco y el contrario es negro. Así no importa la lógica del módulo.
const bool AUTO_CALIBRAR_TRACK       = true;
int trackValorNegro                  = HIGH; // Se sobrescribe si AUTO_CALIBRAR_TRACK = true
const int LECTURAS_CONFIRMAR_LINEA   = 3;    // Lecturas seguidas en negro para aceptar la línea

const int VELOCIDAD_GIRO_DEMO        = 150;
const long PULSOS_GIRO_180           = 600;  // CALIBRAR con PRUEBA_GIRO y escribir aquí el valor
long pulsosGiro180                   = PULSOS_GIRO_180; // Ajustable por Serial ('+' / '-')
const unsigned long PAUSA_ANTES_GIRO_MS = 300;  // Deja frenar el robot antes de girar
const unsigned long TIMEOUT_GIRO_MS  = 4000; // Seguridad si los encoders no cuentan

// Si el ultrasonido lateral ve partes del propio robot, poner en false:
// solo el frontal iniciará la esquiva.
const bool USAR_LATERAL_EN_DETECCION = true;

const unsigned long TIMEOUT_HOMING  = 7000;
const unsigned long TIMEOUT_CAPTURA = 4000;
const unsigned long WATCHDOG_CAMARA = 800;

// ==========================================
// 5. VARIABLES DE CONTROL
// ==========================================
unsigned long tiempoInicioMision = 0;
unsigned long tiempoInicioEstado = 0;
unsigned long tiempoArmadoLinea  = 0; // Desde aquí corre la ventana de ignorar línea
int contadorLinea                = 0;
int trackCrudo                   = 0; // Última lectura de PB0 (para el monitor)
unsigned long inicioGiro         = 0;
unsigned long ultimoMonitorDemo  = 0;
bool estadoIniciado              = false;
int direccionBarrido             = 1;

int errorCamaraX                  = 0;
int errorCamaraY                  = 0;
bool pelotaCentrada               = false;
unsigned long ultimaLecturaCamara = 0;
char bufferUART[64];
size_t idxBuffer = 0;

// ==========================================
// 6. FUNCIONES EXTERNAS (otras pestañas)
// ==========================================
void setupMotores();
void actualizarControlMotores();
void moverOmni(int Vx, int Vy, int W);
void pararMotores();
void resetEncoders();
long getPulsosM1();
long getPulsosM2();
long getPulsosM3();
long getPulsosM4();

void setupInfrarrojos();
bool estaSobreLineaFrontal();
bool detectoMargenIzquierdo();
bool detectoMargenDerecho();

void setupUltrasonidos();
void actualizarLecturasUltrasonido();
int leerUltrasonidoAlto();
int leerUltrasonidoLateral();
float calcularErrorLinea();
int obtenerCorreccionParedPD(int distMedida);

// ==========================================
// 7. GESTIÓN DE MODOS Y TRANSICIONES
// ==========================================
const char* nombreEstado(EstadoFSM e) {
  switch (e) {
    case ESPERA_INICIO:          return "ESPERA_INICIO";
    case IR_A_CULTIVO:           return "IR_A_CULTIVO";
    case ESQUIVAR_PISCINA:       return "ESQUIVAR_PISCINA";
    case VERIFICAR_LIBERACION:   return "VERIFICAR_LIBERACION";
    case HOMING_Y_ESCUADRADO:    return "HOMING_Y_ESCUADRADO";
    case RECOLECCION_MULTINIVEL: return "RECOLECCION_MULTINIVEL";
    case ALINEAR_Y_CAPTURAR:     return "ALINEAR_Y_CAPTURAR";
    case REGRESO_BENEFICIADERO:  return "REGRESO_BENEFICIADERO";
    case DESCARGA_PAYLOAD:       return "DESCARGA_PAYLOAD";
    case GIRO_180:               return "GIRO_180";
    case FIN_MISION:             return "FIN_MISION";
    case ESTADO_ERROR:           return "ESTADO_ERROR";
  }
  return "?";
}

EstadoFSM estadoInicial() {
  switch (MODO) {
    case PRUEBA_SENSORES:     return FIN_MISION; // No se mueve
    case PRUEBA_ESQUIVA:
    case PRUEBA_IR_A_CULTIVO: return IR_A_CULTIVO;
    case PRUEBA_HOMING:       return HOMING_Y_ESCUADRADO;
    case PRUEBA_RECOLECCION:  return RECOLECCION_MULTINIVEL;
    case PRUEBA_REGRESO:      return REGRESO_BENEFICIADERO;
    case PRUEBA_DEMO:         return IR_A_CULTIVO;
    case PRUEBA_GIRO:         return FIN_MISION; // Quieto hasta recibir 'g'
    case COMPETENCIA:         return ESPERA_INICIO;
  }
  return ESPERA_INICIO;
}

bool estadoPermitido(EstadoFSM e) {
  if (e == FIN_MISION || e == ESTADO_ERROR) return true;

  switch (MODO) {
    case PRUEBA_SENSORES:
      return false;
    case PRUEBA_ESQUIVA:
    case PRUEBA_IR_A_CULTIVO:
      return (e == IR_A_CULTIVO || e == ESQUIVAR_PISCINA || e == VERIFICAR_LIBERACION);
    case PRUEBA_HOMING:
      return (e == HOMING_Y_ESCUADRADO);
    case PRUEBA_RECOLECCION:
      return (e == RECOLECCION_MULTINIVEL || e == ALINEAR_Y_CAPTURAR);
    case PRUEBA_REGRESO:
      return (e == REGRESO_BENEFICIADERO || e == DESCARGA_PAYLOAD);
    case PRUEBA_DEMO:
      return (e == IR_A_CULTIVO || e == ESQUIVAR_PISCINA ||
              e == VERIFICAR_LIBERACION || e == GIRO_180);
    case PRUEBA_GIRO:
      return (e == GIRO_180);
    case COMPETENCIA:
      return true;
  }
  return false;
}

// A dónde ir al detectar la línea: en la demo se da media vuelta,
// en los demás modos se pasa al homing.
EstadoFSM estadoAlDetectarLinea() {
  return (MODO == PRUEBA_DEMO) ? GIRO_180 : HOMING_Y_ESCUADRADO;
}

// parar = true detiene motores al cambiar (como hacía la prueba en los
// cambios AVANZAR->ESQUIVAR y VERIFICAR->AVANZAR); false mantiene el movimiento.
void cambiarEstado(EstadoFSM nuevoEstado, bool parar) {
  if (nuevoEstado == estadoActual) return;

  if (!estadoPermitido(nuevoEstado)) {
    Serial.printf("[PRUEBA] %s no incluido en este modo -> FIN\n", nombreEstado(nuevoEstado));
    nuevoEstado = FIN_MISION;
    parar = true;
  }

  if (parar) pararMotores();

  Serial.printf("[FSM] %s -> %s  (t=%lu ms)\n",
                nombreEstado(estadoActual), nombreEstado(nuevoEstado),
                millis() - tiempoInicioMision);

  estadoActual = nuevoEstado;
  estadoIniciado = false;
  tiempoInicioEstado = millis();
}

// ==========================================
// 8. ACTUADORES SECUNDARIOS
// ==========================================
void moverElevadorANivel(int nivel) {
  if (!USAR_ELEVADOR) return;
  pararMotores();
  if (nivel == nivelActualH) return;

  mcp.digitalWrite(pinDirElevador, (nivel > nivelActualH) ? HIGH : LOW);
  int pasos = abs(nivel - nivelActualH) * 400; // TODO: calcular con el piñón real

  for (int i = 0; i < pasos; i++) {
    mcp.digitalWrite(pinStepElevador, HIGH);
    delayMicroseconds(800);
    mcp.digitalWrite(pinStepElevador, LOW);
    delayMicroseconds(800);
  }
  nivelActualH = nivel;

  while (Serial2.available() > 0) Serial2.read();
  ultimaLecturaCamara = millis();
}

void abrirCompuertaDescarga() {
  if (!USAR_SERVO) return;
  servoCompuerta.write(90);
  delay(2000);
  servoCompuerta.write(0);
  while (Serial2.available() > 0) Serial2.read();
}

void ejecutarMecanismoRecolector() {
  delay(250);
  while (Serial2.available() > 0) Serial2.read();
}

// ==========================================
// 9. UART ESP32-CAM
// ==========================================
void parsearTrama(char* trama) {
  int errX, errY, centrado;
  if (sscanf(trama, "%d,%d,%d", &errX, &errY, &centrado) == 3) {
    errorCamaraX = errX;
    errorCamaraY = errY;
    pelotaCentrada = (centrado == 1);
    ultimaLecturaCamara = millis();
  }
}

void procesarCamaraVision() {
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    if (c == '\n') {
      bufferUART[idxBuffer] = '\0';
      parsearTrama(bufferUART);
      idxBuffer = 0;
    } else if (c != '\r') {
      if (idxBuffer < sizeof(bufferUART) - 1) bufferUART[idxBuffer++] = c;
      else idxBuffer = 0;
    }
  }
}

// ==========================================
// 9b. DIAGNÓSTICO DE SENSORES (solo PRUEBA_SENSORES)
// ==========================================
// Lee el puerto B del MCP completo en una sola transacción I2C.
// Mapeo según Infrarojos.ino:
//   PB0 = frontal centro, PB1 = frontal izq, PB2 = frontal der,
//   PB3 = lateral izq
//   Lateral der: GPIO directo de la ESP32 (PB4 ahora es del paso a paso)
// Se espera: 1 = negro, 0 = blanco.
const unsigned long INTERVALO_DIAGNOSTICO_MS = 200;
unsigned long ultimoDiagnostico = 0;

void imprimirDiagnosticoSensores(int distFrontal, int distLateral) {
  if (millis() - ultimoDiagnostico < INTERVALO_DIAGNOSTICO_MS) return;
  ultimoDiagnostico = millis();

  uint8_t puertoB = mcp.readGPIOB();
  int fCen = (puertoB >> 0) & 1;
  int fIzq = (puertoB >> 1) & 1;
  int fDer = (puertoB >> 2) & 1;
  int lIzq = (puertoB >> 3) & 1;
  int lDer = detectoMargenDerecho() ? 1 : 0; // GPIO de la ESP32

  float posLinea = calcularErrorLinea(); // -1 izq, 0 centro, +1 der, 999 sin línea

  Serial.printf("IR frontal [izq cen der] = %d %d %d  pos=", fIzq, fCen, fDer);
  if (posLinea >= 999.0f) Serial.print(" ---");
  else                    Serial.printf("%+.2f", posLinea);
  Serial.printf("  | lateral izq=%d der=%d", lIzq, lDer);
  Serial.printf("  | US frontal(T18/E23)=%3d lateral(T32/E26)=%3d\n",
                distFrontal, distLateral);
}

// ==========================================
// 9c. COMANDOS SERIAL, MONITOR DE DEMO Y AUTOCALIBRACIÓN DEL TRACK
// ==========================================
// 'g' = girar 180° (solo PRUEBA_GIRO, con el robot quieto)
// '+' / '-' = sumar / restar 100 pulsos al giro
// '>' / '<' = sumar / restar 20 pulsos al giro
void procesarComandosSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    long antes = pulsosGiro180;
    if (c == '+') pulsosGiro180 += 100;
    if (c == '-') pulsosGiro180 = max(100L, pulsosGiro180 - 100);
    if (c == '>') pulsosGiro180 += 20;
    if (c == '<') pulsosGiro180 = max(100L, pulsosGiro180 - 20);
    if (pulsosGiro180 != antes) {
      Serial.printf(">>> pulsosGiro180 = %ld\n", pulsosGiro180);
    }
    if (c == 'g' && MODO == PRUEBA_GIRO && estadoActual == FIN_MISION) {
      cambiarEstado(GIRO_180, true);
    }
  }
}

void imprimirMonitorDemo(int distFrontal, int distLateral) {
  if (millis() - ultimoMonitorDemo < 300) return;
  ultimoMonitorDemo = millis();
  Serial.printf("[DEMO] %-20s | track=%d (%s) | US F=%3d L=%3d\n",
                nombreEstado(estadoActual), trackCrudo,
                (trackCrudo == trackValorNegro) ? "NEGRO" : "blanco",
                distFrontal, distLateral);
}

void autocalibrarTrack() {
  int altos = 0;
  for (int i = 0; i < 10; i++) {
    if (mcp.digitalRead(pinTrackLinea) == HIGH) altos++;
    delay(5);
  }
  int valorBlanco = (altos >= 5) ? HIGH : LOW;
  trackValorNegro = (valorBlanco == HIGH) ? LOW : HIGH;
  Serial.printf("[TRACK] Autocalibrado: blanco=%d, negro=%d (el robot debe arrancar sobre blanco)\n",
                valorBlanco, trackValorNegro);
}

// ==========================================
// 10. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  // RX 16 = ESP32-CAM. TX en 19 (libre desde que se quitó el eco frontal bajo).
  // No usar 26: ahora es el eco del ultrasonido lateral.
  Serial2.begin(115200, SERIAL_8N1, 16, 19);

  Wire.begin(21, 22);
  if (!mcp.begin_I2C()) {
    Serial.println("Error al iniciar MCP23017");
    while (1);
  }

  setupMotores();
  setupUltrasonidos();
  pararMotores();

  if (LEER_IR) setupInfrarrojos();

  if (USAR_ELEVADOR) {
    mcp.pinMode(pinStepElevador, OUTPUT);
    mcp.pinMode(pinDirElevador, OUTPUT);
  }

  // El servo solo se activa cuando se necesita: ESP32Servo y analogWrite
  // comparten temporizadores LEDC y así no afecta las pruebas de navegación.
  if (USAR_SERVO) {
    servoCompuerta.attach(pinServoCompuerta);
    servoCompuerta.write(0);
  }

  if (MODO == PRUEBA_DEMO) {
    mcp.pinMode(pinTrackLinea, INPUT_PULLUP);
    delay(20);
    if (AUTO_CALIBRAR_TRACK) autocalibrarTrack();
    else Serial.printf("[TRACK] Valor de negro fijo: %d\n", trackValorNegro);
  }

  if (MODO == PRUEBA_GIRO) {
    estadoActual = FIN_MISION;
    Serial.println("--- CALIBRACIÓN DE GIRO ---");
    Serial.println("'g' = girar 180 | '+'/'-' = +/-100 pulsos | '>'/'<' = +/-20 pulsos");
    Serial.printf("pulsosGiro180 actual = %ld\n", pulsosGiro180);
  } else if (MODO == COMPETENCIA) {
    mcp.pinMode(pinBotonStart, INPUT_PULLUP);
    estadoActual = ESPERA_INICIO;
    Serial.println("--- COMPETENCIA: esperando botón de inicio ---");
  } else if (MODO == PRUEBA_SENSORES) {
    estadoActual = FIN_MISION;
    Serial.println("--- MODO SENSORES: robot quieto, diagnóstico cada 200 ms ---");
    Serial.println("Esperado: 1 = negro, 0 = blanco. Ultrasonidos en cm (999 = sin eco).");
  } else {
    delay(2000); // Pausa de seguridad (igual que la prueba)
    estadoActual = estadoInicial();
    tiempoInicioMision = millis();
    tiempoInicioEstado = millis();
    tiempoArmadoLinea  = millis();
    Serial.printf("--- MODO PRUEBA: arranca en %s ---\n", nombreEstado(estadoActual));
  }
}

// ==========================================
// 11. LOOP PRINCIPAL
// ==========================================
void loop() {
  procesarComandosSerial();
  actualizarControlMotores();
  actualizarLecturasUltrasonido();
  procesarCamaraVision();

  if (millis() - ultimaLecturaCamara > WATCHDOG_CAMARA) {
    pelotaCentrada = false;
    errorCamaraX = 0;
    errorCamaraY = 0;
  }

  // --- Ultrasonidos (misma lógica que la prueba funcional) ---
  int distFrontal = leerUltrasonidoAlto();
  int distLateral = leerUltrasonidoLateral();

  int distParedEfectiva = 999;
  if (distFrontal < 999 && distLateral < 999) distParedEfectiva = min(distFrontal, distLateral);
  else if (distFrontal < 999)                 distParedEfectiva = distFrontal;
  else if (distLateral < 999)                 distParedEfectiva = distLateral;

  // --- Infrarrojos (solo si el modo los usa) ---
  bool lineaFrontal = false, margenIzq = false, margenDer = false;
  if (LEER_IR) {
    lineaFrontal = estaSobreLineaFrontal();
    margenIzq    = detectoMargenIzquierdo();
    margenDer    = detectoMargenDerecho();
  }
  // Demo: la línea la detecta solo el módulo track en PB0, con confirmación
  if (MODO == PRUEBA_DEMO) {
    trackCrudo = mcp.digitalRead(pinTrackLinea);
    bool negro = (trackCrudo == trackValorNegro);
    contadorLinea = negro ? min(contadorLinea + 1, 100) : 0;
    lineaFrontal  = (contadorLinea >= LECTURAS_CONFIRMAR_LINEA);
  }

  bool lineaArmada = (millis() - tiempoArmadoLinea > VENTANA_IGNORAR_LINEA_MS);

  if (MODO == PRUEBA_DEMO) imprimirMonitorDemo(distFrontal, distLateral);

  // Modo diagnóstico: solo imprime, la FSM no corre y los motores siguen en 0
  if (MODO == PRUEBA_SENSORES) {
    imprimirDiagnosticoSensores(distFrontal, distLateral);
    return;
  }

  // ==========================================
  // MÁQUINA DE ESTADOS
  // ==========================================
  switch (estadoActual) {

    case ESPERA_INICIO:
      if (mcp.digitalRead(pinBotonStart) == LOW) {
        tiempoInicioMision = millis();
        tiempoArmadoLinea  = millis();
        cambiarEstado(IR_A_CULTIVO, true);
      }
      break;

    // ---------- NAVEGACIÓN (lógica probada) ----------
    case IR_A_CULTIVO:
      moverOmni(0, VELOCIDAD_BASE, 0);

      // Evalúa AMBOS sensores (igual que la prueba funcional)
      if ((distFrontal >= 3 && distFrontal < UMBRAL_PISCINA_CM) ||
          (USAR_LATERAL_EN_DETECCION && distLateral >= 3 && distLateral < UMBRAL_PISCINA_CM)) {
        Serial.printf("-> Obstáculo detectado. Frontal: %d cm | Lateral: %d cm. Iniciando esquiva...\n",
                      distFrontal, distLateral);
        cambiarEstado(ESQUIVAR_PISCINA, true);
        break;
      }
      if (lineaFrontal && lineaArmada) {
        cambiarEstado(estadoAlDetectarLinea(), true);
      }
      break;

    case ESQUIVAR_PISCINA: {
      int correccionPD = obtenerCorreccionParedPD(distParedEfectiva);
      moverOmni(VELOCIDAD_BASE, correccionPD, 0);

      if (lineaFrontal && lineaArmada) {
        cambiarEstado(estadoAlDetectarLinea(), true);
        break;
      }
      if (distFrontal > UMBRAL_LIBRE_CM && distFrontal < 300) {
        Serial.println("-> Frente despejado. Verificando liberación...");
        cambiarEstado(VERIFICAR_LIBERACION, false);
      }
      // TODO: timeout y cambio de dirección con márgenes IR (siempre esquiva a la derecha)
      break;
    }

    case VERIFICAR_LIBERACION: {
      int correccionPD = obtenerCorreccionParedPD(distParedEfectiva);
      moverOmni(VELOCIDAD_BASE, correccionPD, 0);

      if (lineaFrontal && lineaArmada) {
        cambiarEstado(estadoAlDetectarLinea(), true);
        break;
      }
      if (distFrontal >= 3 && distFrontal <= UMBRAL_LIBRE_CM) {
        cambiarEstado(ESQUIVAR_PISCINA, false);
        break;
      }
      if (millis() - tiempoInicioEstado > TIEMPO_CONFIRMAR_LIBRE_MS) {
        Serial.println("-> Obstáculo superado. Reanudando avance...");
        cambiarEstado(IR_A_CULTIVO, true);
      }
      break;
    }

    // ---------- GIRO 180° (solo PRUEBA_DEMO) ----------
    case GIRO_180: {
      // Pausa corta: el robot frena antes de girar (motores ya en 0)
      if (millis() - tiempoInicioEstado < PAUSA_ANTES_GIRO_MS) break;

      if (!estadoIniciado) {
        estadoIniciado = true;
        resetEncoders();
        inicioGiro = millis();
        Serial.printf("-> Girando 180° (objetivo %ld pulsos)...\n", pulsosGiro180);
      }

      long avance = (labs(getPulsosM1()) + labs(getPulsosM2()) +
                     labs(getPulsosM3()) + labs(getPulsosM4())) / 4;
      bool porTimeout = (millis() - inicioGiro > TIMEOUT_GIRO_MS);

      if (avance >= pulsosGiro180 || porTimeout) {
        pararMotores();
        Serial.printf("-> Giro terminado: %ld pulsos promedio en %lu ms%s\n",
                      avance, millis() - inicioGiro,
                      porTimeout ? "  (TIMEOUT: revisar encoders)" : "");
        tiempoArmadoLinea = millis(); // Ignora la línea un momento al salir
        contadorLinea = 0;
        if (MODO == PRUEBA_GIRO) {
          Serial.println("   ¿Quedó a 180°? Ajusta con '+'/'-' y vuelve a enviar 'g'.");
          cambiarEstado(FIN_MISION, true);
        } else {
          cambiarEstado(IR_A_CULTIVO, true);
        }
        break;
      }
      moverOmni(0, 0, VELOCIDAD_GIRO_DEMO);
      break;
    }

    // ---------- HOMING (esqueleto) ----------
    case HOMING_Y_ESCUADRADO:
      // TODO: escuadrado con los IR frontales antes del strafe
      if (margenIzq || millis() - tiempoInicioEstado > TIMEOUT_HOMING) {
        if (!margenIzq) Serial.println("[HOMING] Timeout sin ver margen izquierdo");
        resetEncoders();
        if (estadoPermitido(RECOLECCION_MULTINIVEL)) {
          moverElevadorANivel(1);
          direccionBarrido = 1;
        }
        cambiarEstado(RECOLECCION_MULTINIVEL, true);
        break;
      }
      moverOmni(-150, 0, 0);
      break;

    // ---------- RECOLECCIÓN (esqueleto) ----------
    case RECOLECCION_MULTINIVEL:
      // TODO: el árbol sigue al frente después de capturar, así que esta
      // condición vuelve a disparar de inmediato. Definir el disparo con la cámara.
      if (distFrontal >= 3 && distFrontal < 25) {
        cambiarEstado(ALINEAR_Y_CAPTURAR, true);
        break;
      }
      if ((direccionBarrido == 1 && margenDer) || (direccionBarrido == -1 && margenIzq)) {
        pararMotores();
        if (nivelActualH < 3) {
          moverElevadorANivel(nivelActualH + 1);
          direccionBarrido *= -1;
        } else {
          moverElevadorANivel(1);
          cambiarEstado(REGRESO_BENEFICIADERO, true);
          break;
        }
      }
      moverOmni(direccionBarrido * 120, 0, 0);
      break;

    case ALINEAR_Y_CAPTURAR:
      if (millis() - tiempoInicioEstado > TIMEOUT_CAPTURA) {
        cambiarEstado(RECOLECCION_MULTINIVEL, true);
        break;
      }
      if (pelotaCentrada) {
        pararMotores();
        ejecutarMecanismoRecolector();
        cambiarEstado(RECOLECCION_MULTINIVEL, true);
      } else {
        int vx = (abs(errorCamaraX) > 10) ? constrain(errorCamaraX, -80, 80) : 0;
        int vy = (abs(errorCamaraY) > 10) ? constrain(errorCamaraY, -80, 80) : 0;
        moverOmni(vx, vy, 0);
      }
      break;

    // ---------- REGRESO Y DESCARGA (esqueleto) ----------
    case REGRESO_BENEFICIADERO:
      // TODO: al entrar, el IR frontal puede seguir sobre la línea del cultivo
      // -> agregar ventana de ignorar línea desde la entrada al estado.
      // TODO: en reversa los ultrasonidos apuntan al lado contrario.
      if (lineaFrontal) {
        cambiarEstado(DESCARGA_PAYLOAD, true);
        break;
      }
      if (distFrontal >= 3 && distFrontal < UMBRAL_PISCINA_CM) moverOmni(150, 0, 0);
      else                                                     moverOmni(0, -180, 0);
      break;

    case DESCARGA_PAYLOAD:
      pararMotores();
      abrirCompuertaDescarga();
      cambiarEstado(FIN_MISION, true);
      break;

    case FIN_MISION:
    case ESTADO_ERROR:
      pararMotores();
      break;
  }
}