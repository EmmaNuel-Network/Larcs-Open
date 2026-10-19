// ==========================================
// PESTAÑA PRINCIPAL: Robot-Larc.ino (Versión con Recolector Servo 180° -> 100°)
// ==========================================
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>

Adafruit_MCP23X17 mcp;

struct ControlPD {
  float setpoint = 0.0f;
  long pulsosAnteriores = 0;
  float errorPrev = 0.0f;
  float Kp = 1.2f;  // Ganancia Proporcional base
  float Kd = 0.15f; // Ganancia Derivativa base
  int outputPWM = 0;
};

// 1. ESTADOS DE LA FSM PRINCIPAL (TDP Octavio I)
enum EstadoFSM {
  ESPERA_INICIO,
  IR_A_CULTIVO,
  ESQUIVAR_PISCINA,
  HOMING_Y_ESCUADRADO,
  RECOLECCION_MULTINIVEL,
  ALINEAR_Y_CAPTURAR,
  REGRESO_BENEFICIADERO,
  DESCARGA_PAYLOAD,
  FIN_MISION,
  ESTADO_ERROR
};

EstadoFSM estadoActual = ESPERA_INICIO;

// 2. PINES Y ACTUADORES
const int pinBotonStart        = 13; // Botón de inicio en MCP23017 (PB5)
const int pinServoCompuerta    = 25; // Servo de descarga trasera (GPIO 25)
const int pinServoRecolector   = 0;  // Servo de recolección Rampa/Cesta (GPIO 0)

Servo servoCompuerta;
Servo servoRecolector;

// ÁNGULOS DEL SERVO RECOLECTOR SG90
const int ANGULO_REPOSO_REC   = 180; // Centrado en 180 grados (reposo/recepción)
const int ANGULO_CAPTURA_REC  = 100; // Retroceso a 100 grados (volcado de la pelota)

// CONTROL DEL ELEVADOR (Rack & Pinion)
const int pinStepElevador      = 14; // MCP23017 PB6
const int pinDirElevador       = 15; // MCP23017 PB7

const float PASOS_POR_MM       = 15.0f;  // Calibración de cremallera
const float DISTANCIA_NIVEL_MM = 150.0f; // Distancia entre niveles (150 mm)
int nivelActualH               = 1;      // Niveles: 1 (H1: 390mm), 2 (H2: 540mm), 3 (H3: 690mm)

// 3. VARIABLES DE CONTROL DE TIEMPO Y NAVEGACIÓN
unsigned long tiempoInicioEstado   = 0;
bool estadoIniciado                = false;

const unsigned long TIMEOUT_ESQUIVA = 6000;
const unsigned long TIMEOUT_HOMING  = 7000;
const unsigned long TIMEOUT_CAPTURA = 4000; // Maximum timeout para alineación de captura (4s)
const unsigned long WATCHDOG_CAMARA = 800;

int direccionEsquiva               = 1; 
unsigned long tiempoLibreObstaculo = 0;
unsigned long ultimoCambioEsquiva  = 0; // Antirrebote para cambio de esquiva

int direccionBarrido               = 1; // 1 = Derecha (+Vx), -1 = Izquierda (-Vx)

// Datos recibidos desde ESP32-CAM por UART2 (GPIO 16 RX2, GPIO 26 TX2)
int errorCamaraX                    = 0;
int errorCamaraY                    = 0;
bool pelotaCentrada                 = false;
unsigned long ultimaLecturaCamara   = 0;

char bufferUART[64];
size_t idxBuffer = 0;

// Declaración de funciones externas (Motores, Infrarrojos, Ultrasonidos)
void setupMotores();
void actualizarControlMotores();
void moverOmni(int Vx, int Vy, int W);
void pararMotores();
void resetEncoders();

void setupInfrarrojos();
bool estaSobreLineaFrontal();
bool detectoMargenIzquierdo();
bool detectoMargenDerecho();

void setupUltrasonidos();
int leerUltrasonidoFrontalBajo();
int leerUltrasonidoAlto();

// 4. CAMBIO DE ESTADO
void cambiarEstado(EstadoFSM nuevoEstado) {
  pararMotores();
  estadoActual = nuevoEstado;
  estadoIniciado = false;
  tiempoInicioEstado = millis();
}

// 5. FUNCIONES DE ACTUACIÓN SECUNDARIAS
void moverElevadorANivel(int nivel) {
  pararMotores(); // Detiene inercia de tracción antes de energizar el elevador
  if (nivel == nivelActualH) return;

  mcp.digitalWrite(pinDirElevador, (nivel > nivelActualH) ? HIGH : LOW);
  
  // Cálculo de pasos según distancia (150 mm por nivel * 15 pasos/mm = 2250 pasos)
  int pasoDiferencia = abs(nivel - nivelActualH);
  int pasosTotales = (int)(pasoDiferencia * DISTANCIA_NIVEL_MM * PASOS_POR_MM);

  for (int i = 0; i < pasosTotales; i++) {
    mcp.digitalWrite(pinStepElevador, HIGH);
    delayMicroseconds(800); 
    mcp.digitalWrite(pinStepElevador, LOW);
    delayMicroseconds(800);
  }
  nivelActualH = nivel;

  // Purga del buffer UART para descartar tramas acumuladas durante el movimiento
  while (Serial2.available() > 0) Serial2.read();
  ultimaLecturaCamara = millis();
}

void abrirCompuertaDescarga() {
  servoCompuerta.write(90); // Ángulo de apertura de descarga
  delay(2000);              // Mantiene abierta 2s
  servoCompuerta.write(0);  // Cierra compuerta
  
  while (Serial2.available() > 0) Serial2.read();
}

// MECANISMO RECOLECTOR: CENTRADO EN 180° Y RETROCESO A 100°
void ejecutarMecanismoRecolector() {
  Serial.println("-> Accionando Servo Recolector (180° -> 100° -> 180°)...");
  
  // 1. Retrocede suavemente desde 180° hasta 100°
  for (int ang = ANGULO_REPOSO_REC; ang >= ANGULO_CAPTURA_REC; ang -= 2) {
    servoRecolector.write(ang);
    delay(20);
  }
  
  delay(1200); // Mantiene en 100° para asegurar que la pelota caiga

  // 2. Regresa progresivamente a la posición centrada (180°)
  for (int ang = ANGULO_CAPTURA_REC; ang <= ANGULO_REPOSO_REC; ang += 2) {
    servoRecolector.write(ang);
    delay(20);
  }
  
  delay(600); // Pausa de estabilización mecánica

  // Limpieza de buffer UART tras delays
  while (Serial2.available() > 0) Serial2.read();
  ultimaLecturaCamara = millis();
}

// 6. PROCESAMIENTO UART DE LA ESP32-CAM
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
      if (idxBuffer < sizeof(bufferUART) - 1) {
        bufferUART[idxBuffer++] = c;
      } else {
        idxBuffer = 0; 
      }
    }
  }
}

// 7. SETUP
void setup() {
  Serial.begin(115200);   
  Serial2.begin(115200, SERIAL_8N1, 16, 26); // RX2 = 16, TX2 = 26

  // Inicialización de Bus I2C y Expansor MCP23017
  Wire.begin(21, 22); // SDA = 21, SCL = 22
  if (!mcp.begin_I2C()) {
    Serial.println("Error al iniciar el MCP23017. Revisa conexiones.");
    while (1); 
  }

  mcp.pinMode(pinBotonStart, INPUT_PULLUP);
  mcp.pinMode(pinStepElevador, OUTPUT);
  mcp.pinMode(pinDirElevador, OUTPUT);

  // Inicialización de Servomotores (ESP32Servo)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  servoCompuerta.setPeriodHertz(50);
  servoCompuerta.attach(pinServoCompuerta, 500, 2400);
  servoCompuerta.write(0);

  servoRecolector.setPeriodHertz(50);
  servoRecolector.attach(pinServoRecolector, 500, 2400);
  servoRecolector.write(ANGULO_REPOSO_REC); // Inicia centrado a 180°

  setupMotores();
  pararMotores();
  setupUltrasonidos();
  setupInfrarrojos();

  Serial.println("=== ROBOT OCTAVIO I INICIALIZADO ===");
}

// 8. LOOP PRINCIPAL CON FSM
void loop() {
  actualizarControlMotores();
  procesarCamaraVision();

  // Watchdog de visión (reinicia consignas si se pierde comunicación)
  if (millis() - ultimaLecturaCamara > WATCHDOG_CAMARA) {
    pelotaCentrada = false;
    errorCamaraX = 0;
    errorCamaraY = 0;
  }

  // Lectura continua de sensores
  int distFrontalBaja  = leerUltrasonidoFrontalBajo();
  int distAlto         = leerUltrasonidoAlto();
  bool lineaFrontal    = estaSobreLineaFrontal();
  bool margenIzq       = detectoMargenIzquierdo();
  bool margenDer       = detectoMargenDerecho();

  // MÁQUINA DE ESTADOS FINITOS
  switch (estadoActual) {

    case ESPERA_INICIO:
      if (mcp.digitalRead(pinBotonStart) == LOW) {
        cambiarEstado(IR_A_CULTIVO);
      }
      break;

    case IR_A_CULTIVO:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      if (distFrontalBaja > 0 && distFrontalBaja < 20) {
        cambiarEstado(ESQUIVAR_PISCINA);
        break;
      }
      if (lineaFrontal) {
        cambiarEstado(HOMING_Y_ESCUADRADO);
        break;
      }
      moverOmni(0, 180, 0); // Avance longitudinal (+Vy)
      break;

    case ESQUIVAR_PISCINA:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      if (millis() - tiempoInicioEstado > TIMEOUT_ESQUIVA) {
        cambiarEstado(ESTADO_ERROR);
        break;
      }

      if (lineaFrontal) {
        cambiarEstado(HOMING_Y_ESCUADRADO);
        break;
      }

      if (distFrontalBaja > 35 && distFrontalBaja < 300) {
        if (tiempoLibreObstaculo == 0) {
          tiempoLibreObstaculo = millis();
        } else if (millis() - tiempoLibreObstaculo > 800) {
          tiempoLibreObstaculo = 0;
          cambiarEstado(IR_A_CULTIVO);
          break;
        }
      } else {
        tiempoLibreObstaculo = 0;
      }

      if ((margenIzq || margenDer) && (millis() - ultimoCambioEsquiva > 600)) {
        direccionEsquiva *= -1;
        ultimoCambioEsquiva = millis();
      }

      moverOmni(direccionEsquiva * 180, 0, 0);
      break;

    case HOMING_Y_ESCUADRADO:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      if (millis() - tiempoInicioEstado > TIMEOUT_HOMING) {
        resetEncoders();
        moverElevadorANivel(1); // Elevador a Nivel H1 (390 mm)
        direccionBarrido = 1;
        cambiarEstado(RECOLECCION_MULTINIVEL);
        break;
      }
      if (margenIzq) {
        resetEncoders();
        moverElevadorANivel(1);
        direccionBarrido = 1;
        cambiarEstado(RECOLECCION_MULTINIVEL);
        break;
      }
      moverOmni(-150, 0, 0); // Strafe a la izquierda
      break;

    case RECOLECCION_MULTINIVEL:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      // Detección de pelota por ultrasonido alto
      if (distAlto > 0 && distAlto < 25) {
        cambiarEstado(ALINEAR_Y_CAPTURAR);
        break;
      }

      // Verificación de fin de fila
      if ((direccionBarrido == 1 && margenDer) || (direccionBarrido == -1 && margenIzq)) {
        pararMotores();
        if (nivelActualH < 3) {
          moverElevadorANivel(nivelActualH + 1); // Sube al nivel H2 o H3 (+150 mm)
          direccionBarrido *= -1;                 // Invierte recorrido zigzag
        } else {
          moverElevadorANivel(1);                // Regresa a H1 para estabilidad
          cambiarEstado(REGRESO_BENEFICIADERO);
          break;
        }
      }

      moverOmni(direccionBarrido * 120, 0, 0); 
      break;

    case ALINEAR_Y_CAPTURAR:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      if (millis() - tiempoInicioEstado > TIMEOUT_CAPTURA) {
        cambiarEstado(RECOLECCION_MULTINIVEL);
        break;
      }

      if (pelotaCentrada) {
        pararMotores();
        ejecutarMecanismoRecolector(); // Ejecuta movimiento 180° -> 100° -> 180°
        cambiarEstado(RECOLECCION_MULTINIVEL);
      } else {
        int vx = (abs(errorCamaraX) > 10) ? constrain(errorCamaraX, -80, 80) : 0;
        int vy = (abs(errorCamaraY) > 10) ? constrain(errorCamaraY, -80, 80) : 0;
        moverOmni(vx, vy, 0);
      }
      break;

    case REGRESO_BENEFICIADERO:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      if (lineaFrontal) {
        cambiarEstado(DESCARGA_PAYLOAD);
        break;
      }
      if (distFrontalBaja > 0 && distFrontalBaja < 20) {
        moverOmni(150, 0, 0);
      } else {
        moverOmni(0, -180, 0);
      }
      break;

    case DESCARGA_PAYLOAD:
      pararMotores();
      abrirCompuertaDescarga();
      cambiarEstado(FIN_MISION);
      break;

    case FIN_MISION:
      pararMotores();
      break;

    case ESTADO_ERROR:
      pararMotores();
      break;
  }
}
