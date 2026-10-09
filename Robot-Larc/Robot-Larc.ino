// ==========================================
// PESTAÑA PRINCIPAL: Robot-Larc.ino (Versión Corregida y Robusta)
// ==========================================
#include <Arduino.h>
#include <ESP32Servo.h>

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

// 2. PINES Y ACTUADORES ADICIONALES
const int pinBotonStart      = 15; // Botón de inicio de competencia
const int pinServoCompuerta  = 18; // Servo de descarga trasera
Servo servoCompuerta;

// Control del Elevador NEMA-17 (Rack & Pinion)
const int pinStepElevador    = 22;
const int pinDirElevador     = 23;
int nivelActualH             = 1;  // Niveles: 1 (H1), 2 (H2), 3 (H3)

// 3. VARIABLES DE CONTROL DE TIEMPO Y NAVEGACIÓN
unsigned long tiempoInicioEstado   = 0;
bool estadoIniciado                = false;

const unsigned long TIMEOUT_ESQUIVA = 6000;
const unsigned long TIMEOUT_HOMING  = 7000;
const unsigned long TIMEOUT_CAPTURA = 4000; // CORRECCIÓN: Maximum timeout para alineación de captura (4s)
const unsigned long WATCHDOG_CAMARA = 800;

int direccionEsquiva               = 1; 
unsigned long tiempoLibreObstaculo = 0;
unsigned long ultimoCambioEsquiva  = 0; // Antirrebote para el cambio de esquiva

int direccionBarrido               = 1; // 1 = Derecha (+Vx), -1 = Izquierda (-Vx)

// Datos recibidos desde ESP32-CAM por UART2 (GPIO 16 RX2, GPIO 17 TX2)
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
  pararMotores(); // Detiene inercia de tracción antes de energizar paso a paso
  if (nivel == nivelActualH) return;

  digitalWrite(pinDirElevador, (nivel > nivelActualH) ? HIGH : LOW);
  int pasos = abs(nivel - nivelActualH) * 400; // Pasos según relación mecánica

  for (int i = 0; i < pasos; i++) {
    digitalWrite(pinStepElevador, HIGH);
    delayMicroseconds(800); // 800us para asegurar torque con carga
    digitalWrite(pinStepElevador, LOW);
    delayMicroseconds(800);
  }
  nivelActualH = nivel;

  // CORRECCIÓN: Purga el buffer UART para descartar tramas desactualizadas acumuladas durante el movimiento
  while (Serial2.available() > 0) Serial2.read();
  ultimaLecturaCamara = millis(); // Reinicia el temporizador del Watchdog
}

void abrirCompuertaDescarga() {
  servoCompuerta.write(90); // Ángulo de apertura de descarga
  delay(2000);              // Mantiene la compuerta abierta 2.0s
  servoCompuerta.write(0);  // Cierra compuerta
  
  // Limpieza de buffer tras el delay bloqueante
  while (Serial2.available() > 0) Serial2.read();
}

void ejecutarMecanismoRecolector() {
  delay(250);
  while (Serial2.available() > 0) Serial2.read();
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
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // Comunicación UART2 con ESP32-CAM

  pinMode(pinBotonStart, INPUT_PULLUP);
  pinMode(pinStepElevador, OUTPUT);
  pinMode(pinDirElevador, OUTPUT);

  servoCompuerta.attach(pinServoCompuerta);
  servoCompuerta.write(0); // Compuerta cerrada por defecto

  setupMotores();
  pararMotores();
  setupUltrasonidos();
  setupInfrarrojos();
}

// 8. LOOP PRINCIPAL CON FSM
void loop() {
  actualizarControlMotores();
  procesarCamaraVision();

  // Watchdog de visión (reinicia consignas si pierde la comunicación)
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
      if (digitalRead(pinBotonStart) == LOW) { // Botón presionado (PULLUP)
        cambiarEstado(IR_A_CULTIVO);
      }
      break;

    case IR_A_CULTIVO:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      // Detección de piscinas centrales (d < 20 cm)
      if (distFrontalBaja > 0 && distFrontalBaja < 20) {
        cambiarEstado(ESQUIVAR_PISCINA);
        break;
      }
      // Detección de la línea límite de la zona de cultivo
      if (lineaFrontal) {
        cambiarEstado(HOMING_Y_ESCUADRADO);
        break;
      }
      moverOmni(0, 180, 0); // Avance continuo longitudinal (+Vy)
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

      // CORRECCIÓN: Si detecta la línea frontal mientras esquiva lateralmente, cambia inmediatamente a Homing
      if (lineaFrontal) {
        cambiarEstado(HOMING_Y_ESCUADRADO);
        break;
      }

      // Confirmación de ruta despejada durante 800 ms
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

      // Rebote de seguridad en bordes laterales con antirrebote (600 ms)
      if ((margenIzq || margenDer) && (millis() - ultimoCambioEsquiva > 600)) {
        direccionEsquiva *= -1;
        ultimoCambioEsquiva = millis();
      }

      moverOmni(direccionEsquiva * 180, 0, 0); // Desplazamiento lateral holonómico (+/- Vx)
      break;

    case HOMING_Y_ESCUADRADO:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      if (millis() - tiempoInicioEstado > TIMEOUT_HOMING) {
        resetEncoders();
        moverElevadorANivel(1); // Inicia elevador en Nivel H1
        direccionBarrido = 1;   // Inicia barrido hacia la derecha
        cambiarEstado(RECOLECCION_MULTINIVEL);
        break;
      }
      // Desplazamiento al origen lateral (X0, Y0)
      if (margenIzq) {
        resetEncoders();
        moverElevadorANivel(1);
        direccionBarrido = 1;
        cambiarEstado(RECOLECCION_MULTINIVEL);
        break;
      }
      moverOmni(-150, 0, 0); // Strafe a la izquierda hacia el origen
      break;

    case RECOLECCION_MULTINIVEL:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      // Detección de objetivo por ultrasonido alto
      if (distAlto > 0 && distAlto < 25) {
        cambiarEstado(ALINEAR_Y_CAPTURAR);
        break;
      }

      // Verificación de fin de fila según la dirección de avance actual
      if ((direccionBarrido == 1 && margenDer) || (direccionBarrido == -1 && margenIzq)) {
        pararMotores(); // Detiene tracción antes de mover elevador
        if (nivelActualH < 3) {
          moverElevadorANivel(nivelActualH + 1); // Sube al siguiente nivel (H2 o H3)
          direccionBarrido *= -1;                 // Invierte sentido de avance (recorrido en zigzag)
        } else {
          moverElevadorANivel(1);                // Regresa elevador a H1 para estabilidad en tránsito
          cambiarEstado(REGRESO_BENEFICIADERO);
          break;
        }
      }

      // Avance lateral con sentido dinámico (+/- Vx)
      moverOmni(direccionBarrido * 120, 0, 0); 
      break;

    case ALINEAR_Y_CAPTURAR:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      // CORRECCIÓN: Timeout de seguridad (4s). Evita bloqueos infinitos si la visión pierde la fruta
      if (millis() - tiempoInicioEstado > TIMEOUT_CAPTURA) {
        cambiarEstado(RECOLECCION_MULTINIVEL);
        break;
      }

      if (pelotaCentrada) {
        pararMotores();
        ejecutarMecanismoRecolector();
        cambiarEstado(RECOLECCION_MULTINIVEL);
      } else {
        // Corrección holonómica con Zona Muerta (Deadband = 10 px)
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
      // Retorno hacia el beneficiadero con esquiva rápida si detecta la piscina
      if (distFrontalBaja > 0 && distFrontalBaja < 20) {
        moverOmni(150, 0, 0); // Esquiva lateral
      } else {
        moverOmni(0, -180, 0); // Retroceso hacia la base (-Vy)
      }
      break;

    case DESCARGA_PAYLOAD:
      pararMotores();
      abrirCompuertaDescarga(); // Ejecuta apertura/cierre de la compuerta trasera
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