// ==========================================
// PESTAÑA PRINCIPAL: Robot-Larc.ino (Refactorizado)
// ==========================================
#include <Arduino.h>

// 3. ESTRUCTURA Y VARIABLES DEL CONTROL PD CON FEEDFORWARD
struct ControlPD {
  float Kp = 0.8f;         // Proporcional: Corrección según la desviación
  float Kd = 0.02f;        // Derivativo: Amortiguación de cambios bruscos
  float setpoint = 0.0f;   // Consigna PWM base (-255 a 255)
  float errorPrev = 0.0f;  // Error del ciclo anterior
  long pulsosAnteriores = 0;
  int outputPWM = 0;
};

extern const int VELOCIDAD_CRUCERO;

void setupMotores();
void resetEncoders();
void moverOmni(int Vx, int Vy, int W);
void pararMotores();

enum EstadoNavegacion {
  IR_A_CULTIVO,
  ESQUIVAR_PISCINA_X,
  AVANZAR_MARGEN,
  HOMING_ESQUINA,
  ESCANEAR_ARBOLES,
  ALINEAR_Y_RECOLECTAR,
  REGRESO_TOLVAS,
  ESTADO_ERROR
};

EstadoNavegacion estadoActual = IR_A_CULTIVO;

unsigned long tiempoInicioEstado = 0;
bool estadoIniciado = false;

const unsigned long TIMEOUT_ESQUIVA = 6000;
const unsigned long TIMEOUT_HOMING  = 7000;
const unsigned long TIMEOUT_SCAN    = 10000;
const unsigned long WATCHDOG_CAMARA = 800; // ms máximo sin trama de visión

int direccionEsquiva = 1; 

int errorCamaraX = 0;
int errorCamaraY = 0;
bool pelotaCentrada = false;
unsigned long ultimaLecturaCamara = 0;

char bufferUART[64];
size_t idxBuffer = 0;

// Variables de filtrado para esquiva de piscina
unsigned long tiempoLibreObstaculo = 0;

void cambiarEstado(EstadoNavegacion nuevoEstado) {
  pararMotores();
  estadoActual = nuevoEstado;
  estadoIniciado = false;
}

void setup() {
  Serial.begin(115200);   
  // RX2 = GPIO 16, TX2 = GPIO 17
  Serial2.begin(115200, SERIAL_8N1, 16, 17); 

  setupMotores();
  pararMotores();
  setupUltrasonidos();
  setupInfrarrojos();
}

void loop() {
  actualizarControlMotores();
  // A. PROCESAMIENTO ASÍNCRONO Y WATCHDOG DE VISIÓN
  procesarCamaraVision();
  if (millis() - ultimaLecturaCamara > WATCHDOG_CAMARA) {
    pelotaCentrada = false;
    errorCamaraX = 0;
    errorCamaraY = 0;
  }

  // B. LECTURA DE SENSORES
  int distFrontalBaja  = leerUltrasonidoFrontalBajo();
  int distAlto         = leerUltrasonidoAlto();
  bool lineaPisoFrente = leerSensorPisoFrente();
  bool lineaPisoLateral= leerSensorPisoLateral();

  // C. MÁQUINA DE ESTADOS FINITOS
  switch (estadoActual) {

    case IR_A_CULTIVO:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      // Filtrado de error: ignorar 0 y valores fuera de rango (>400 cm)
      if (distFrontalBaja > 0 && distFrontalBaja < 20) {
        cambiarEstado(ESQUIVAR_PISCINA_X);
        break;
      }

      if (lineaPisoFrente) {
        cambiarEstado(HOMING_ESQUINA);
        break;
      }

      moverOmni(0, VELOCIDAD_CRUCERO, 0);
      break;

    case ESQUIVAR_PISCINA_X:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        tiempoLibreObstaculo = 0;
        estadoIniciado = true;
      }

      if (millis() - tiempoInicioEstado > TIMEOUT_ESQUIVA) {
        cambiarEstado(ESTADO_ERROR);
        break;
      }

      // Verificación de rango válido (descarta lecturas en 999 o 0)
      if (distFrontalBaja > 35 && distFrontalBaja < 300) {
        if (tiempoLibreObstaculo == 0) {
          tiempoLibreObstaculo = millis();
        } else if (millis() - tiempoLibreObstaculo > 800) { // Requiere 800 ms continuos despejado
          cambiarEstado(IR_A_CULTIVO);
          break;
        }
      } else {
        tiempoLibreObstaculo = 0; // Reinicia el conteo si vuelve a ver el obstáculo
      }

      // Seguridad adicional: Evita salir del terreno si toca la línea lateral mientras esquiva
      if (lineaPisoLateral) {
        direccionEsquiva *= -1; // Invierte la dirección de esquiva
      }

      moverOmni(direccionEsquiva * VELOCIDAD_CRUCERO, 0, 0);
      break;

    case HOMING_ESQUINA:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      // Fallback: Si no detecta línea en 7s, asume alineación por odometría/pared y continúa
      if (millis() - tiempoInicioEstado > TIMEOUT_HOMING) {
        resetEncoders();
        cambiarEstado(ESCANEAR_ARBOLES);
        break;
      }

      if (lineaPisoLateral) {
        resetEncoders();
        cambiarEstado(ESCANEAR_ARBOLES);
        break;
      }

      moverOmni(-VELOCIDAD_CRUCERO, 0, 0); 
      break;

    case ESCANEAR_ARBOLES:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      if (millis() - tiempoInicioEstado > TIMEOUT_SCAN) {
        cambiarEstado(REGRESO_TOLVAS);
        break;
      }

      if (distAlto > 0 && distAlto < 25) {
        cambiarEstado(ALINEAR_Y_RECOLECTAR);
        break;
      }

      moverOmni(120, 0, 0); 
      break;

    case ALINEAR_Y_RECOLECTAR:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      if (pelotaCentrada) {
        pararMotores();
        ejecutarMecanismoRecolector();
        cambiarEstado(ESCANEAR_ARBOLES);
      } else {
        // Aplica escalado proporcional a la velocidad devuelta por visión
        moverOmni(constrain(errorCamaraX, -100, 100), constrain(errorCamaraY, -100, 100), 0);
      }
      break;

    case REGRESO_TOLVAS:
      moverOmni(0, -VELOCIDAD_CRUCERO, 0);
      break;

    case ESTADO_ERROR:
      pararMotores();
      break;
  }
}

void parsearTrama(char* trama) {
  int errX, errY, centrado;
  if (sscanf(trama, "%d,%d,%d", &errX, &errY, &centrado) == 3) {
    errorCamaraX = errX;
    errorCamaraY = errY;
    pelotaCentrada = (centrado == 1);
    ultimaLecturaCamara = millis(); // Actualiza el timestamp del watchdog
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

//__attribute__((weak)) int leerUltrasonidoFrontalBajo() { return 999; }
//__attribute__((weak)) int leerUltrasonidoAlto() { return 999; }
__attribute__((weak)) bool leerSensorPisoFrente() { return false; }
__attribute__((weak)) bool leerSensorPisoLateral() { return false; }
__attribute__((weak)) void ejecutarMecanismoRecolector() { delay(100); }