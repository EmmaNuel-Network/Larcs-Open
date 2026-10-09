// ==========================================
// PESTAÑA PRINCIPAL: Robot-Larc.ino (Refactorizado Completo)
// ==========================================
#include <Arduino.h>

// 1. ESTRUCTURAS Y DECLARACIONES EXTERNAS
struct ControlPD {
  float Kp = 0.8f;         // Proporcional: Corrección según la desviación
  float Kd = 0.02f;        // Derivativo: Amortiguación de cambios bruscos
  float setpoint = 0.0f;   // Consigna PWM base (-255 a 255)
  float errorPrev = 0.0f;  // Error del ciclo anterior
  long pulsosAnteriores = 0;
  int outputPWM = 0;
};

extern const int VELOCIDAD_CRUCERO;
extern const int MIN_PWM_DEADBAND;

// Prototipos de funciones externas
void setupMotores();
void resetEncoders();
void moverOmni(int Vx, int Vy, int W);
void pararMotores();
void setupUltrasonidos();
void setupInfrarrojos();
void actualizarControlMotores();

// 2. ENUMERACIÓN DE LA MÁQUINA DE ESTADOS (FSM)
enum EstadoNavegacion {
  ESPERAR_INICIO,
  IR_A_CULTIVO,
  ESQUIVAR_PISCINA_X,
  AVANZAR_MARGEN,
  HOMING_ESQUINA,
  ESCANEAR_ARBOLES,
  ALINEAR_Y_RECOLECTAR,
  REGRESO_TOLVAS,
  DESCARGAR_TOLVA,
  ESTADO_ERROR
};

EstadoNavegacion estadoActual = IR_A_CULTIVO;

// 3. VARIABLES GLOBALES DE CONTROL Y TIEMPOS
unsigned long tiempoInicioEstado = 0;
bool estadoIniciado = false;
bool lineaLateralPrevia = false;

// Variables y constantes para HOMING_ESQUINA
const unsigned long TIEMPO_CONFIRMACION_LINEA = 80; // 80 ms para confirmar línea real
unsigned long tiempoLineaLateralDetectada = 0;

// Variables y constantes para AVANZAR_MARGEN
const unsigned long MARGEN_AVANCE_PISCINA = 1000; // 1 segundo de avance para superar profundidad de piscina (20 cm)

// Variables para ESCANEAR_ARBOLES
unsigned long tiempoConfirmacionArbol = 0;
const unsigned long TIEMPO_CONFIRMACION_ARBOL = 60; // 60 ms sostenidos sobre el tronco
bool saliendoDeArbol = false;                       // Evita re-enganchar el mismo árbol recién cosechado
int contadorArboles = 0;

// Variables y constantes para ALINEAR_Y_RECOLECTAR
const unsigned long TIMEOUT_ALINEACION = 5000; // 5 segundos máximo para intentar centrar
unsigned long tiempoCentradoConfirmado = 0;
const unsigned long TIEMPO_CONFIRMACION_CENTRADO = 100; // Requiere 100 ms sostenidos de centrado

// Constantes de Timeout globales
const unsigned long TIMEOUT_ESQUIVA  = 6000;
const unsigned long TIMEOUT_HOMING   = 7000;
const unsigned long TIMEOUT_SCAN     = 10000;
const unsigned long TIMEOUT_REGRESO  = 8000;
const unsigned long WATCHDOG_CAMARA  = 800; // ms máximo sin trama de visión

int direccionEsquiva = 1; 

// Variables de visión ESP32-CAM
int errorCamaraX = 0;
int errorCamaraY = 0;
bool pelotaCentrada = false;
unsigned long ultimaLecturaCamara = 0;

char bufferUART[64];
size_t idxBuffer = 0;

// Variables de filtrado para esquiva de piscina
unsigned long tiempoLibreObstaculo = 0;

// 4. FUNCIONES AUXILIARES
int mapearVelocidadVision(int error) {
  if (abs(error) < 5) return 0; // Tolerancia de centrado (Deadband visual)
  int vel = map(abs(error), 5, 120, MIN_PWM_DEADBAND + 15, 110);
  vel = constrain(vel, MIN_PWM_DEADBAND + 15, 110);
  return (error > 0) ? vel : -vel;
}

void cambiarEstado(EstadoNavegacion nuevoEstado) {
  pararMotores();
  estadoActual = nuevoEstado;
  estadoIniciado = false;
}

// 5. INICIALIZACIÓN (SETUP)
void setup() {
  Serial.begin(115200);   
  // RX2 = GPIO 16, TX2 = GPIO 17
  Serial2.begin(115200, SERIAL_8N1, 16, 17); 

  setupMotores();
  pararMotores();
  setupUltrasonidos();
  setupInfrarrojos();
}

// 6. BUCLE PRINCIPAL (LOOP)
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
  int distFrontalBaja   = leerUltrasonidoFrontalBajo();
  int distAlto          = leerUltrasonidoAlto();
  bool lineaPisoFrente  = leerSensorPisoFrente();
  bool lineaPisoLateral = leerSensorPisoLateral();

  // C. MÁQUINA DE ESTADOS FINITOS (FSM)
  switch (estadoActual) {

    case ESPERAR_INICIO:
      pararMotores();
      break;

    case IR_A_CULTIVO:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      // Si detecta piscina a menos de 20 cm, entra a esquivar lateralmente
      if (distFrontalBaja > 0 && distFrontalBaja < 20) {
        cambiarEstado(ESQUIVAR_PISCINA_X);
        break;
      }

      // Si toca la línea de la franja de cultivo
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
        lineaLateralPrevia = false;
        estadoIniciado = true;
      }

      if (millis() - tiempoInicioEstado > TIMEOUT_ESQUIVA) {
        cambiarEstado(ESTADO_ERROR);
        break;
      }

      // Si durante la esquiva lateral encuentra la línea frontal del cultivo, pasa a HOMING
      if (lineaPisoFrente) {
        cambiarEstado(HOMING_ESQUINA);
        break;
      }

      // Verificación de camino despejado (Incluye lecturas válidas > 35 cm o fuera de rango 999)
      if ((distFrontalBaja > 35 && distFrontalBaja < 400) || distFrontalBaja == 999) {
        if (tiempoLibreObstaculo == 0) {
          tiempoLibreObstaculo = millis();
        } else if (millis() - tiempoLibreObstaculo > 800) { // 800 ms sostenidos sin obstáculo
          cambiarEstado(AVANZAR_MARGEN); // Pasa a superar la profundidad de la piscina
          break;
        }
      } else {
        tiempoLibreObstaculo = 0; // Reinicia si vuelve a detectar la piscina
      }

      // Inversión de dirección con detección de FLANCO en margen lateral
      if (lineaPisoLateral && !lineaLateralPrevia) {
        direccionEsquiva *= -1;
      }
      lineaLateralPrevia = lineaPisoLateral;

      moverOmni(direccionEsquiva * VELOCIDAD_CRUCERO, 0, 0);
      break;

    case AVANZAR_MARGEN:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      // Si durante el avance en Y toca la línea frontal de la franja de cultivo
      if (lineaPisoFrente) {
        cambiarEstado(HOMING_ESQUINA);
        break;
      }

      // Si vuelve a encontrarse una piscina de frente
      if (distFrontalBaja > 0 && distFrontalBaja < 20) {
        cambiarEstado(ESQUIVAR_PISCINA_X);
        break;
      }

      // Una vez superado el tiempo de margen de profundidad, vuelve a la navegación normal
      if (millis() - tiempoInicioEstado > MARGEN_AVANCE_PISCINA) {
        cambiarEstado(IR_A_CULTIVO);
        break;
      }

      moverOmni(0, VELOCIDAD_CRUCERO, 0);
      break;

    case HOMING_ESQUINA:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        tiempoLineaLateralDetectada = 0;
        estadoIniciado = true;
      }

      // Fallback: Si no detecta línea en 7s, frena, ajusta y continúa por seguridad
      if (millis() - tiempoInicioEstado > TIMEOUT_HOMING) {
        pararMotores();
        resetEncoders();
        cambiarEstado(ESCANEAR_ARBOLES);
        break;
      }

      // Detección con filtro de estabilidad para lineaPisoLateral
      if (lineaPisoLateral) {
        if (tiempoLineaLateralDetectada == 0) {
          tiempoLineaLateralDetectada = millis();
        } else if (millis() - tiempoLineaLateralDetectada >= TIEMPO_CONFIRMACION_LINEA) {
          pararMotores();
          delay(50); 
          resetEncoders();
          cambiarEstado(ESCANEAR_ARBOLES);
          break;
        }
      } else {
        tiempoLineaLateralDetectada = 0;
      }

      // Corrección de Deriva en Y para mantener la línea frontal
      int velocidadY_correccionHoming = 0;
      if (!lineaPisoFrente) {
        velocidadY_correccionHoming = 45; 
      }

      moverOmni(-VELOCIDAD_CRUCERO, velocidadY_correccionHoming, 0);
      break;

    case ESCANEAR_ARBOLES:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        tiempoConfirmacionArbol = 0;
        saliendoDeArbol = true; // Activa la bandera de despeje
        estadoIniciado = true;
      }

      // Interrupción por Fin de Campo (Límite lateral derecho alcanzado)
      if (lineaPisoLateral) {
        pararMotores();
        cambiarEstado(REGRESO_TOLVAS);
        break;
      }

      // Fallback por Timeout
      if (millis() - tiempoInicioEstado > TIMEOUT_SCAN) {
        pararMotores();
        cambiarEstado(REGRESO_TOLVAS);
        break;
      }

      // Mecanismo de Despeje: avanzar hasta alejarse del árbol recién cosechado
      if (saliendoDeArbol) {
        if (distAlto > 30 || distAlto == 999) {
          saliendoDeArbol = false;
        }
      } else {
        // Búsqueda y filtrado del NUEVO árbol
        if (distAlto > 0 && distAlto < 25) {
          if (tiempoConfirmacionArbol == 0) {
            tiempoConfirmacionArbol = millis();
          } else if (millis() - tiempoConfirmacionArbol >= TIEMPO_CONFIRMACION_ARBOL) {
            pararMotores();
            delay(50);
            contadorArboles++;
            cambiarEstado(ALINEAR_Y_RECOLECTAR);
            break;
          }
        } else {
          tiempoConfirmacionArbol = 0;
        }
      }

      // Corrección de Deriva en Y
      int velocidadY_correccionScan = 0;
      if (!lineaPisoFrente) {
        velocidadY_correccionScan = 40;
      }

      moverOmni(120, velocidadY_correccionScan, 0);
      break;

    case ALINEAR_Y_RECOLECTAR:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        tiempoCentradoConfirmado = 0;
        estadoIniciado = true;
      }

      // Timeout de seguridad: aborta centrado en 5 segundos si pierde la pelota
      if (millis() - tiempoInicioEstado > TIMEOUT_ALINEACION) {
        pararMotores();
        cambiarEstado(ESCANEAR_ARBOLES);
        break;
      }

      // Verificación de centrado sostenido
      if (pelotaCentrada) {
        if (tiempoCentradoConfirmado == 0) {
          tiempoCentradoConfirmado = millis();
        } else if (millis() - tiempoCentradoConfirmado >= TIEMPO_CONFIRMACION_CENTRADO) {
          pararMotores();
          delay(150); 
          
          ejecutarMecanismoRecolector();
          
          delay(200);
          cambiarEstado(ESCANEAR_ARBOLES);
          break;
        }
      } else {
        tiempoCentradoConfirmado = 0;
      }

      // Control Proporcional Ajustado con Deadband
      int vx_ajuste = mapearVelocidadVision(errorCamaraX);
      int vy_ajuste = mapearVelocidadVision(errorCamaraY);

      moverOmni(vx_ajuste, vy_ajuste, 0);
      break;

    case REGRESO_TOLVAS:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      // Fallback por Timeout
      if (millis() - tiempoInicioEstado > TIMEOUT_REGRESO) {
        pararMotores();
        delay(100);
        cambiarEstado(DESCARGAR_TOLVA);
        break;
      }

      // Detección de línea del Beneficiadero
      if (leerSensorPisoFrente() || leerSensorPisoLateral()) {
        pararMotores();
        delay(150);
        cambiarEstado(DESCARGAR_TOLVA);
        break;
      }

      moverOmni(0, -VELOCIDAD_CRUCERO, 0);
      break;

    case DESCARGAR_TOLVA:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        pararMotores();
        
        ejecutarDescargaTolva(); 
        
        estadoIniciado = true;
      }

      // Espera 3 segundos para el vaciado completo de los frutos
      if (millis() - tiempoInicioEstado > 3000) {
        pararMotores();
        cambiarEstado(ESPERAR_INICIO); // Misión cumplida: listo para reiniciar
      }
      break;

    case ESTADO_ERROR:
      pararMotores();
      break;
  }
}

// 7. PARSER Y RECEPCIÓN UART (ESP32-CAM)
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

// 8. FUNCIONES COMODÍN (WEAK STUBS PARA COMPILACIÓN)
__attribute__((weak)) int leerUltrasonidoFrontalBajo() { return 999; }
__attribute__((weak)) int leerUltrasonidoAlto() { return 999; }
__attribute__((weak)) bool leerSensorPisoFrente() { return false; }
__attribute__((weak)) bool leerSensorPisoLateral() { return false; }
__attribute__((weak)) void ejecutarMecanismoRecolector() { delay(100); }
__attribute__((weak)) void ejecutarDescargaTolva() { delay(100); }
__attribute__((weak)) void setupUltrasonidos() {}
__attribute__((weak)) void setupInfrarrojos() {}
__attribute__((weak)) void actualizarControlMotores() {}
