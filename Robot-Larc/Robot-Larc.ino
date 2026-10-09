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
  DESCARGAR_TOLVA,
  ESTADO_ERROR
};

EstadoNavegacion estadoActual = IR_A_CULTIVO;

unsigned long tiempoInicioEstado = 0;
bool estadoIniciado = false;
bool lineaLateralPrevia = false;
// Variables globales / estáticas necesarias para el escaneo
unsigned long tiempoConfirmacionArbol = 0;
const unsigned long TIEMPO_CONFIRMACION_ARBOL = 60; // 60 ms sostenidos sobre el tronco
bool saliendoDeArbol = false;                        // Evita volver a enganchar el mismo árbol recién cosechado
int contadorArboles = 0;
// Variables y constantes para ALINEAR_Y_RECOLECTAR
const unsigned long TIMEOUT_ALINEACION = 5000; // 5 segundos máximo para intentar centrar
unsigned long tiempoCentradoConfirmado = 0;
const unsigned long TIEMPO_CONFIRMACION_CENTRADO = 100; // Requiere 100 ms sostenidos de centrado

// Función auxiliar para aplicar zona muerta en alineación por visión
int mapearVelocidadVision(int error) {
  if (abs(error) < 5) return 0; // Tolerancia de centrado (Deadband visual)
  int vel = map(abs(error), 5, 120, MIN_PWM_DEADBAND + 15, 110);
  vel = constrain(vel, MIN_PWM_DEADBAND + 15, 110);
  return (error > 0) ? vel : -vel;
}
// Constante de tiempo límite para el viaje de regreso
const unsigned long TIMEOUT_REGRESO = 8000; // 8 segundos máximo de reversa

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
//  Si durante la esquiva lateral encuentra la línea frontal del cultivo, pasa a HOMING
      if (lineaPisoFrente) {
        cambiarEstado(HOMING_ESQUINA);
        break;
      }
      // Verificación de rango válido (descarta lecturas en 999 o 0)
      if ((distFrontalBaja > 35 && distFrontalBaja < 400) || distFrontalBaja == 999) {
        if (tiempoLibreObstaculo == 0) {
          tiempoLibreObstaculo = millis();
        } else if (millis() - tiempoLibreObstaculo > 800) { // 800 ms sostenidos sin obstáculo
          cambiarEstado(IR_A_CULTIVO);
          break;
        }
      } else {
        tiempoLibreObstaculo = 0; // Reinicia si vuelve a detectar la piscina
      }

      // 4. Inversión de dirección con detección de FLANCO (Evita oscilación rápida)
      if (lineaPisoLateral && !lineaLateralPrevia) {
        direccionEsquiva *= -1; // Invierte dirección solo en el instante que toca la línea
      }
      lineaLateralPrevia = lineaPisoLateral; // Actualiza el estado previ
      moverOmni(direccionEsquiva * VELOCIDAD_CRUCERO, 0, 0);
      break;

    case HOMING_ESQUINA:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        tiempoLineaLateralDetectada = 0;
        estadoIniciado = true;
      }

      // Fallback: Si no detecta línea en 7s, asume alineación por odometría/pared y continúa
      if (millis() - tiempoInicioEstado > TIMEOUT_HOMING) {
        pararMotores();
        resetEncoders();
        cambiarEstado(ESCANEAR_ARBOLES);
        break;
      }
// 2. Detección con filtro de estabilidad para lineaPisoLateral (Elimina ruido óptico)
      if (lineaPisoLateral) {
        if (tiempoLineaLateralDetectada == 0) {
          tiempoLineaLateralDetectada = millis();
        } else if (millis() - tiempoLineaLateralDetectada >= TIEMPO_CONFIRMACION_LINEA) {
          // LLegó a la esquina real: frena, estabiliza chasis y resetea odometría
          pararMotores();
          delay(50); 
          resetEncoders();
          cambiarEstado(ESCANEAR_ARBOLES);
          break;
        }
      } else {
        tiempoLineaLateralDetectada = 0; // Reinicia el filtro si fue solo un destello efímero
      }

      // 3. Corrección de Deriva en Y: Mantener la línea frontal mientras se mueve a la izquierda
      int velocidadY_correccion = 0;
      if (!lineaPisoFrente) {
        // Si el robot empieza a derivar hacia atrás perdiendo la línea frontal, empuja levemente en Y
        velocidadY_correccion = 45; 
      }

      // 4. Traslación lateral a la izquierda con ajuste de retención en Y
      moverOmni(-VELOCIDAD_CRUCERO, velocidadY_correccion, 0);
      break;

    case ESCANEAR_ARBOLES:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        tiempoConfirmacionArbol = 0;
        saliendoDeArbol = true; // Activa la bandera de despeje al iniciar o regresar de recolectar
        estadoIniciado = true;
      }

      // 1. Interrupción por Fin de Campo (Límite lateral derecho alcanzado)
      if (lineaPisoLateral) {
        pararMotores();
        cambiarEstado(REGRESO_TOLVAS);
        break;
      }

      // 2. Fallback por Timeout de seguridad
      if (millis() - tiempoInicioEstado > TIMEOUT_SCAN) {
        pararMotores();
        cambiarEstado(REGRESO_TOLVAS);
        break;
      }

      // 3. Mecanismo de Despeje: avanzar hasta alejarse del árbol recién cosechado
      if (saliendoDeArbol) {
        if (distAlto > 30 || distAlto == 999) {
          saliendoDeArbol = false; // Ya superó el tronco anterior, habilita la búsqueda del siguiente
        }
      } else {
        // 4. Búsqueda y filtrado del NUEVO árbol
        if (distAlto > 0 && distAlto < 25) {
          if (tiempoConfirmacionArbol == 0) {
            tiempoConfirmacionArbol = millis();
          } else if (millis() - tiempoConfirmacionArbol >= TIEMPO_CONFIRMACION_ARBOL) {
            // Confirmado: Tronco detectado
            pararMotores();
            delay(50);
            contadorArboles++;
            cambiarEstado(ALINEAR_Y_RECOLECTAR);
            break;
          }
        } else {
          tiempoConfirmacionArbol = 0; // Reinicia si fue una lectura errática efímera
        }
      }

      // 5. Corrección de Deriva en Y: Ajusta si el chasis se separa de la franja frontal
      int velocidadY_correccion = 0;
      if (!lineaPisoFrente) {
        velocidadY_correccion = 40; // Mantiene el robot ceñido hacia el frente
      }

      // Avanza lateralmente hacia la derecha escaneando
      moverOmni(120, velocidadY_correccion, 0);
      break;
    case ALINEAR_Y_RECOLECTAR:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        tiempoCentradoConfirmado = 0;
        estadoIniciado = true;
      }
      // 1. Timeout de seguridad: Si en 5 segundos no logra centrar, aborta y sigue escaneando
      if (millis() - tiempoInicioEstado > TIMEOUT_ALINEACION) {
        pararMotores();
        cambiarEstado(ESCANEAR_ARBOLES);
        break;
      }
      // 2. Verificación de centrado sostenido (Filtro anti-vibración)
      if (pelotaCentrada) {
        if (tiempoCentradoConfirmado == 0) {
          tiempoCentradoConfirmado = millis();
        } else if (millis() - tiempoCentradoConfirmado >= TIEMPO_CONFIRMACION_CENTRADO) {
          // Centrado confirmado: frena, deja estabilizar el chasis y recolecta
          pararMotores();
          delay(150); 
          
          ejecutarMecanismoRecolector(); // Acciona la rampa/servo/elevador
          
          delay(200); // Pausa post-cosecha
          cambiarEstado(ESCANEAR_ARBOLES); // Vuelve al escaneo de la fila
          break;
        }
      } else {
        tiempoCentradoConfirmado = 0; // Reinicia si pierde el centrado momentáneamente
      }

      // 3. Control Proporcional Ajustado con Deadband
      int vx_ajuste = mapearVelocidadVision(errorCamaraX);
      int vy_ajuste = mapearVelocidadVision(errorCamaraY);

      // Si no está centrada pero el error es muy bajo, se detiene levemente para no sobrepasar el blanco
      moverOmni(vx_ajuste, vy_ajuste, 0);
      break;

case REGRESO_TOLVAS:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      // 1. Fallback por Timeout: Si en 8s no ve la línea, asume posición cercana y pasa a descargar
      if (millis() - tiempoInicioEstado > TIMEOUT_REGRESO) {
        pararMotores();
        delay(100);
        cambiarEstado(DESCARGAR_TOLVA);
        break;
      }

      // 2. Detección de la línea negra de la zona de Beneficiadero
      if (leerSensorPisoFrente() || leerSensorPisoLateral()) {
        pararMotores();
        delay(150); // Frenado e inercia cero
        cambiarEstado(DESCARGAR_TOLVA);
        break;
      }

      // 3. Desplazamiento suave en reversa hacia las tolvas
      moverOmni(0, -VELOCIDAD_CRUCERO, 0);
      break;

    case DESCARGAR_TOLVA:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        pararMotores(); // Asegura que el chasis esté completamente inmóvil
        
        // Acciona el servomotor de la compuerta trasera para liberar los granos
        ejecutarDescargaTolva(); 
        
        estadoIniciado = true;
      }

      // Espera 3 segundos para asegurar la caída completa por gravedad de los frutos
      if (millis() - tiempoInicioEstado > 3000) {
        pararMotores();
        // Misión finalizada con éxito: Pasa a reposo/espera
        cambiarEstado(ESTADO_ERROR); // O un estado FIN_MISION / ESPERAR_INICIO
      }
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
