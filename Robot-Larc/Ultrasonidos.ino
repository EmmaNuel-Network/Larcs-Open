// ==========================================
// PESTAÑA: Ultrasonidos.ino (Optimizada con Muestreo No Bloqueante)
// ==========================================
#include <Arduino.h>

// 1. ASIGNACIÓN DE PINES EN ESP32 (Según tabla e instrucción)
const int pinTrigUnificado   = 18; // ÚNICO PIN PARA TODOS LOS TRIGGERS

// Nombres adaptados a la lógica de tu FSM
const int pinEchoFrontalBajo = 19; // Frontal (Echo)
const int pinEchoAlto        = 23; // Izquierdo (Usado como alto)
const int pinEchoLateral     = 32; // Derecho (Usado como lateral)

// 2. CONFIGURACIÓN Y CONSTANTES DE DISTANCIA
const int DISTANCIA_DESEADA_PARED = 15; // cm objetivo a la pared

// Temporización de lecturas para no saturar el loop principal
unsigned long ultimoMuestreoUS = 0;
const unsigned long INTERVALO_US_MS = 40; // Muestreo cada 40 ms (25 Hz)

// Caché de distancias
int distCacheFrontalBajo = 999;
int distCacheAlto        = 999;
int distCacheLateral     = 999;

// 3. PARÁMETROS DEL CONTROL PD
float Kp_Pared = 3.5f;  
float Kd_Pared = 1.2f;  

float errorAnteriorPared = 0.0f;
unsigned long ultimoTiempoUltrasonido = 0;

// 4. INICIALIZACIÓN DE PINES
void setupUltrasonidos() {
  pinMode(pinTrigUnificado, OUTPUT); // Solo declaramos el Trigger una vez
  
  pinMode(pinEchoFrontalBajo, INPUT);
  pinMode(pinEchoAlto, INPUT);
  pinMode(pinEchoLateral, INPUT);
}

// 5. LECTURA BASE CON TIMEOUT OPTIMIZADO (Max ~60 cm / 3500 us)
int medirSensorDirecto(int pinTrig, int pinEcho) {
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);

  // Timeout de 3500 us max (~60 cm). Evita bloqueos largos si no hay objeto.
  long duracion = pulseIn(pinEcho, HIGH, 3500); 

  if (duracion == 0) {
    return 999; // Fuera de rango o sin eco
  }

  return (int)(duracion * 0.034 / 2);
}

// 6. ACTUALIZACIÓN PERIÓDICA DE SENSORES (Llamar desde las funciones de lectura)
void actualizarLecturasUltrasonido() {
  unsigned long tiempoActual = millis();
  if (tiempoActual - ultimoMuestreoUS >= INTERVALO_US_MS) {
    ultimoMuestreoUS = tiempoActual;

    distCacheFrontalBajo = medirSensorDirecto(pinTrigUnificado, pinEchoFrontalBajo);
    distCacheAlto        = medirSensorDirecto(pinTrigUnificado, pinEchoAlto);
    distCacheLateral     = medirSensorDirecto(pinTrigUnificado, pinEchoLateral);
  }
}

// 7. FUNCIONES UTILIZADAS POR EL ORQUESTADOR (Retornan caché rápida)
int leerUltrasonidoFrontalBajo() {
  actualizarLecturasUltrasonido();
  return distCacheFrontalBajo;
}

int leerUltrasonidoAlto() {
  actualizarLecturasUltrasonido();
  return distCacheAlto;
}

int leerUltrasonidoLateral() {
  actualizarLecturasUltrasonido();
  return distCacheLateral;
}

// 8. CÁLCULO DE ERROR
int calcularErrorPared(int distMedida) {
  if (distMedida >= 999) {
    return 0; // Sin pared detectable
  }
  return distMedida - DISTANCIA_DESEADA_PARED;
}

// 9. CONTROL PD DE NAVEGACIÓN POR PARED
int obtenerCorreccionParedPD(int distMedida) {
  if (distMedida >= 999) {
    errorAnteriorPared = 0.0f;
    return 0;
  }

  unsigned long tiempoActual = millis();
  float dt = (tiempoActual - ultimoTiempoUltrasonido) / 1000.0f;
  
  if (dt <= 0.0f) dt = 0.01f; 
  ultimoTiempoUltrasonido = tiempoActual;

  float errorActual = (float)(distMedida - DISTANCIA_DESEADA_PARED);
  float derivativa = (errorActual - errorAnteriorPared) / dt;
  errorAnteriorPared = errorActual;

  float salidaPD = (Kp_Pared * errorActual) + (Kd_Pared * derivativa);

  return (int)constrain(salidaPD, -80.0f, 80.0f);
}