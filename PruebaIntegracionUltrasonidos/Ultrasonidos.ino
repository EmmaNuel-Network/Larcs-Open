// ==========================================
// PESTAÑA: Ultrasonidos.ino (Adaptada a Pines 23 y 32)
// ==========================================
#include <Arduino.h>

// 1. ASIGNACIÓN DE PINES EN ESP32
const int pinTrigUnificado   = 18; // ÚNICO PIN PARA TODOS LOS TRIGGERS

const int pinEchoFrontalBajo = 19; // Frontal Bajo
const int pinEchoAlto        = 23; // Conectado Físicamente
const int pinEchoLateral     = 32; // Conectado Físicamente

// 2. CONFIGURACIÓN Y CONSTANTES DE DISTANCIA
const int DISTANCIA_DESEADA_PARED = 15; // cm objetivo a la pared

// Temporización de lecturas para no saturar el loop principal
unsigned long ultimoMuestreoUS = 0;
const unsigned long INTERVALO_US_MS = 60; // 60 ms para estabilizar la onda

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
  pinMode(pinTrigUnificado, OUTPUT);
  pinMode(pinEchoFrontalBajo, INPUT);
  pinMode(pinEchoAlto, INPUT);
  pinMode(pinEchoLateral, INPUT);

  digitalWrite(pinTrigUnificado, LOW);
}

// 5. LECTURA BASE CON TIMEOUT (Max ~5 metros / 30000 us)
int medirSensorDirecto(int pinTrig, int pinEcho) {
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);

  long duracion = pulseIn(pinEcho, HIGH, 30000); 

  if (duracion == 0) {
    return 999; // Fuera de rango o sin eco
  }

  return (int)(duracion * 0.034 / 2);
}

// 6. ACTUALIZACIÓN PERIÓDICA DE SENSORES (Muestrea Pines 23 y 32)
void actualizarLecturasUltrasonido() {
  unsigned long tiempoActual = millis();
  if (tiempoActual - ultimoMuestreoUS >= INTERVALO_US_MS) {
    ultimoMuestreoUS = tiempoActual;

    // 1. Leer Sensor Pin 23 (Alto / Frontal Activo)
    distCacheAlto = medirSensorDirecto(pinTrigUnificado, pinEchoAlto);
    
    // PAUSA CRÍTICA: Desvanecer eco antes de la siguiente medición
    delay(40);
    
    // 2. Leer Sensor Pin 32 (Lateral Activo)
    distCacheLateral = medirSensorDirecto(pinTrigUnificado, pinEchoLateral);

    // NOTA: El pin 19 queda en 999 para no congelar el bus
    distCacheFrontalBajo = 999;
  }
}

// 7. FUNCIONES UTILIZADAS POR EL ORQUESTADOR
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