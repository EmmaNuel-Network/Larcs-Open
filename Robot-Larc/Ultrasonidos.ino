// ==========================================
// PESTAÑA: Ultrasonidos.ino (Con Pines, Setup y Control PD)
// ==========================================
#include <Arduino.h>

// 1. ASIGNACIÓN DE PINES EN ESP32
const int pinTrigFrontalBajo = 12;
const int pinEchoFrontalBajo = 13;

const int pinTrigAlto        = 2;
const int pinEchoAlto        = 15;

const int pinTrigLateral     = 0;  // Opcional para seguimiento de pared
const int pinEchoLateral     = 4;  

// 2. CONFIGURACIÓN Y CONSTANTES DE DISTANCIA
const int DISTANCIA_DESEADA_PARED = 15; // cm objetivo a la pared

// 3. PARÁMETROS DEL CONTROL PD
float Kp_Pared = 3.5f;  // Proporcional: Reacción a la desviación en cm
float Kd_Pared = 1.2f;  // Derivativo: Amortigua cambios bruscos

float errorAnteriorPared = 0.0f;
unsigned long ultimoTiempoUltrasonido = 0;

// 4. INICIALIZACIÓN DE PINES
void setupUltrasonidos() {
  pinMode(pinTrigFrontalBajo, OUTPUT);
  pinMode(pinEchoFrontalBajo, INPUT);

  pinMode(pinTrigAlto, OUTPUT);
  pinMode(pinEchoAlto, INPUT);

  pinMode(pinTrigLateral, OUTPUT);
  pinMode(pinEchoLateral, INPUT);
}

// 5. LECTURA BASE DE CUALQUIER SENSOR ULTRASONIDO
int leerDistancia(int pinTrig, int pinEcho) {
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);

  // Timeout de 10000 us (10 ms -> max ~1.7 metros) para evitar bloqueos
  long duracion = pulseIn(pinEcho, HIGH, 10000); 

  if (duracion == 0) {
    return 999; // Sin lectura válida o fuera de rango
  }

  return (int)(duracion * 0.034 / 2);
}

// 6. FUNCIONES ESPECÍFICAS UTILIZADAS POR EL ORQUESTADOR (Robot-Larc.ino)
int leerUltrasonidoFrontalBajo() {
  return leerDistancia(pinTrigFrontalBajo, pinEchoFrontalBajo);
}

int leerUltrasonidoAlto() {
  return leerDistancia(pinTrigAlto, pinEchoAlto);
}

int leerUltrasonidoLateral() {
  return leerDistancia(pinTrigLateral, pinEchoLateral);
}

// 7. CÁLCULO DE ERROR
int calcularErrorPared(int distMedida) {
  if (distMedida >= 999) {
    return 0; // Sin pared detectable
  }
  return distMedida - DISTANCIA_DESEADA_PARED;
}

// 8. CONTROL PD DE NAVEGACIÓN POR PARED
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