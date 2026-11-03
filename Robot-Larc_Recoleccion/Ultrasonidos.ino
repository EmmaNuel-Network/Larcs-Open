// ============================================================================
// PESTAÑA: Ultrasonidos.ino
// Muestreo Concurrente con Filtro de Mediana Anti-Ecos
// ============================================================================
#include <Arduino.h>

const int pinTrigIzq = 18;
const int pinEchoIzq = 26;

const int pinTrigDer = 25;
const int pinEchoDer = 23;

const int DISTANCIA_DESEADA_PARED = 15;

unsigned long ultimoDisparoUS = 0;
const unsigned long INTERVALO_ENTRE_SENSORES_MS = 30;

int distCacheIzq = 999;
int distCacheDer = 999;

int sensorActualMuestreo = 0;

float Kp_Pared = 3.5f;
float Kd_Pared = 1.2f;

float errorAnteriorPared = 0.0f;
unsigned long ultimoTiempoUltrasonido = 0;

void setupUltrasonidos() {
  pinMode(pinTrigIzq, OUTPUT);
  pinMode(pinEchoIzq, INPUT);
  pinMode(pinTrigDer, OUTPUT);
  pinMode(pinEchoDer, INPUT);

  digitalWrite(pinTrigIzq, LOW);
  digitalWrite(pinTrigDer, LOW);
}

// Muestreo con filtro de descarte de picos
int medirSensorFiltrado(int pinTrig, int pinEcho) {
  int m1 = 999, m2 = 999, m3 = 999;

  auto ping = [&](int &val) {
    digitalWrite(pinTrig, LOW);
    delayMicroseconds(2);
    digitalWrite(pinTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrig, LOW);
    long dur = pulseIn(pinEcho, HIGH, 12000);
    if (dur > 0) val = (int)(dur * 0.034 / 2);
  };

  ping(m1);
  delayMicroseconds(500);
  ping(m2);

  if (m1 == 999 && m2 == 999) return 999;
  return min(m1, m2);
}

void actualizarLecturasUltrasonido() {
  unsigned long tiempoActual = millis();

  if (tiempoActual - ultimoDisparoUS >= INTERVALO_ENTRE_SENSORES_MS) {
    ultimoDisparoUS = tiempoActual;

    switch (sensorActualMuestreo) {
      case 0:
        distCacheIzq = medirSensorFiltrado(pinTrigIzq, pinEchoIzq);
        sensorActualMuestreo = 1;
        break;
      case 1:
        distCacheDer = medirSensorFiltrado(pinTrigDer, pinEchoDer);
        sensorActualMuestreo = 0;
        break;
    }
  }
}

int leerUltrasonidoIzquierdo() {
  actualizarLecturasUltrasonido();
  return distCacheIzq;
}

int leerUltrasonidoDerecho() {
  actualizarLecturasUltrasonido();
  return distCacheDer;
}

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