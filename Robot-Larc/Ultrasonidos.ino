// ==========================================
// PESTAÑA: Ultrasonidos.ino
// Muestreo Secuencial Anti-Ecos Fantasma
// ==========================================
#include <Arduino.h>

const int pinTrigUnificado   = 18;
const int pinEchoFrontalBajo = 19;
const int pinEchoAlto        = 23;
const int pinEchoLateral     = 32;

unsigned long ultimoDisparoUS = 0;
int distCacheFrontalBajo = 999;
int distCacheAlto        = 999;
int distCacheLateral     = 999;
int sensorActualMuestreo = 0;

void setupUltrasonidos() {
  pinMode(pinTrigUnificado, OUTPUT);
  pinMode(pinEchoFrontalBajo, INPUT);
  pinMode(pinEchoAlto, INPUT);
  pinMode(pinEchoLateral, INPUT);
  digitalWrite(pinTrigUnificado, LOW);
}

int medirSensorDirecto(int pinEcho) {
  digitalWrite(pinTrigUnificado, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrigUnificado, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrigUnificado, LOW);

  long duracion = pulseIn(pinEcho, HIGH, 4000);
  if (duracion == 0) return 999;

  int distancia = (int)(duracion * 0.034 / 2);
  return (distancia <= 0) ? 999 : distancia;
}

void actualizarLecturasUltrasonido() {
  unsigned long tiempoActual = millis();
  if (tiempoActual - ultimoDisparoUS >= 35) {
    ultimoDisparoUS = tiempoActual;

    switch (sensorActualMuestreo) {
      case 0:
        distCacheFrontalBajo = medirSensorDirecto(pinEchoFrontalBajo);
        sensorActualMuestreo = 1;
        break;
      case 1:
        distCacheAlto = medirSensorDirecto(pinEchoAlto);
        sensorActualMuestreo = 2;
        break;
      case 2:
        distCacheLateral = medirSensorDirecto(pinEchoLateral);
        sensorActualMuestreo = 0;
        break;
    }
  }
}

int obtenerDistanciaMinimaFrontal() {
  actualizarLecturasUltrasonido();
  return min(distCacheFrontalBajo, distCacheAlto);
}
