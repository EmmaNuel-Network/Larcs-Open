// ==========================================
// PESTAÑA: Ultrasonidos.ino (2 Triggers Independientes)
// ==========================================
#include <Arduino.h>

// 1. ASIGNACIÓN DE PINES EN ESP32 (2 Sensores Independientes)
const int pinTrigAlto        = 18; // Trigger Sensor Alto
const int pinEchoAlto        = 23; // Echo Sensor Alto

const int pinTrigLateral     = 32; // Trigger Sensor Lateral
const int pinEchoLateral     = 26; // Echo Sensor Lateral

// 2. CONFIGURACIÓN Y CONSTANTES DE DISTANCIA
const int DISTANCIA_DESEADA_PARED = 15; // cm objetivo a la pared

// Temporización secuencial para eliminar ecos fantasma sin bloquear
unsigned long ultimoDisparoUS = 0;
const unsigned long INTERVALO_ENTRE_SENSORES_MS = 35; // 35 ms entre cada disparo

// Caché de distancias para el Orquestador
int distCacheAlto    = 999;
int distCacheLateral = 999;

// Índice de la máquina de estados interna (0 = Alto, 1 = Lateral)
int sensorActualMuestreo = 0; 

// 3. PARÁMETROS DEL CONTROL PD DE PARED
float Kp_Pared = 3.5f;  
float Kd_Pared = 1.2f;  

float errorAnteriorPared = 0.0f;
unsigned long ultimoTiempoUltrasonido = 0;

// 4. INICIALIZACIÓN DE PINES
void setupUltrasonidos() {
  pinMode(pinTrigAlto, OUTPUT);
  pinMode(pinEchoAlto, INPUT);

  pinMode(pinTrigLateral, OUTPUT);
  pinMode(pinEchoLateral, INPUT);

  digitalWrite(pinTrigAlto, LOW);
  digitalWrite(pinTrigLateral, LOW);
}

// 5. LECTURA BASE CON TIMEOUT SEGURO (Max ~2.5 metros)
int medirSensorDirecto(int pinTrig, int pinEcho) {
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);

  long duracion = pulseIn(pinEcho, HIGH, 15000); // 15ms timeout (~2.5 m)

  if (duracion == 0) return 999; 

  int distancia = (int)(duracion * 0.034 / 2);
  if (distancia <= 0) return 999; 

  return distancia;
}

// 6. MUESTREO SECUENCIAL NO BLOQUEANTE (Anti-Ecos Fantasma)
void actualizarLecturasUltrasonido() {
  unsigned long tiempoActual = millis();

  if (tiempoActual - ultimoDisparoUS >= INTERVALO_ENTRE_SENSORES_MS) {
    ultimoDisparoUS = tiempoActual;

    switch (sensorActualMuestreo) {
      case 0:
        distCacheAlto = medirSensorDirecto(pinTrigAlto, pinEchoAlto);
        sensorActualMuestreo = 1;
        break;

      case 1:
        distCacheLateral = medirSensorDirecto(pinTrigLateral, pinEchoLateral);
        sensorActualMuestreo = 0; // Vuelve al primer sensor
        break;
    }
  }
}

// 7. FUNCIONES UTILIZADAS POR LA FSM PRINCIPAL (Retornan caché rápida)
int leerUltrasonidoAlto() {
  actualizarLecturasUltrasonido();
  return distCacheAlto;
}

int leerUltrasonidoLateral() {
  actualizarLecturasUltrasonido();
  return distCacheLateral;
}

// 8. CÁLCULO DE ERROR Y PD PARA NAVEGACIÓN
int calcularErrorPared(int distMedida) {
  if (distMedida >= 999) return 0;
  return distMedida - DISTANCIA_DESEADA_PARED;
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