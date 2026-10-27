// ==========================================
// PESTAÑA: Ultrasonidos.ino (Secuencial No Bloqueante)
// ==========================================
#include <Arduino.h>

// 1. ASIGNACIÓN DE PINES EN ESP32
const int pinTrigUnificado   = 18; // Trigger compartido

const int pinEchoFrontalBajo = 19; // Sensor Frontal Bajo
const int pinEchoAlto        = 23; // Sensor Alto (Detección de frutos)
const int pinEchoLateral     = 32; // Sensor Lateral (Bordes/Paredes)

// 2. CONFIGURACIÓN Y CONSTANTES DE DISTANCIA
const int DISTANCIA_DESEADA_PARED = 15; // cm objetivo a la pared

// Temporización secuencial para eliminar ecos fantasma sin bloquear
unsigned long ultimoDisparoUS = 0;
const unsigned long INTERVALO_ENTRE_SENSORES_MS = 35; // 35 ms de "descanso" entre cada disparador

// Caché de distancias para el Orquestador
int distCacheFrontalBajo = 999;
int distCacheAlto        = 999;
int distCacheLateral     = 999;

// Índice de la máquina de estados interna
int sensorActualMuestreo = 0; 

// 3. PARÁMETROS DEL CONTROL PD DE PARED
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

// 5. LECTURA BASE CON TIMEOUT SEGURO (Max ~80 cm / 4500 us)
int medirSensorDirecto(int pinTrig, int pinEcho) {
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);

  // Timeout ajustado a 15000 us (~2.5 metros máximo)
  long duracion = pulseIn(pinEcho, HIGH, 15000); 

  // Si no hay respuesta o la lectura es 0, asigna fuera de rango (999)
  if (duracion == 0) {
    return 999; 
  }

  int distancia = (int)(duracion * 0.034 / 2);
  
  // Evita falsos positivos por lecturas en 0
  if (distancia <= 0) return 999; 

  return distancia;
}

// 6. MUESTREO SECUENCIAL NO BLOQUEANTE (Anti-Ecos Fantasma)
void actualizarLecturasUltrasonido() {
  unsigned long tiempoActual = millis();

  // Cambia de sensor cada 35 ms sin usar delay()
  if (tiempoActual - ultimoDisparoUS >= INTERVALO_ENTRE_SENSORES_MS) {
    ultimoDisparoUS = tiempoActual;

    switch (sensorActualMuestreo) {
      case 0:
        distCacheFrontalBajo = medirSensorDirecto(pinTrigUnificado, pinEchoFrontalBajo);
        sensorActualMuestreo = 1;
        break;

      case 1:
        distCacheAlto = medirSensorDirecto(pinTrigUnificado, pinEchoAlto);
        sensorActualMuestreo = 2;
        break;

      case 2:
        distCacheLateral = medirSensorDirecto(pinTrigUnificado, pinEchoLateral);
        sensorActualMuestreo = 0; // Reinicia el ciclo
        break;
    }
  }
}

// 7. FUNCIONES UTILIZADAS POR LA FSM PRINCIPAL (Retornan caché rápida)
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