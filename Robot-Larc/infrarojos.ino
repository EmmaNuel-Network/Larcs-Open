// ==========================================
// PESTAÑA: SensoresIR.ino
// ==========================================
#include <Arduino.h>

// 1. ASIGNACIÓN DE PINES EN ESP32
// (Asigna aquí los pines digitales libres que conectes a las salidas D1, D2, D3 del módulo)
const int pinIR_Izquierda = 2;   // Sensor lateral izquierdo
const int pinIR_Frente    = 0;   // Sensor frontal
const int pinIR_Derecha   = 12;  // Sensor lateral derecho

// 2. LÓGICA DE DETECCIÓN
// Ajusta a HIGH o LOW según el comportamiento del módulo (si manda 1 o 0 al detectar la línea)
const int NIVEL_LINEA = HIGH; 

// 3. INICIALIZACIÓN DE SENSORES
void setupSensoresIR() {
  pinMode(pinIR_Izquierda, INPUT);
  pinMode(pinIR_Frente,    INPUT);
  pinMode(pinIR_Derecha,   INPUT);
}

// 4. LECTURAS INDIVIDUALES
bool leerSensorPisoIzquierda() {
  return (digitalRead(pinIR_Izquierda) == NIVEL_LINEA);
}

bool leerSensorPisoFrente() {
  return (digitalRead(pinIR_Frente) == NIVEL_LINEA);
}

bool leerSensorPisoDerecha() {
  return (digitalRead(pinIR_Derecha) == NIVEL_LINEA);
}

// 5. FUNCIONES DE INTEGRACIÓN PARA MÁQUINA DE ESTADOS (Robot-Larc.ino)
// Reemplaza el 'weak' de la pestaña principal para el sensor frontal
bool leerSensorPisoFrente() __attribute__((alias("leerSensorPisoFrente"))); 

// Reemplaza el 'weak' para evaluar si cualquiera de los lados toca la línea
bool leerSensorPisoLateral() {
  return leerSensorPisoIzquierda() || leerSensorPisoDerecha();
}
