// ==========================================
// PESTAÑA: Infrarrojos.ino (3 QTR-8A: 3 Frontales PD + 2 Laterales Margen)
// ==========================================
#include <Arduino.h>

// 1. ASIGNACIÓN DE PINES EN ESP32 (Pines ADC1)
// Frontales: 3 sensores activos de la regleta central
const int pinIR_Front_Izq    = 34; // Peso -1.0
const int pinIR_Front_Centro = 35; // Peso  0.0
const int pinIR_Front_Der    = 32; // Peso +1.0

// Laterales: 1 sensor activo de cada regleta lateral (detección de margen)
const int pinIR_Lat_Izq      = 33; 
const int pinIR_Lat_Der      = 36; // VP

// Umbral de lectura analógica QTR-8A (ESP32 ADC 0 - 4095)
// Si la lectura supera este valor, significa que está sobre la línea negra
const int UMBRAL_LINEA = 2000; 

// 2. PARÁMETROS DEL CONTROL PD DE LÍNEA
float Kp_Linea = 50.0f;  // Proporcional: Corrección según la desviación
float Kd_Linea = 15.0f;  // Derivativo: Previene oscilaciones bruscas

float errorAnteriorLinea = 0.0f;
unsigned long ultimoTiempoIR = 0;

// 3. INICIALIZACIÓN
void setupInfrarrojos() {
  pinMode(pinIR_Front_Izq, INPUT);
  pinMode(pinIR_Front_Centro, INPUT);
  pinMode(pinIR_Front_Der, INPUT);
  pinMode(pinIR_Lat_Izq, INPUT);
  pinMode(pinIR_Lat_Der, INPUT);
}

// 4. CÁLCULO DE ERROR PONDERADO (3 Sensores Frontales)
// Retorna un valor continuo entre -1.0 y +1.0. Si no hay línea, retorna 999.0f
float calcularErrorLinea() {
  int s1 = (analogRead(pinIR_Front_Izq)    > UMBRAL_LINEA) ? 1 : 0;
  int s2 = (analogRead(pinIR_Front_Centro) > UMBRAL_LINEA) ? 1 : 0;
  int s3 = (analogRead(pinIR_Front_Der)    > UMBRAL_LINEA) ? 1 : 0;

  int sumaLecturas = s1 + s2 + s3;

  // Si ningún sensor frontal detecta la línea
  if (sumaLecturas == 0) {
    return 999.0f; 
  }

  // Promedio ponderado con pesos: Izq (-1), Centro (0), Der (+1)
  float posicionLinea = (float)(-1 * s1 + 0 * s2 + 1 * s3) / (float)sumaLecturas;
  return posicionLinea;
}

// 5. CONTROL PD DE ALINEACIÓN FRONTAL
int obtenerCorreccionLineaPD() {
  float errorActual = calcularErrorLinea();

  // Si se pierde la línea, se mantiene una suave tendencia según el último giro
  if (errorActual >= 999.0f) {
    return (errorAnteriorLinea > 0) ? 40 : -40; 
  }

  unsigned long tiempoActual = millis();
  float dt = (tiempoActual - ultimoTiempoIR) / 1000.0f;
  if (dt <= 0.0f) dt = 0.01f;
  ultimoTiempoIR = tiempoActual;

  // Cálculo del término derivativo
  float derivativa = (errorActual - errorAnteriorLinea) / dt;
  errorAnteriorLinea = errorActual;

  // Ecuación PD
  float salidaPD = (Kp_Linea * errorActual) + (Kd_Linea * derivativa);

  // Limita la corrección máxima enviada a los motores
  return (int)constrain(salidaPD, -120.0f, 120.0f);
}

// 6. DETECCIÓN DE MÁRGENES Y BORDES LATERALES
bool detectoMargenIzquierdo() {
  return (analogRead(pinIR_Lat_Izq) > UMBRAL_LINEA);
}

bool detectoMargenDerecho() {
  return (analogRead(pinIR_Lat_Der) > UMBRAL_LINEA);
}

bool estaSobreLineaFrontal() {
  return (calcularErrorLinea() < 999.0f);
}