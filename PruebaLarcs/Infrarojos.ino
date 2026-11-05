// ==========================================
// PESTAÑA: Infrarojos.ino (2 Frontales + 2 Laterales, todos en el MCP)
// ==========================================
#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

extern Adafruit_MCP23X17 mcp;

// 1. ASIGNACIÓN DE PINES EN MCP23017 (Puerto B: 8 a 15)
// Si los frontales quedan cruzados (pos sale al revés), intercambiar 8 y 9.
const int pinIR_Front_Der = 8;  // PB0
const int pinIR_Front_Izq = 9;  // PB1
const int pinIR_Lat_Izq   = 10; // PB2
const int pinIR_Lat_Der   = 11; // PB3
// PB4 queda libre para el paso a paso.

// Valor que entrega el sensor sobre NEGRO.
// Si en PRUEBA_SENSORES ven 1 sobre blanco y 0 sobre negro, cambiar a LOW.
const int IR_VALOR_NEGRO = HIGH;

// 2. PARÁMETROS DEL CONTROL PD DE LÍNEA
float Kp_Linea = 50.0f;  // Proporcional
float Kd_Linea = 15.0f;  // Derivativo

float errorAnteriorLinea = 0.0f;
unsigned long ultimoTiempoIR = 0;
bool lineaPerdidaPrevia = true; // Previene picos derivativos al recuperar la línea

// Lectura normalizada: 1 = negro, 0 = blanco (sin importar la lógica del módulo)
int leerIR(int pin) {
  return (mcp.digitalRead(pin) == IR_VALOR_NEGRO) ? 1 : 0;
}

// 3. INICIALIZACIÓN
void setupInfrarrojos() {
  mcp.pinMode(pinIR_Front_Der, INPUT_PULLUP);
  mcp.pinMode(pinIR_Front_Izq, INPUT_PULLUP);
  mcp.pinMode(pinIR_Lat_Izq, INPUT_PULLUP);
  mcp.pinMode(pinIR_Lat_Der, INPUT_PULLUP);
}

// 4. POSICIÓN DE LA LÍNEA CON 2 SENSORES FRONTALES
// -1 = línea bajo el izquierdo, 0 = bajo ambos, +1 = bajo el derecho, 999 = sin línea
float calcularErrorLinea() {
  int sIzq = leerIR(pinIR_Front_Izq);
  int sDer = leerIR(pinIR_Front_Der);

  int suma = sIzq + sDer;
  if (suma == 0) return 999.0f;

  return (float)(sDer - sIzq) / (float)suma;
}

// 5. CONTROL PD DE ALINEACIÓN FRONTAL
int obtenerCorreccionLineaPD() {
  float errorActual = calcularErrorLinea();
  unsigned long tiempoActual = millis();

  if (errorActual >= 999.0f) {
    ultimoTiempoIR = tiempoActual;
    lineaPerdidaPrevia = true;
    return (errorAnteriorLinea > 0) ? 40 : -40;
  }

  float dt = (tiempoActual - ultimoTiempoIR) / 1000.0f;
  if (dt <= 0.0f) dt = 0.01f;
  ultimoTiempoIR = tiempoActual;

  if (lineaPerdidaPrevia) {
    errorAnteriorLinea = errorActual;
    lineaPerdidaPrevia = false;
  }

  float derivativa = (errorActual - errorAnteriorLinea) / dt;
  errorAnteriorLinea = errorActual;

  float salidaPD = (Kp_Linea * errorActual) + (Kd_Linea * derivativa);
  return (int)constrain(salidaPD, -120.0f, 120.0f);
}

// 6. DETECCIÓN DE MÁRGENES Y LÍNEA FRONTAL
bool detectoMargenIzquierdo() {
  return leerIR(pinIR_Lat_Izq) == 1;
}

bool detectoMargenDerecho() {
  return leerIR(pinIR_Lat_Der) == 1;
}

bool estaSobreLineaFrontal() {
  return (calcularErrorLinea() < 999.0f);
}

// 7. PARA EL DIAGNÓSTICO (PRUEBA_SENSORES)
int irFrontalIzq() { return leerIR(pinIR_Front_Izq); }
int irFrontalDer() { return leerIR(pinIR_Front_Der); }