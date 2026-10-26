// ==========================================
// PESTAÑA: Infrarrojos.ino (3 QTR-8A: 3 Frontales PD + 2 Laterales Margen)
// ==========================================
#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

extern Adafruit_MCP23X17 mcp;

// 1. ASIGNACIÓN DE PINES EN MCP23017
const int pinIR_Front_Centro = 8;  // PB0 (D4)
const int pinIR_Front_Izq    = 9;  // PB1 (D2)
const int pinIR_Front_Der    = 10; // PB2 (D3)

const int pinIR_Lat_Izq      = 11; // PB3 (D1 Izq)
const int pinIR_Lat_Der      = 0;  // PA0 / GPIO0 (D1 Der)

// 2. PARÁMETROS DEL CONTROL PD DE LÍNEA
float Kp_Linea = 50.0f;  // Proporcional
float Kd_Linea = 15.0f;  // Derivativo

float errorAnteriorLinea = 0.0f;
unsigned long ultimoTiempoIR = 0;
bool lineaPerdidaPrevia = true; // Previene picos derivativos al recuperar la línea

// 3. INICIALIZACIÓN
void setupInfrarrojos() {
  mcp.pinMode(pinIR_Front_Centro, INPUT_PULLUP);
  mcp.pinMode(pinIR_Front_Izq, INPUT_PULLUP);
  mcp.pinMode(pinIR_Front_Der, INPUT_PULLUP);
  mcp.pinMode(pinIR_Lat_Izq, INPUT_PULLUP);
  mcp.pinMode(pinIR_Lat_Der, INPUT_PULLUP);
}

// 4. FUNCIONES DE LECTURA INDIVIDUAL (Añadidas para corregir el error de compilación)
bool detectoLineaFrontalCentro() {
  return (mcp.digitalRead(pinIR_Front_Centro) == HIGH);
}

bool detectoLineaFrontalIzq() {
  return (mcp.digitalRead(pinIR_Front_Izq) == HIGH);
}

bool detectoLineaFrontalDer() {
  return (mcp.digitalRead(pinIR_Front_Der) == HIGH);
}

// 5. CÁLCULO DE ERROR PONDERADO (3 Sensores Frontales)
float calcularErrorLinea() {
  int s1 = mcp.digitalRead(pinIR_Front_Izq);
  int s2 = mcp.digitalRead(pinIR_Front_Centro);
  int s3 = mcp.digitalRead(pinIR_Front_Der);

  int sumaLecturas = s1 + s2 + s3;

  if (sumaLecturas == 0) {
    return 999.0f; 
  }

  float posicionLinea = (float)(-1 * s1 + 0 * s2 + 1 * s3) / (float)sumaLecturas;
  return posicionLinea;
}

// 6. CONTROL PD DE ALINEACIÓN FRONTAL
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

// 7. DETECCIÓN DE MÁRGENES Y BORDES LATERALES
bool detectoMargenIzquierdo() {
  return (mcp.digitalRead(pinIR_Lat_Izq) == HIGH);
}

bool detectoMargenDerecho() {
  return (mcp.digitalRead(pinIR_Lat_Der) == HIGH);
}

bool estaSobreLineaFrontal() {
  return (calcularErrorLinea() < 999.0f);
}