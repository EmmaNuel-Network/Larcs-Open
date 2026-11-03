// ============================================================================
// PESTAÑA: Infrarrojos.ino
// Lectura de Sensores QTR-8A conectados al MCP23017 (Puerto B)
// ============================================================================
#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

extern Adafruit_MCP23X17 mcp;

// Mapeo MCP23017 Puerto B
const int pinIR_Centro_1 = 8;  // PB0 (Pin 8)
const int pinIR_Centro_2 = 9;  // PB1 (Pin 9)
const int pinIR_Lat_Izq  = 10; // PB2 (Pin 10)
const int pinIR_Lat_Der  = 11; // PB3 (Pin 11)

float Kp_Linea = 50.0f;
float Kd_Linea = 15.0f;

float errorAnteriorLinea = 0.0f;
unsigned long ultimoTiempoIR = 0;
bool lineaPerdidaPrevia = true;

void setupInfrarrojos() {
  mcp.pinMode(pinIR_Centro_1, INPUT_PULLUP);
  mcp.pinMode(pinIR_Centro_2, INPUT_PULLUP);
  mcp.pinMode(pinIR_Lat_Izq, INPUT_PULLUP);
  mcp.pinMode(pinIR_Lat_Der, INPUT_PULLUP);
}

float calcularErrorLinea() {
  int s1 = (mcp.digitalRead(pinIR_Centro_1) == HIGH) ? 1 : 0;
  int s2 = (mcp.digitalRead(pinIR_Centro_2) == HIGH) ? 1 : 0;

  int sumaLecturas = s1 + s2;
  if (sumaLecturas == 0) return 999.0f;

  return (float)(-1 * s1 + 1 * s2) / (float)sumaLecturas;
}

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

bool detectoMargenIzquierdo() {
  return (mcp.digitalRead(pinIR_Lat_Izq) == HIGH);
}

bool estaSobreLineaFrontal() {
  return (calcularErrorLinea() < 999.0f);
}