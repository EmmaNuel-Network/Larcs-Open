// ==========================================
// PESTAÑA: Infrarrojos.ino
// Modulos TRACKING-8-CH (Lectura por Salida D8)
// ==========================================
#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

extern Adafruit_MCP23X17 mcp;

const int pinIR_Front_Centro = 8;  // PB0
const int pinIR_Front_Izq    = 9;  // PB1
const int pinIR_Front_Der    = 10; // PB2
const int pinIR_Lat_Izq      = 11; // PB3
const int pinIR_Lat_Der      = 12; // PB4

const bool IR_LOGICA_NEGRO_HIGH = true;

void setupInfrarrojos() {
  mcp.pinMode(pinIR_Front_Izq, INPUT_PULLUP);
  mcp.pinMode(pinIR_Front_Centro, INPUT_PULLUP);
  mcp.pinMode(pinIR_Front_Der, INPUT_PULLUP);
  mcp.pinMode(pinIR_Lat_Izq, INPUT_PULLUP);
  mcp.pinMode(pinIR_Lat_Der, INPUT_PULLUP);
}

bool leerEstadoIR(int lecturaRaw) {
  return IR_LOGICA_NEGRO_HIGH ? (lecturaRaw == HIGH) : (lecturaRaw == LOW);
}

bool estaSobreLineaFrontal() {
  bool s1 = leerEstadoIR(mcp.digitalRead(pinIR_Front_Izq));
  bool s2 = leerEstadoIR(mcp.digitalRead(pinIR_Front_Centro));
  bool s3 = leerEstadoIR(mcp.digitalRead(pinIR_Front_Der));
  return (s1 || s2 || s3);
}

bool detectoMargenIzquierdo() {
  return leerEstadoIR(mcp.digitalRead(pinIR_Lat_Izq));
}

bool detectoMargenDerecho() {
  return leerEstadoIR(mcp.digitalRead(pinIR_Lat_Der));
}
