// ==========================================
// PESTAÑA: Elevador.ino
// Motor paso a paso 28BYJ-48 en el MCP23017 (PB4 a PB7), medio paso.
// Basado en el driver Fast-I2C del equipo: escribe el puerto B completo
// en una sola transacción. PB0-PB3 son entradas (infrarrojos) y no se
// ven afectadas por escribir el registro de salida.
// ==========================================
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>

extern Adafruit_MCP23X17 mcp;
void actualizarControlMotores();

const uint8_t MCP_I2C_ADDR_ELEV = 0x20;
const uint8_t REG_GPIOB_ELEV    = 0x13;

// Secuencia de medio paso sobre PB4..PB7
const byte FASES_MEDIO_PASO[8] = {
  0b00010000, 0b00110000, 0b00100000, 0b01100000,
  0b01000000, 0b11000000, 0b10000000, 0b10010000
};

const float PASOS_POR_MM          = 30.0f;  // Medido por el equipo
unsigned long retardoPasoUs       = 1800;   // Probar menor (1200-1400) si el motor aguanta
const float ALTURA_MAX_MM         = 320.0f; // Límite de seguridad del recorrido

int  indiceFase       = 0;
long posicionPasos    = 0;      // 0 = elevador abajo (nivel 1) al encender

void escribirFasesElevador(byte valor) {
  Wire.beginTransmission(MCP_I2C_ADDR_ELEV);
  Wire.write(REG_GPIOB_ELEV);
  Wire.write(valor);
  Wire.endTransmission();
}

void apagarBobinasElevador() {
  escribirFasesElevador(0x00);
}

void setupElevador() {
  mcp.pinMode(12, OUTPUT); mcp.pinMode(13, OUTPUT);   // PB4, PB5
  mcp.pinMode(14, OUTPUT); mcp.pinMode(15, OUTPUT);   // PB6, PB7
  apagarBobinasElevador();
}

float posicionElevadorMM() {
  return posicionPasos / PASOS_POR_MM;
}

void fijarCeroElevador() {
  posicionPasos = 0;
}

// Mueve el elevador a una altura absoluta (mm sobre el nivel 1).
// Es bloqueante: el robot debe estar detenido. Sigue atendiendo el
// control de los motores para que permanezcan frenados.
void elevadorIrAMM(float objetivoMM) {
  objetivoMM = constrain(objetivoMM, 0.0f, ALTURA_MAX_MM);
  long objetivoPasos = lroundf(objetivoMM * PASOS_POR_MM);
  int sentido = (objetivoPasos > posicionPasos) ? 1 : -1;

  unsigned long ultimoControl = millis();
  while (posicionPasos != objetivoPasos) {
    indiceFase += sentido;
    if (indiceFase > 7) indiceFase = 0;
    if (indiceFase < 0) indiceFase = 7;
    escribirFasesElevador(FASES_MEDIO_PASO[indiceFase]);
    posicionPasos += sentido;
    delayMicroseconds(retardoPasoUs);

    if (millis() - ultimoControl >= 20) {
      actualizarControlMotores();
      ultimoControl = millis();
    }
  }
  apagarBobinasElevador();
}