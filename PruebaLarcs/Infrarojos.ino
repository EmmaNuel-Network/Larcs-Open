// ==========================================
// PESTAÑA: Infrarojos.ino (3 Sensores Activos: 2 Frontales + 1 Lateral Izq en MCP)
// ==========================================
#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

extern Adafruit_MCP23X17 mcp;

// 1. ASIGNACIÓN DE PINES EN MCP23017 (Puerto B: 8 a 15)
const int pinIR_Front_Der = 8;  // PB0 (Activo)
const int pinIR_Front_Izq = 9;  // PB1 (Activo)
const int pinIR_Lat_Izq   = 10; // PB2 (Activo)
const int pinIR_Lat_Der   = 11; // PB3 (Inactivo físicamente)

const bool USAR_IR_LAT_DER = false; // Solo tenemos 3 sensores activos (2 frontales + 1 lat izq)

// Autocalibración individual de cada sensor al encender sobre piso BLANCO.
const bool AUTO_CALIBRAR_IR = true;
const int IR_VALOR_NEGRO_DEFAULT = HIGH;

int valorNegro_Front_Der = IR_VALOR_NEGRO_DEFAULT;
int valorNegro_Front_Izq = IR_VALOR_NEGRO_DEFAULT;
int valorNegro_Lat_Izq   = IR_VALOR_NEGRO_DEFAULT;
int valorNegro_Lat_Der   = IR_VALOR_NEGRO_DEFAULT;

// Filtro anti-vibración por tiempo continuo (en ms):
const unsigned long TIEMPO_FILTRO_IR_FRONT_MS = 25;
const unsigned long TIEMPO_FILTRO_IR_LAT_MS   = 25;

// Ventana de coincidencia entre los 2 sensores frontales (en ms):
const unsigned long VENTANA_COINCIDENCIA_FRONTAL_MS = 450;

struct FiltroTiempoIR {
  bool enNegroCrudo = false;
  unsigned long inicioNegroMs = 0;
  unsigned long ultimoConfirmadoMs = 0;
  bool confirmado = false;
};

FiltroTiempoIR filtroFrontDer, filtroFrontIzq, filtroLatIzq, filtroLatDer;

// 2. PARÁMETROS DEL CONTROL PD DE LÍNEA
float Kp_Linea = 50.0f;  // Proporcional
float Kd_Linea = 15.0f;  // Derivativo

float errorAnteriorLinea = 0.0f;
unsigned long ultimoTiempoIR = 0;
bool lineaPerdidaPrevia = true;

int calibrarPinIRSobreBlanco(int pin, const char* nombre) {
  int conteoHigh = 0;
  int cambios = 0;
  int prev = -1;
  const int MUESTRAS = 40;
  for (int i = 0; i < MUESTRAS; i++) {
    int v = (mcp.digitalRead(pin) == HIGH) ? 1 : 0;
    if (v == 1) conteoHigh++;
    if (prev != -1 && v != prev) cambios++;
    prev = v;
    delay(4);
  }
  int valorBlanco = (conteoHigh >= (MUESTRAS / 2)) ? HIGH : LOW;
  int valorNegro  = (valorBlanco == HIGH) ? LOW : HIGH;
  Serial.printf("[IR CALIB] %-10s (MCP pin %2d): blanco=%d -> NEGRO=%d (estabilidad: %d/%d, cambios=%d)%s\n",
                nombre, pin, valorBlanco, valorNegro,
                (valorBlanco == HIGH) ? conteoHigh : (MUESTRAS - conteoHigh),
                MUESTRAS, cambios,
                (cambios > 2) ? "  [ALERTA: sensor al borde del umbral, ajustar potenciómetro]" : "");
  return valorNegro;
}

// Lectura cruda normalizada: 1 = negro, 0 = blanco
int leerIRCrudo(int pin) {
  if (pin == pinIR_Lat_Der && !USAR_IR_LAT_DER) return 0;
  int valCrudo = mcp.digitalRead(pin);
  int valNegro = IR_VALOR_NEGRO_DEFAULT;
  if (pin == pinIR_Front_Der)      valNegro = valorNegro_Front_Der;
  else if (pin == pinIR_Front_Izq) valNegro = valorNegro_Front_Izq;
  else if (pin == pinIR_Lat_Izq)   valNegro = valorNegro_Lat_Izq;
  else if (pin == pinIR_Lat_Der)   valNegro = valorNegro_Lat_Der;
  return (valCrudo == valNegro) ? 1 : 0;
}

bool actualizarFiltroTiempo(FiltroTiempoIR &f, bool crudoNegro, unsigned long tiempoMinMs) {
  unsigned long ahora = millis();
  if (!crudoNegro) {
    f.enNegroCrudo = false;
    f.confirmado = false;
    return false;
  }
  if (!f.enNegroCrudo) {
    f.enNegroCrudo = true;
    f.inicioNegroMs = ahora;
  }
  if (ahora - f.inicioNegroMs >= tiempoMinMs) {
    f.confirmado = true;
    f.ultimoConfirmadoMs = ahora;
  }
  return f.confirmado;
}

// Lectura filtrada contra vibraciones: 1 = negro confirmado, 0 = blanco
int leerIR(int pin) {
  bool crudo = (leerIRCrudo(pin) == 1);
  if (pin == pinIR_Front_Der) return actualizarFiltroTiempo(filtroFrontDer, crudo, TIEMPO_FILTRO_IR_FRONT_MS) ? 1 : 0;
  if (pin == pinIR_Front_Izq) return actualizarFiltroTiempo(filtroFrontIzq, crudo, TIEMPO_FILTRO_IR_FRONT_MS) ? 1 : 0;
  if (pin == pinIR_Lat_Izq)   return actualizarFiltroTiempo(filtroLatIzq,   crudo, TIEMPO_FILTRO_IR_LAT_MS)   ? 1 : 0;
  if (pin == pinIR_Lat_Der) {
    if (!USAR_IR_LAT_DER) return 0;
    return actualizarFiltroTiempo(filtroLatDer, crudo, TIEMPO_FILTRO_IR_LAT_MS) ? 1 : 0;
  }
  return crudo ? 1 : 0;
}

// 3. INICIALIZACIÓN Y AUTOCALIBRACIÓN
void setupInfrarrojos() {
  mcp.pinMode(pinIR_Front_Der, INPUT_PULLUP);
  mcp.pinMode(pinIR_Front_Izq, INPUT_PULLUP);
  mcp.pinMode(pinIR_Lat_Izq, INPUT_PULLUP);
  mcp.pinMode(pinIR_Lat_Der, INPUT_PULLUP);
  delay(30);

  if (AUTO_CALIBRAR_IR) {
    Serial.println("--- Autocalibrando 3 sensores IR activos (robot sobre BLANCO) ---");
    valorNegro_Front_Der = calibrarPinIRSobreBlanco(pinIR_Front_Der, "Front_Der");
    valorNegro_Front_Izq = calibrarPinIRSobreBlanco(pinIR_Front_Izq, "Front_Izq");
    valorNegro_Lat_Izq   = calibrarPinIRSobreBlanco(pinIR_Lat_Izq,   "Lat_Izq");
    if (USAR_IR_LAT_DER) {
      valorNegro_Lat_Der = calibrarPinIRSobreBlanco(pinIR_Lat_Der,   "Lat_Der");
    }
  }
}

// 4. POSICIÓN DE LA LÍNEA CON 2 SENSORES FRONTALES
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
  if (!USAR_IR_LAT_DER) return false;
  return leerIR(pinIR_Lat_Der) == 1;
}

bool estaSobreLineaFrontal() {
  int izq = leerIR(pinIR_Front_Izq);
  int der = leerIR(pinIR_Front_Der);
  unsigned long ahora = millis();

  if (izq && der) return true;

  bool izqReciente = (filtroFrontIzq.ultimoConfirmadoMs > 0) &&
                     (ahora - filtroFrontIzq.ultimoConfirmadoMs <= VENTANA_COINCIDENCIA_FRONTAL_MS);
  bool derReciente = (filtroFrontDer.ultimoConfirmadoMs > 0) &&
                     (ahora - filtroFrontDer.ultimoConfirmadoMs <= VENTANA_COINCIDENCIA_FRONTAL_MS);

  return (izqReciente && derReciente);
}

long diferenciaTiempoFrontalesMs() {
  return (long)(filtroFrontDer.ultimoConfirmadoMs - filtroFrontIzq.ultimoConfirmadoMs);
}

void limpiarHistorialLineaFrontal() {
  filtroFrontIzq.enNegroCrudo = false;
  filtroFrontIzq.confirmado = false;
  filtroFrontIzq.ultimoConfirmadoMs = 0;
  filtroFrontDer.enNegroCrudo = false;
  filtroFrontDer.confirmado = false;
  filtroFrontDer.ultimoConfirmadoMs = 0;
}

// 7. PARA EL DIAGNÓSTICO (PRUEBA_SENSORES)
int irFrontalIzq() { return leerIR(pinIR_Front_Izq); }
int irFrontalDer() { return leerIR(pinIR_Front_Der); }
