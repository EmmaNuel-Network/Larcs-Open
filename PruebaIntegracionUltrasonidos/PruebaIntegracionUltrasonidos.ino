// ============================================================================
// ORQUESTADOR DE PRUEBA: 6 MOVIMIENTOS HOLONÓMICOS
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>

Adafruit_MCP23X17 mcp;

// Declaración del struct para evitar el error de ámbito de compilación
struct ControlPD {
  float setpoint = 0.0f;
  long pulsosAnteriores = 0;
  float errorPrev = 0.0f;
  float Kp = 1.2f;
  float Kd = 0.15f;
  int outputPWM = 0;
};

// Declaración de funciones de Motores.ino
void setupMotores();
void actualizarControlMotores();
void moverOmni(int Vx, int Vy, int W);
void pararMotores();

// Variables temporales de la secuencia
unsigned long ultimoCambioEtapa = 0;
int etapaActual = -1; 
const unsigned long TIEMPO_MOVIMIENTO = 2500; // 2.5 segundos de movimiento
const unsigned long TIEMPO_PAUSA      = 1000; // 1.0 segundo de pausa

bool enPausa = true;

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);
  if (!mcp.begin_I2C()) {
    Serial.println("¡Error al conectar con el MCP23017!");
    while (1);
  }

  setupMotores();
  pararMotores();

  Serial.println("--- ESTABILIZANDO PROCESADOR (3 SEGUNDOS) ---");
  delay(3000); // Pausa de arranque seguro

  Serial.println("--- INICIANDO SECUENCIA DE 6 MOVIMIENTOS OMNI ---");
  ultimoCambioEtapa = millis();
}

void loop() {
  // 1. Ejecución continua del control PD a 50Hz (Motores.ino)
  actualizarControlMotores();

  // 2. Máquina de tiempo para iterar los 6 movimientos
  unsigned long tiempoActual = millis();

  if (enPausa) {
    if (tiempoActual - ultimoCambioEtapa >= TIEMPO_PAUSA) {
      enPausa = false;
      etapaActual = (etapaActual + 1) % 6;
      ultimoCambioEtapa = tiempoActual;

      switch (etapaActual) {
        case 0:
          Serial.println("\n1. AVANZAR (+Vy): Todas las ruedas hacia adelante");
          moverOmni(0, 150, 0);
          break;

        case 1:
          Serial.println("\n2. RETROCEDER (-Vy): Todas las ruedas hacia atrás");
          moverOmni(0, -150, 0);
          break;

        case 2:
          Serial.println("\n3. TRASLACIÓN DERECHA (+Vx): Traslación lateral limpia");
          moverOmni(150, 0, 0);
          break;

        case 3:
          Serial.println("\n4. TRASLACIÓN IZQUIERDA (-Vx): Traslación lateral limpia");
          moverOmni(-150, 0, 0);
          break;

        case 4:
          Serial.println("\n5. ROTACIÓN CLOCKWISE (+W): Giro horario sobre su eje");
          moverOmni(0, 0, 100);
          break;

        case 5:
          Serial.println("\n6. ROTACIÓN COUNTER-CLOCKWISE (-W): Giro antihorario");
          moverOmni(0, 0, -100);
          break;
      }
    }
  } else {
    if (tiempoActual - ultimoCambioEtapa >= TIEMPO_MOVIMIENTO) {
      pararMotores();
      enPausa = true;
      ultimoCambioEtapa = tiempoActual;
      Serial.println("-> [PAUSA]");
    }
  }
}