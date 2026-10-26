// ==========================================
// PRUEBA DE ESQUIVA DE PISCINAS CON PD (Corregido)
// ==========================================
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>

Adafruit_MCP23X17 mcp;

// Definición local de struct ControlPD
struct ControlPD {
  float setpoint = 0.0f;
  long pulsosAnteriores = 0;
  float errorPrev = 0.0f;
  float Kp = 1.2f;
  float Kd = 0.15f;
  int outputPWM = 0;
};

// Declaración de funciones externas de tus módulos
void setupMotores();
void actualizarControlMotores();
void moverOmni(int Vx, int Vy, int W);
void pararMotores();

void setupUltrasonidos();
void actualizarLecturasUltrasonido();
int leerUltrasonidoAlto();
int leerUltrasonidoLateral();
int obtenerCorreccionParedPD(int distMedida);

// Estados de la máquina reducida
enum EstadoPrueba {
  AVANZAR_AL_FRENTE,
  ESQUIVAR_DERECHA,
  VERIFICAR_LIBERACION
};

EstadoPrueba estadoPrueba = AVANZAR_AL_FRENTE;
unsigned long tiempoInicioEstado = 0;
unsigned long tiempoDespejado = 0;

const int VELOCIDAD_BASE = 120; // Velocidad moderada y precisa

void setup() {
  Serial.begin(115200);
  
  Wire.begin(21, 22); // SDA = 21, SCL = 22
  if (!mcp.begin_I2C()) {
    Serial.println("Error al iniciar MCP23017");
    while (1);
  }

  setupMotores();
  setupUltrasonidos();
  pararMotores();

  delay(2000); // Pausa de seguridad antes de arrancar
  Serial.println("--- Inicio de Prueba de Esquiva de Piscinas ---");
}

void loop() {
  actualizarControlMotores();
  actualizarLecturasUltrasonido();

  int distFrontal = leerUltrasonidoAlto();
  int distLateral = leerUltrasonidoLateral();

  // UNIFICACIÓN: Toma la menor distancia válida entre ambos sensores
  // Si ambos ven pared, usará la más cercana. Si solo uno la ve, usará esa.
  int distParedEfectiva = 999;
  if (distFrontal < 999 && distLateral < 999) {
    distParedEfectiva = min(distFrontal, distLateral);
  } else if (distFrontal < 999) {
    distParedEfectiva = distFrontal;
  } else if (distLateral < 999) {
    distParedEfectiva = distLateral;
  }

  switch (estadoPrueba) {

    case AVANZAR_AL_FRENTE:
      moverOmni(0, VELOCIDAD_BASE, 0);

      // Si cualquiera de los dos detecta obstáculo al frente/cerca
      if (distFrontal >= 3 && distFrontal < 20) {
        pararMotores();
        tiempoInicioEstado = millis();
        estadoPrueba = ESQUIVAR_DERECHA;
        Serial.println("-> Obstáculo detectado. Iniciando esquiva...");
      }
      break;

    case ESQUIVAR_DERECHA: {
      // Usa la distancia unificada (sea del frontal, lateral o ambos)
      int correccionPD = obtenerCorreccionParedPD(distParedEfectiva);

      // Desplazamiento lateral con corrección homogénea
      moverOmni(VELOCIDAD_BASE, correccionPD, 0);

      // Salida del estado solo si AMBOS sensores ven libre el frente
      if (distFrontal > 35 && distFrontal < 300) {
        tiempoDespejado = millis();
        estadoPrueba = VERIFICAR_LIBERACION;
        Serial.println("-> Frente despejado. Verificando liberación...");
      }
      break;
    }

    case VERIFICAR_LIBERACION: {
      int correccionPD = obtenerCorreccionParedPD(distParedEfectiva);
      moverOmni(VELOCIDAD_BASE, correccionPD, 0);

      if (distFrontal >= 3 && distFrontal <= 35) {
        estadoPrueba = ESQUIVAR_DERECHA;
        break;
      }

      if (millis() - tiempoDespejado > 800) {
        pararMotores();
        estadoPrueba = AVANZAR_AL_FRENTE;
        Serial.println("-> Obstáculo superado. Reanudando avance...");
      }
      break;
    }
  }
}