
// ==========================================
// PRUEBA DE ESQUIVA INTEGRADA (Ultrasonidos + Infrarrojos)
// ==========================================
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>

Adafruit_MCP23X17 mcp;

struct ControlPD {
  float setpoint = 0.0f;
  long pulsosAnteriores = 0;
  float errorPrev = 0.0f;
  float Kp = 1.2f;
  float Kd = 0.15f;
  int outputPWM = 0;
};

// Declaraciones externas
void setupMotores();
void actualizarControlMotores();
void moverOmni(int Vx, int Vy, int W);
void pararMotores();

void setupUltrasonidos();
void actualizarLecturasUltrasonido();
int leerUltrasonidoAlto();
int leerUltrasonidoLateral();
int obtenerCorreccionParedPD(int distMedida);

void setupInfrarrojos();
bool estaSobreLineaFrontal();
bool detectoLineaFrontalIzq();
bool detectoLineaFrontalDer();
bool detectoMargenIzquierdo();
bool detectoMargenDerecho();

// Máquina de estados con evasión de líneas
enum EstadoPrueba {
  AVANZAR_AL_FRENTE,
  ESQUIVAR_DERECHA_ULTRASONIDO,
  EVADIR_LINEA_DERECHA,   // Reacción a línea frontal / izquierda
  EVADIR_LINEA_IZQUIERDA,  // Reacción a línea derecha
  VERIFICAR_LIBERACION
};

EstadoPrueba estadoPrueba = AVANZAR_AL_FRENTE;
unsigned long tiempoInicioEstado = 0;
unsigned long tiempoDespejado = 0;

const int VELOCIDAD_BASE = 120;

void setup() {
  Serial.begin(115200);
  
  Wire.begin(21, 22);
  if (!mcp.begin_I2C()) {
    Serial.println("Error al iniciar MCP23017");
    while (1);
  }

  setupMotores();
  setupUltrasonidos();
  setupInfrarrojos();
  pararMotores();

  delay(2000);
  Serial.println("--- Inicio de Navegación Integrada (US + IR) ---");
}

void loop() {
  actualizarControlMotores();
  actualizarLecturasUltrasonido();

  int distFrontal = leerUltrasonidoAlto();
  int distLateral = leerUltrasonidoLateral();

  // Distancia efectiva unificada para ultrasónicos
  int distParedEfectiva = 999;
  if (distFrontal < 999 && distLateral < 999) {
    distParedEfectiva = min(distFrontal, distLateral);
  } else if (distFrontal < 999) {
    distParedEfectiva = distFrontal;
  } else if (distLateral < 999) {
    distParedEfectiva = distLateral;
  }

  // Lecturas Infrarrojas
  bool lineaFrontal = estaSobreLineaFrontal();
  bool lineaIzq     = detectoLineaFrontalIzq() || detectoMargenIzquierdo();
  bool lineaDer     = detectoLineaFrontalDer() || detectoMargenDerecho();

  switch (estadoPrueba) {

    case AVANZAR_AL_FRENTE:
      moverOmni(0, VELOCIDAD_BASE, 0);

      // PRIORIDAD 1: Reglas de Infrarrojos
      if (lineaDer) {
        pararMotores();
        tiempoInicioEstado = millis();
        estadoPrueba = EVADIR_LINEA_IZQUIERDA;
        Serial.println("-> Línea Derecha detectada. Evadiendo a la IZQUIERDA...");
      } 
      else if (lineaFrontal || lineaIzq) {
        pararMotores();
        tiempoInicioEstado = millis();
        estadoPrueba = EVADIR_LINEA_DERECHA;
        Serial.println("-> Línea Frontal/Izquierda detectada. Evadiendo a la DERECHA...");
      }
      // PRIORIDAD 2: Obstáculo Ultrasonido
      else if (distFrontal >= 3 && distFrontal < 20) {
        pararMotores();
        tiempoInicioEstado = millis();
        estadoPrueba = ESQUIVAR_DERECHA_ULTRASONIDO;
        Serial.println("-> Obstáculo por Ultrasonido. Esquivando a la derecha...");
      }
      break;

    case EVADIR_LINEA_DERECHA:
      // Movimiento lateral hacia la derecha (+Vx)
      moverOmni(VELOCIDAD_BASE, 0, 0);

      // Sostiene el pulso de evasión 400ms o hasta liberarse de la línea
      if (!lineaFrontal && !lineaIzq && (millis() - tiempoInicioEstado > 400)) {
        pararMotores();
        estadoPrueba = AVANZAR_AL_FRENTE;
        Serial.println("-> Línea evadida. Reanudando avance...");
      }
      break;

    case EVADIR_LINEA_IZQUIERDA:
      // Movimiento lateral hacia la izquierda (-Vx)
      moverOmni(-VELOCIDAD_BASE, 0, 0);

      // Sostiene el pulso de evasión 400ms o hasta liberarse de la línea
      if (!lineaDer && (millis() - tiempoInicioEstado > 400)) {
        pararMotores();
        estadoPrueba = AVANZAR_AL_FRENTE;
        Serial.println("-> Línea evadida. Reanudando avance...");
      }
      break;

    case ESQUIVAR_DERECHA_ULTRASONIDO: {
      int correccionPD = obtenerCorreccionParedPD(distParedEfectiva);
      moverOmni(VELOCIDAD_BASE, correccionPD, 0);

      // Interrupción por línea durante la esquiva por ultrasonido
      if (lineaDer) {
        estadoPrueba = EVADIR_LINEA_IZQUIERDA;
        break;
      }

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
        estadoPrueba = ESQUIVAR_DERECHA_ULTRASONIDO;
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