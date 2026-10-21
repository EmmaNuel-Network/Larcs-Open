// ==========================================
// PRUEBA DE ESQUIVA DE PISCINAS CON PD
// ==========================================
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>


Adafruit_MCP23X17 mcp;

// Definición local de struct ControlPD si no usas la cabecera global
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
  // 1. Mantiene corriendo el control PD de motores a alta frecuencia
  actualizarControlMotores();
  actualizarLecturasUltrasonido();

  // 2. Lecturas continuas de sensores
  int distFrontal = leerUltrasonidoAlto();
  int distLateral = leerUltrasonidoLateral();

  // 3. Máquina de estados reducida
  switch (estadoPrueba) {

    case AVANZAR_AL_FRENTE:
      // Avance longitudinal constante
      moverOmni(0, VELOCIDAD_BASE, 0);

      // Si detecta piscina o pared enfrente (< 20 cm)
      if (distFrontal > 0 && distFrontal < 20) {
        pararMotores();
        tiempoInicioEstado = millis();
        estadoPrueba = ESQUIVAR_DERECHA;
        Serial.println("-> Obstáculo detectado. Iniciando esquiva a la derecha...");
      }
      break;

    case ESQUIVAR_DERECHA: {
      // Ajuste PD usando la distancia del sensor lateral a la piscina
      int correccionPD = obtenerCorreccionParedPD(distLateral);

      // Se desplaza a la derecha (+Vx) mientras corrige el avance/retroceso (+/- Vy) con el PD
      moverOmni(VELOCIDAD_BASE, correccionPD, 0);

      // Si el sensor frontal ya ve libre (> 35 cm)
      if (distFrontal > 35 && distFrontal < 300) {
        tiempoDespejado = millis();
        estadoPrueba = VERIFICAR_LIBERACION;
        Serial.println("-> Frente despejado. Verificando liberación...");
      }
      break;
    }

    case VERIFICAR_LIBERACION: {
      int correccionPD = obtenerCorreccionParedPD(distLateral);
      moverOmni(VELOCIDAD_BASE, correccionPD, 0);

      // Si el obstáculo vuelve a aparecer enfrente, regresa al estado de esquiva
      if (distFrontal > 0 && distFrontal <= 35) {
        estadoPrueba = ESQUIVAR_DERECHA;
        break;
      }

      // Sostiene la esquiva lateral durante 800 ms para rebasar la esquina del obstáculo
      if (millis() - tiempoDespejado > 800) {
        pararMotores();
        estadoPrueba = AVANZAR_AL_FRENTE;
        Serial.println("-> Obstáculo superado por completo. Reanudando avance al frente...");
      }
      break;
    }
  }
}