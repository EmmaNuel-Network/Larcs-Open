// ============================================================================
// ORQUESTADOR DE PRUEBA: Esquivador de Piscinas con Ultrasonidos Bajos
// Integra directamente: Motores.ino y Ultrasonidos.ino
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>

// Instancia global compartida para la expansión I2C
Adafruit_MCP23X17 mcp;

// Estrutura utilizada por Motores.ino
struct ControlPD {
  float setpoint = 0.0f;
  long pulsosAnteriores = 0;
  float errorPrev = 0.0f;
  float Kp = 1.2f;
  float Kd = 0.15f;
  int outputPWM = 0;
};

// ----------------------------------------------------------------------------
// Declaración de Funciones Externas (Provenientes de Motores.ino y Ultrasonidos.ino)
// ----------------------------------------------------------------------------
// De Motores.ino:
void setupMotores();
void actualizarControlMotores();
void moverOmni(int Vx, int Vy, int W);
void pararMotores();

// De Ultrasonidos.ino:
void setupUltrasonidos();
int leerUltrasonidoFrontalBajo();
int leerUltrasonidoLateral();

// ----------------------------------------------------------------------------
// Estados para la Prueba Integrada de Esquiva
// ----------------------------------------------------------------------------
enum EstadoPrueba {
  AVANZAR_LIBRE,       // Avance longitudinal hacia adelante
  ESQUIVAR_OBSTACULO,  // Evitación lateral holonómica
  DETENIDO_SEGURIDAD   // Parada de protección por tiempo límite (Timeout)
};

EstadoPrueba estadoActual = AVANZAR_LIBRE;

// Parámetros de prueba
const int UMBRAL_DISTANCIA_PISCINA = 20; // Detección de piscina a menos de 20 cm
const int DISTANCIA_DESPEJADA      = 35; // Umbral para considerar vía libre (> 35 cm)
const unsigned long TIMEOUT_ESQUIVA = 5000; // Máximo 5s esquivando antes de frenar

unsigned long tiempoInicioEsquiva = 0;
unsigned long tiempoLibrePiscina  = 0;
int direccionEsquiva              = 1;  // 1: Derecha (+Vx), -1: Izquierda (-Vx)

// ============================================================================
// SETUP PRINCIPAL
// ============================================================================
void setup() {
  Serial.begin(115200);

  // Inicialización del bus I2C para el expansor MCP23017
  Wire.begin(21, 22); // SDA = 21, SCL = 22
  if (!mcp.begin_I2C()) {
    Serial.println("¡Error de comunicación con MCP23017! Verifique conexiones.");
    while (1); 
  }

  // Inicialización de hardware mediante los subsistemas integrados
  setupMotores();
  setupUltrasonidos();
  
  pararMotores();
  Serial.println("--- PRUEBA INTEGRADA: MOTOR + ULTRASONIDOS BAJOS ---");
  delay(1000); // Pausa de estabilización inicial
}

// ============================================================================
// LOOP PRINCIPAL (Máquina de Estados de la Prueba)
// ============================================================================
void loop() {
  // 1. Mantenimiento periódico obligatorio del control PID/PD de motores (50 Hz)
  actualizarControlMotores();

  // 2. Muestreo de los dos ultrasonidos bajos requeridos
  int distFrontalBaja = leerUltrasonidoFrontalBajo();
  int distLateral     = leerUltrasonidoLateral();

  // 3. Lógica de control de movimiento integrando los ultrasonidos
  switch (estadoActual) {

    case AVANZAR_LIBRE:
      // Si el sensor frontal bajo detecta la piscina/obstáculo cercano
      if (distFrontalBaja > 0 && distFrontalBaja < UMBRAL_DISTANCIA_PISCINA) {
        Serial.print("Piscina detectada a ");
        Serial.print(distFrontalBaja);
        Serial.println(" cm. Iniciando esquiva lateral...");

        tiempoInicioEsquiva = millis();
        tiempoLibrePiscina  = 0;
        
        // Determina sentido de esquiva según espacio lateral disponible
        if (distLateral < 20) {
          direccionEsquiva = 1;  // Mover a la derecha si hay pared/obstáculo a la izquierda
        } else {
          direccionEsquiva = -1; // Mover a la izquierda por defecto
        }

        estadoActual = ESQUIVAR_OBSTACULO;
        break;
      }

      // Avance hacia adelante constante en eje Y (+Vy)
      moverOmni(0, 160, 0);
      break;

    case ESQUIVAR_OBSTACULO:
      // Protección por Timeout: Si dura más de 5 segundos esquivando, se detiene
      if (millis() - tiempoInicioEsquiva > TIMEOUT_ESQUIVA) {
        Serial.println("ALERTA: Tiempo límite de esquiva superado. Deteniendo.");
        pararMotores();
        estadoActual = DETENIDO_SEGURIDAD;
        break;
      }

      // Validación de ruta frontal libre durante 600 ms continuos
      if (distFrontalBaja > DISTANCIA_DESPEJADA && distFrontalBaja < 300) {
        if (tiempoLibrePiscina == 0) {
          tiempoLibrePiscina = millis();
        } else if (millis() - tiempoLibrePiscina > 600) {
          Serial.println("Ruta frontal despejada. Retomando marcha adelante.");
          tiempoLibrePiscina = 0;
          estadoActual = AVANZAR_LIBRE;
          break;
        }
      } else {
        tiempoLibrePiscina = 0; // Reinicia el conteo si vuelve a aparecer el obstáculo
      }

      // Desplazamiento lateral holonómico (+/- Vx) para rebasar la piscina
      moverOmni(direccionEsquiva * 160, 0, 0);
      break;

    case DETENIDO_SEGURIDAD:
      pararMotores();
      break;
  }
}