// ============================================================================
// PESTAÑA PRINCIPAL: Robot-Larc.ino
// FSM Completa Integrada: Esquiva + Multi-Árbol + Cosecha 16 Pelotas + Vaciado
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <ESP32Servo.h>

Adafruit_MCP23X17 mcp;

// Estructura de control PD para motores DC
struct ControlPD {
  float setpoint = 0.0f;
  long pulsosAnteriores = 0;
  float errorPrev = 0.0f;
  float Kp = 1.2f;
  float Kd = 0.15f;
  int outputPWM = 0;
};

// Referencia global externa para las instancias reales creadas en Motores.ino
extern ControlPD pdM1, pdM2, pdM3, pdM4;

// Máquina de Estados Finitos (FSM)
enum EstadoFSM {
  ESPERA_INICIO,
  IR_A_CULTIVO,
  ESQUIVAR_PISCINA,
  VERIFICAR_LIBERACION,
  HOMING_Y_ESCUADRADO,
  RECOLECCION_16_PELOTAS,
  DESPLAZAR_SIGUIENTE_ARBOL,
  GIRO_180_REGRESO,
  REGRESO_BENEFICIADERO,
  DESCARGA_PAYLOAD,
  FIN_MISION
};

EstadoFSM estadoActual = ESPERA_INICIO;

// Configuración de Misión Multi-Árbol
const int TOTAL_ARBOLES = 2; // Ajustar a la cantidad real de árboles en fila
int arbolActual         = 1;

// Prototipos de Funciones de los otros módulos
void setupMotores();
void actualizarControlMotores();
void moverOmni(int Vx, int Vy, int W);
void girar180Grados();
void pararMotores();
void resetEncoders();

void setupInfrarrojos();
int obtenerCorreccionLineaPD();
bool estaSobreLineaFrontal();
bool detectoMargenIzquierdo();

void setupUltrasonidos();
void actualizarLecturasUltrasonido();
int leerUltrasonidoIzquierdo();
int leerUltrasonidoDerecho();
int obtenerCorreccionParedPD(int distMedida);

void acercarAPelota();
void retirarDePelota();
void cosecharEstandar();
void cosecharEspecialNivel3();
void vaciarCompuertaTrasera();
void desplazarseALaSiguientePelota(int direccion);
void desplazarseEntreArboles();
void moverMM(float mm, bool subir);
void apagarBobinasElevador();

// Constantes de Mantenimiento y Tiempos
const int VELOCIDAD_BASE                      = 130;
const int UMBRAL_PISCINA_CM                 = 20;
const int UMBRAL_LIBRE_CM                   = 35;
const unsigned long TIEMPO_CONFIRMAR_LIBRE_MS = 800;
const unsigned long VENTANA_IGNORAR_LINEA_MS  = 1200;
const float DISTANCIA_NIVEL_MM              = 150.0f;

unsigned long tiempoInicioMision = 0;
unsigned long tiempoInicioEstado = 0;

void esperarConControl(unsigned long ms) {
  unsigned long inicio = millis();
  while (millis() - inicio < ms) {
    actualizarControlMotores();
    actualizarLecturasUltrasonido();
    delay(2);
  }
}

// Rutina de Cosecha Autónoma de 16 Pelotas (3 Niveles)
void ejecutarRutina16Pelotas(int numeroArbol) {
  int pelotasTotal = 0;
  Serial.printf("\n=======================================================\n");
  Serial.printf("   INICIANDO COSECHA EN ÁRBOL #%d DE %d\n", numeroArbol, TOTAL_ARBOLES);
  Serial.printf("=======================================================\n");

  // Nivel 1: 5 Pelotas (Fila Inferior - 390 mm) | Izquierda -> Derecha
  for (int p = 1; p <= 5; p++) {
    pelotasTotal++;
    acercarAPelota();
    cosecharEstandar();
    retirarDePelota();
    if (p < 5) desplazarseALaSiguientePelota(1);
  }

  // Subir Elevador a Nivel 2 (+150 mm)
  moverMM(DISTANCIA_NIVEL_MM, true);
  delay(300);

  // Nivel 2: 6 Pelotas (Fila Media - 540 mm) | Derecha -> Izquierda
  for (int p = 1; p <= 6; p++) {
    pelotasTotal++;
    acercarAPelota();
    cosecharEstandar();
    retirarDePelota();
    if (p < 6) desplazarseALaSiguientePelota(-1);
  }

  // Subir Elevador a Nivel 3 (+150 mm)
  moverMM(DISTANCIA_NIVEL_MM, true);
  delay(300);

  // Nivel 3: 5 Pelotas (Fila Superior - 690 mm - Rutina Especial)
  for (int p = 1; p <= 5; p++) {
    pelotasTotal++;
    cosecharEspecialNivel3();
    if (p < 5) desplazarseALaSiguientePelota(1);
  }

  // Descenso Completo del Elevador a la Base (-300 mm)
  moverMM(DISTANCIA_NIVEL_MM * 2.0f, false);
  delay(500);
}

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);
  Wire.setClock(400000); // Fast-I2C a 400 kHz

  if (!mcp.begin_I2C()) {
    Serial.println("[CRÍTICO] Fallo al detectar MCP23017 en I2C");
    while (1) { delay(500); }
  }

  setupMotores();
  setupUltrasonidos();
  setupInfrarrojos();
  pararMotores();

  apagarBobinasElevador();
  delay(1000);
}

void loop() {
  actualizarControlMotores();
  actualizarLecturasUltrasonido();

  int distIzq = leerUltrasonidoIzquierdo();
  int distDer = leerUltrasonidoDerecho();
  int distMinimaFrontal = min(distIzq, distDer);

  bool lineaFrontal = estaSobreLineaFrontal();
  bool margenIzq    = detectoMargenIzquierdo();
  bool lineaArmada  = (millis() - tiempoInicioMision > VENTANA_IGNORAR_LINEA_MS);

  switch (estadoActual) {

    case ESPERA_INICIO:
      if (millis() > 1500) {
        tiempoInicioMision = millis();
        estadoActual = IR_A_CULTIVO;
      }
      break;

    case IR_A_CULTIVO: {
      int correccionLinea = lineaFrontal ? obtenerCorreccionLineaPD() : 0;
      moverOmni(0, VELOCIDAD_BASE, correccionLinea);

      if (distMinimaFrontal >= 3 && distMinimaFrontal < UMBRAL_PISCINA_CM) {
        estadoActual = ESQUIVAR_PISCINA;
        break;
      }

      if (lineaFrontal && lineaArmada) {
        pararMotores();
        tiempoInicioEstado = millis();
        estadoActual = HOMING_Y_ESCUADRADO;
      }
      break;
    }

    case ESQUIVAR_PISCINA: {
      int correccionPD = obtenerCorreccionParedPD(distIzq);
      moverOmni(VELOCIDAD_BASE, correccionPD, 0);

      if (lineaFrontal && lineaArmada) {
        pararMotores();
        estadoActual = HOMING_Y_ESCUADRADO;
        break;
      }
      if (distMinimaFrontal > UMBRAL_LIBRE_CM && distMinimaFrontal < 300) {
        tiempoInicioEstado = millis();
        estadoActual = VERIFICAR_LIBERACION;
      }
      break;
    }

    case VERIFICAR_LIBERACION: {
      int correccionPD = obtenerCorreccionParedPD(distIzq);
      moverOmni(VELOCIDAD_BASE, correccionPD, 0);

      if (distMinimaFrontal >= 3 && distMinimaFrontal <= UMBRAL_LIBRE_CM) {
        estadoActual = ESQUIVAR_PISCINA;
        break;
      }
      if (millis() - tiempoInicioEstado > TIEMPO_CONFIRMAR_LIBRE_MS) {
        estadoActual = IR_A_CULTIVO;
      }
      break;
    }

    case HOMING_Y_ESCUADRADO:
      moverOmni(-110, 0, 0); // Alineación lateral a la izquierda del árbol actual
      if (margenIzq || (millis() - tiempoInicioEstado > 4000)) {
        pararMotores();
        resetEncoders();
        estadoActual = RECOLECCION_16_PELOTAS;
      }
      break;

    case RECOLECCION_16_PELOTAS:
      ejecutarRutina16Pelotas(arbolActual);
      
      if (arbolActual < TOTAL_ARBOLES) {
        estadoActual = DESPLAZAR_SIGUIENTE_ARBOL;
      } else {
        estadoActual = GIRO_180_REGRESO;
      }
      break;

    case DESPLAZAR_SIGUIENTE_ARBOL:
      desplazarseEntreArboles();
      arbolActual++;
      tiempoInicioEstado = millis();
      estadoActual = HOMING_Y_ESCUADRADO;
      break;

    case GIRO_180_REGRESO:
      pararMotores();
      delay(300);
      girar180Grados();
      tiempoInicioEstado = millis();
      estadoActual = REGRESO_BENEFICIADERO;
      break;

    case REGRESO_BENEFICIADERO:
      moverOmni(0, VELOCIDAD_BASE, 0);
      if (lineaFrontal || (millis() - tiempoInicioEstado > 5500)) {
        pararMotores();
        estadoActual = DESCARGA_PAYLOAD;
      }
      break;

    case DESCARGA_PAYLOAD:
      pararMotores();
      delay(500);
      vaciarCompuertaTrasera(); // Vaciado de todas las pelotas recolectadas en el beneficiadero
      estadoActual = FIN_MISION;
      break;

    case FIN_MISION:
      pararMotores();
      apagarBobinasElevador();
      break;
  }
}