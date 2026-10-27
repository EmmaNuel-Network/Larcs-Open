// ============================================================================
// PESTAÑA PRINCIPAL: Robot_Octavio_I.ino
// Orquestador FSM - Estrategia Nivel 1 -> Nivel 2 -> Nivel 3
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>

// 1. INSTANCIA GLOBAL DEL EXPANSOR I2C
Adafruit_MCP23X17 mcp;

// 2. ESTRUCTURA DE CONTROL PD DE MOTORES
struct ControlPD {
  float setpoint = 0.0f;
  long pulsosAnteriores = 0;
  float errorPrev = 0.0f;
  float Kp = 1.2f;
  float Kd = 0.15f;
  int outputPWM = 0;
};

ControlPD pdM1, pdM2, pdM3, pdM4;

// 3. ESTADOS DE LA MÁQUINA DE ESTADOS FINITOS (ESTRATEGIA TRES NIVELES)
enum EstadoFSM {
  ESPERA_INICIO,
  IR_A_CULTIVO,
  ESQUIVAR_PISCINA,
  IR_A_IZQUIERDA_INICIAL,
  
  RECOLECCION_NIVEL_1,
  AVANZAR_A_NIVEL_2,
  RECOLECCION_NIVEL_2,
  AVANZAR_A_NIVEL_3,
  RECOLECCION_NIVEL_3,
  
  REGRESO_BENEFICIADERO,
  FIN_MISION
};

EstadoFSM estadoActual = ESPERA_INICIO;

// 4. DECLARACIÓN DE FUNCIONES EXTERNAS
void setupMotores();
void setupUltrasonidos();
void setupInfrarrojos();

void actualizarControlMotores();
void actualizarLecturasUltrasonido();

void moverOmni(int Vx, int Vy, int W);
void pararMotores();
void accionarRecolectorYClasificar(bool esValida);

int obtenerDistanciaMinimaFrontal();
bool estaSobreLineaFrontal();
bool detectoMargenIzquierdo();
bool detectoMargenDerecho();

// 5. PARÁMETROS DE COMPETENCIA Y NAVEGACIÓN
const int VELOCIDAD_BASE                    = 145; 
const int VELOCIDAD_BARRIDO                 = 150; 
const int UMBRAL_PISCINA_CM                 = 20;
const unsigned long VENTANA_IGNORAR_LINEA_MS  = 1000;

const unsigned long TIEMPO_ENTRE_PELOTAS_MS   = 850;  
const unsigned long TIEMPO_ESPACIO_ARBOL_MS   = 1500; 
const unsigned long TIEMPO_CAMBIO_NIVEL_MS    = 1200;

unsigned long tiempoInicioMision = 0;
unsigned long tiempoInicioEstado = 0;
int direccionBarrido = 1;

const int pinServo25 = 25; // Servo Garra
const int pinServo26 = 26; // Servo Clasificador

void esperarConControl(unsigned long ms) {
  unsigned long inicio = millis();
  while (millis() - inicio < ms) {
    actualizarControlMotores();
    actualizarLecturasUltrasonido();
    delay(2);
  }
}

// 6. CONSULTA SERIAL A RASPBERRY PI
bool consultarColorPelotaRPi() {
  pararMotores();
  
  while (Serial.available() > 0) { Serial.read(); }
  Serial.write('C');
  
  unsigned long tInicio = millis();
  char respuesta = '0';
  
  while (millis() - tInicio < 600) {
    if (Serial.available() > 0) {
      respuesta = Serial.read();
      if (respuesta == '1' || respuesta == '0') break;
    }
    delay(5);
  }
  
  return (respuesta == '1');
}

// 7. BARRIDO COMPLETO DE UN NIVEL (3 ÁRBOLES)
void ejecutarBarridoNivel(int direccion) {
  for (int arbol = 1; arbol <= 3; arbol++) {
    for (int pelota = 1; pelota <= 4; pelota++) {
      moverOmni(direccion * VELOCIDAD_BARRIDO, 0, 0);
      esperarConControl(TIEMPO_ENTRE_PELOTAS_MS);

      pararMotores();
      bool esValida = consultarColorPelotaRPi();
      accionarRecolectorYClasificar(esValida);
    }

    if (arbol < 3) {
      moverOmni(direccion * VELOCIDAD_BARRIDO, 0, 0);
      esperarConControl(TIEMPO_ESPACIO_ARBOL_MS);
      pararMotores();
    }
  }
}

// 8. SETUP INICIAL
void setup() {
  Serial.begin(115200); 
  
  Wire.begin(21, 22);
  Wire.setClock(100000); 
  Wire.setTimeOut(100);

  if (!mcp.begin_I2C()) {
    while (1) { delay(500); }
  }

  setupMotores();
  setupUltrasonidos();
  setupInfrarrojos();
  pararMotores();

  pinMode(pinServo25, OUTPUT);
  pinMode(pinServo26, OUTPUT);
  digitalWrite(pinServo25, LOW);
  digitalWrite(pinServo26, LOW);

  accionarRecolectorYClasificar(true);

  delay(1000);
}

// 9. BUCLE PRINCIPAL (LOOP)
void loop() {
  actualizarControlMotores();

  int distFrontal = obtenerDistanciaMinimaFrontal();
  bool lineaFrontal = estaSobreLineaFrontal();
  bool margenIzq = detectoMargenIzquierdo();
  bool margenDer = detectoMargenDerecho();
  bool lineaArmada = (millis() - tiempoInicioMision > VENTANA_IGNORAR_LINEA_MS);

  switch (estadoActual) {

    case ESPERA_INICIO:
      if (millis() > 1500) {
        tiempoInicioMision = millis();
        estadoActual = IR_A_CULTIVO;
      }
      break;

    case IR_A_CULTIVO:
      moverOmni(0, VELOCIDAD_BASE, 0);

      if (distFrontal > 0 && distFrontal < UMBRAL_PISCINA_CM) {
        estadoActual = ESQUIVAR_PISCINA;
        break;
      }

      if (lineaFrontal && lineaArmada) {
        pararMotores();
        tiempoInicioEstado = millis();
        estadoActual = IR_A_IZQUIERDA_INICIAL;
      }
      break;

    case ESQUIVAR_PISCINA:
      moverOmni(-VELOCIDAD_BASE, VELOCIDAD_BASE / 2, 0);
      esperarConControl(800);
      moverOmni(VELOCIDAD_BASE, VELOCIDAD_BASE, 0);
      esperarConControl(800);
      estadoActual = IR_A_CULTIVO;
      break;

    case IR_A_IZQUIERDA_INICIAL:
      moverOmni(-VELOCIDAD_BARRIDO, 0, 0);
      if (margenIzq || (millis() - tiempoInicioEstado > 3000)) {
        pararMotores();
        direccionBarrido = 1;
        estadoActual = RECOLECCION_NIVEL_1;
      }
      break;

    case RECOLECCION_NIVEL_1:
      ejecutarBarridoNivel(direccionBarrido);
      estadoActual = AVANZAR_A_NIVEL_2;
      break;

    case AVANZAR_A_NIVEL_2:
      pararMotores();
      moverOmni(0, VELOCIDAD_BASE, 0);
      esperarConControl(TIEMPO_CAMBIO_NIVEL_MS);
      pararMotores();
      
      direccionBarrido = -1;
      estadoActual = RECOLECCION_NIVEL_2;
      break;

    case RECOLECCION_NIVEL_2:
      ejecutarBarridoNivel(direccionBarrido);
      estadoActual = AVANZAR_A_NIVEL_3;
      break;

    case AVANZAR_A_NIVEL_3:
      pararMotores();
      moverOmni(0, VELOCIDAD_BASE, 0);
      esperarConControl(TIEMPO_CAMBIO_NIVEL_MS);
      pararMotores();
      
      direccionBarrido = 1;
      estadoActual = RECOLECCION_NIVEL_3;
      break;

    case RECOLECCION_NIVEL_3:
      ejecutarBarridoNivel(direccionBarrido);
      estadoActual = REGRESO_BENEFICIADERO;
      break;

    case REGRESO_BENEFICIADERO:
      pararMotores();
      moverOmni(0, -VELOCIDAD_BASE, 0);
      esperarConControl(4500);
      pararMotores();
      estadoActual = FIN_MISION;
      break;

    case FIN_MISION:
      pararMotores();
      break;
  }
}
