// ==========================================
// PESTAÑA PRINCIPAL: Robot-Larc.ino
// FSM única + modos de prueba incrementales
// Navegación completa: ida con esquiva, escuadrado, homing, barrido
// de 3 niveles, regreso, posicionamiento y descarga.
// (Visión y recolección se integran después en el BARRIDO)
// ==========================================
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>

Adafruit_MCP23X17 mcp;

// Tipo compartido con Motores.ino (debe quedar en esta pestaña, al inicio)
struct ControlPD {
  float setpoint = 0.0f;
  long pulsosAnteriores = 0;
  float errorPrev = 0.0f;
  float Kp = 1.2f;
  float Kd = 0.15f;
  int outputPWM = 0;
};

// ==========================================
// 1. ESTADOS DE LA FSM
// ==========================================
enum EstadoFSM {
  NAVEGAR,                // Avanza al frente con rumbo fijo (ida o vuelta)
  ESQUIVAR_PISCINA,       // Lógica probada de PruebaIntegracionUltrasonidos
  VERIFICAR_LIBERACION,
  ESCUADRAR,              // Se pone perpendicular a la línea con los 2 IR frontales
  HOMING_MARGEN,          // Strafe a la izquierda hasta el margen izquierdo
  BARRIDO,                // Recorre la fila de árboles de margen a margen, 3 niveles
  CENTRAR_FILA,           // Vuelve al centro de la fila antes de regresar
  ALINEAR_Y_CAPTURAR,     // Reservado para la visión (aún no se usa)
  GIRO_180,               // Media vuelta controlada por odometría
  POSICIONAR_DESCARGA,    // Strafe hasta quedar frente al recipiente
  RETROCEDER_DESCARGA,    // Acerca la compuerta trasera al recipiente
  DESCARGA_PAYLOAD,
  CALIB_AVANCE,           // Solo PRUEBA_GIRO: avance recto para calibrar cm
  FIN_MISION,
  ESTADO_ERROR
};

enum FaseMision { FASE_IDA, FASE_VUELTA };

// ==========================================
// 2. MODOS DE PRUEBA
// ==========================================
enum ModoPrueba {
  PRUEBA_SENSORES,      // Robot quieto: imprime IR y ultrasonidos cada 200 ms
  PRUEBA_ESQUIVA,       // Idéntico a PruebaIntegracionUltrasonidos (sin IR ni rumbo)
  PRUEBA_DEMO,          // Avanza, esquiva, gira 180° en la línea (PB0) y repite
  PRUEBA_GIRO,          // Calibración por Serial: 'g' giro, 'a' avance, '+'/'-' ajuste
  PRUEBA_IDA,           // Salida -> esquiva -> línea del cultivo -> escuadra -> homing
  PRUEBA_BARRIDO,       // Frente a la línea del cultivo: escuadra -> homing -> 3 niveles
  PRUEBA_REGRESO,       // En la línea del cultivo: giro -> vuelta -> descarga
  PRUEBA_RUTINA,        // Rutina completa de navegación (con monitor)
  COMPETENCIA           // Rutina completa: arranca sola tras RETARDO_ARRANQUE_MS
};

// >>>>>>>>>> CAMBIAR AQUÍ EL MODO Y VOLVER A SUBIR <<<<<<<<<<
const ModoPrueba MODO = PRUEBA_GIRO;

// Subsistemas según el modo
const bool LEER_IR = (MODO != PRUEBA_ESQUIVA && MODO != PRUEBA_DEMO && MODO != PRUEBA_GIRO);

// Actuadores: desactivados hasta integrar esa parte (pines pendientes).
// Mientras estén en false, el elevador y la descarga se simulan por Serial.
const bool HABILITAR_SERVO    = false;
const bool HABILITAR_ELEVADOR = false;
const bool USAR_SERVO    = HABILITAR_SERVO    && (MODO == PRUEBA_REGRESO || MODO == PRUEBA_RUTINA || MODO == COMPETENCIA);
const bool USAR_ELEVADOR = HABILITAR_ELEVADOR && (MODO == PRUEBA_BARRIDO || MODO == PRUEBA_RUTINA || MODO == COMPETENCIA);

EstadoFSM estadoActual  = FIN_MISION;
EstadoFSM estadoTrasGiro = NAVEGAR;   // A dónde ir al terminar GIRO_180
FaseMision fase          = FASE_IDA;

// ==========================================
// 3. PINES Y ACTUADORES ADICIONALES
// ==========================================
const int pinServoCompuerta  = 25; // Pendiente: choca con el trigger lateral
Servo servoCompuerta;

const int pinStepElevador    = 14; // MCP PB6 (pendiente: el 28BYJ-48 usa IN1-IN4)
const int pinDirElevador     = 15; // MCP PB7
int nivelActualH             = 1;

// ==========================================
// 4. PARÁMETROS
// ==========================================
// --- Navegación y esquiva (valores de la prueba funcional) ---
const int VELOCIDAD_BASE                      = 120;
const int UMBRAL_PISCINA_CM                   = 20;
const int UMBRAL_LIBRE_CM                     = 35;
const unsigned long TIEMPO_CONFIRMAR_LIBRE_MS = 800;
const unsigned long TIMEOUT_ESQUIVA_MS        = 6000; // Si no sale, prueba por el otro lado
// Si el ultrasonido lateral ve partes del propio robot, poner en false.
const bool USAR_LATERAL_EN_DETECCION = true;

// --- Arranque (sin botón) ---
const unsigned long RETARDO_ARRANQUE_MS = 2000; // Tiempo entre encender y empezar a moverse

// --- Línea ---
const unsigned long VENTANA_IGNORAR_LINEA_MS = 1500; // Al arrancar y tras cada giro
// La línea debe verse en negro de forma CONTINUA mientras el robot recorre
// esta distancia. Filtra puntos y manchas del piso sin importar la velocidad.
// La línea mide 1.9 cm: no subir de ~1.2. Subir si aún hay falsos disparos.
const float LONGITUD_MIN_LINEA_CM = 0.8f;

// --- Control de rumbo (odometría de encoders) ---
const bool  USAR_CONTROL_RUMBO = true;
const float KP_RUMBO           = 4.0f;  // W por cada grado de error
const int   W_MAX_RUMBO        = 60;

// --- Giro 180° ---
const long PULSOS_GIRO_180           = 600;  // CALIBRAR con PRUEBA_GIRO
long pulsosGiro180                   = PULSOS_GIRO_180;
const int  VELOCIDAD_GIRO_MAX        = 150;
const int  VELOCIDAD_GIRO_MIN        = 70;   // Por encima de la zona muerta
const float TOLERANCIA_GIRO_GRADOS   = 2.0f;
const unsigned long PAUSA_ANTES_GIRO_MS = 300;
const unsigned long TIMEOUT_GIRO_MS  = 5000;

// --- Distancias (odometría) ---
float pulsosPorCm                    = 25.0f; // CALIBRAR con PRUEBA_GIRO ('a')
const long PULSOS_CALIBRACION_AVANCE = 1500;

// --- Escuadrado ---
const int V_LENTO                    = 60;   // Avance lento buscando la línea
const int W_ESCUADRE                 = 60;
const int SIGNO_ESCUADRE             = 1;    // Si al escuadrar se aleja de la línea, poner -1
const int LECTURAS_CONFIRMAR_ESCUADRE = 3;
const unsigned long TIMEOUT_ESCUADRE_MS = 4000;

// --- Homing y barrido ---
const int V_HOMING                   = 100;
const int V_BARRIDO                  = 100;
const float KP_AVANCE                = 0.5f; // Mantiene la distancia a la línea al hacer strafe
const int   MAX_CORRECCION_AVANCE    = 40;
const int LECTURAS_CONFIRMAR_MARGEN  = 2;
const unsigned long TIMEOUT_HOMING_MS       = 8000;
const unsigned long TIMEOUT_BARRIDO_NIVEL_MS = 12000;
const unsigned long TIMEOUT_CENTRAR_MS       = 6000;
const float TOLERANCIA_CENTRAR_CM            = 3.0f;
// Si el robot pisa un margen mientras navega, se aleja de él con esta velocidad lateral
const int V_REPELER_MARGEN                   = 60;
const int NIVELES_BARRIDO            = 3;
const int UMBRAL_ARBOL_CM            = 25;   // Solo registro, para la visión futura
const unsigned long TIEMPO_LIMITE_RECOLECCION_MS = 150000; // Corta el barrido y regresa

// --- Descarga (AJUSTAR con la geometría real de la cancha) ---
const int   LADO_RECIPIENTE              = -1;   // -1 = izquierda, +1 = derecha
const float DESPLAZAMIENTO_DESCARGA_CM   = 0.0f; // 0 = ir hasta el margen
const unsigned long TIMEOUT_POSICIONAR_MS = 8000;
const float RETROCESO_DESCARGA_CM        = 10.0f;
const unsigned long TIMEOUT_RETROCESO_MS = 4000;

const unsigned long WATCHDOG_CAMARA = 800;

// --- Demo: sensor de línea en PB0 con autocalibración ---
const int pinTrackLinea        = 8;
const bool AUTO_CALIBRAR_TRACK = true;
int trackValorNegro            = HIGH;

// ==========================================
// 5. VARIABLES DE CONTROL
// ==========================================
unsigned long tiempoInicioMision = 0;
unsigned long tiempoInicioEstado = 0;
unsigned long tiempoArmadoLinea  = 0;
unsigned long inicioEsquiva      = 0;
unsigned long inicioGiro         = 0;
unsigned long ultimoMonitor      = 0;
unsigned long finGiroCalibracion = 0;  // PRUEBA_GIRO: para medir el giro ya detenido
bool estadoIniciado              = false;

// Filtro de negro por distancia: un sensor solo cuenta como "sobre línea"
// si ve negro sin interrupción mientras el robot recorre LONGITUD_MIN_LINEA_CM.
struct FiltroNegro {
  bool  viendo = false;
  float desdeAvance = 0.0f;
  float desdeLateral = 0.0f;
};
FiltroNegro filtroLinea, filtroMargenIzq, filtroMargenDer;
int contadorEscuadre  = 0;
int contadorMargen    = 0;
int trackCrudo        = 0;
int direccionEsquiva  = 1;   // +1 derecha, -1 izquierda
int direccionBarrido  = 1;   // +1 derecha, -1 izquierda
int nivelBarrido      = 1;
bool frenteAArbol     = false;
int arbolesVistos     = 0;

// Odometría (en pulsos, marco del robot)
long encPrev[4]          = {0, 0, 0, 0};
float odoAvance          = 0.0f;
float odoLateral         = 0.0f;
float odoRumbo           = 0.0f;
float rumboObjetivo      = 0.0f;
float avanceObjetivo     = 0.0f;
float rumboInicioGiro    = 0.0f;
float refAvance          = 0.0f;
float refLateral         = 0.0f;
float latMargenIzq       = 0.0f;  // Odometría lateral en el margen izquierdo (homing)
float latMargenDer       = 0.0f;  // Odometría lateral en el margen derecho (barrido)
bool  margenDerMedido    = false;

int errorCamaraX                  = 0;
int errorCamaraY                  = 0;
bool pelotaCentrada               = false;
unsigned long ultimaLecturaCamara = 0;
char bufferUART[64];
size_t idxBuffer = 0;

// ==========================================
// 6. FUNCIONES EXTERNAS (otras pestañas)
// ==========================================
void setupMotores();
void actualizarControlMotores();
void moverOmni(int Vx, int Vy, int W);
void pararMotores();
long getPulsosM1();
long getPulsosM2();
long getPulsosM3();
long getPulsosM4();

void setupInfrarrojos();
bool estaSobreLineaFrontal();
bool detectoMargenIzquierdo();
bool detectoMargenDerecho();
float calcularErrorLinea();
int irFrontalIzq();
int irFrontalDer();

void setupUltrasonidos();
void actualizarLecturasUltrasonido();
int leerUltrasonidoAlto();
int leerUltrasonidoLateral();
int obtenerCorreccionParedPD(int distMedida);

// ==========================================
// 7. ODOMETRÍA Y CONTROL DE RUMBO
// ==========================================
// Cinemática de moverOmni:  v1 = Vy+Vx-W, v2 = Vy-Vx+W, v3 = Vy-Vx-W, v4 = Vy+Vx+W
// Inversa:  avance = (d1+d2+d3+d4)/4, lateral = (d1-d2-d3+d4)/4, rumbo = (-d1+d2-d3+d4)/4
// W > 0 hace crecer odoRumbo. 180° equivalen a pulsosGiro180.
void leerEncoders(long p[4]) {
  p[0] = getPulsosM1(); p[1] = getPulsosM2();
  p[2] = getPulsosM3(); p[3] = getPulsosM4();
}

void iniciarOdometria() {
  leerEncoders(encPrev);
  odoAvance = odoLateral = odoRumbo = 0.0f;
  rumboObjetivo = avanceObjetivo = 0.0f;
}

void actualizarOdometria() {
  long p[4];
  leerEncoders(p);
  float d1 = p[0] - encPrev[0], d2 = p[1] - encPrev[1];
  float d3 = p[2] - encPrev[2], d4 = p[3] - encPrev[3];
  for (int i = 0; i < 4; i++) encPrev[i] = p[i];

  odoAvance  += ( d1 + d2 + d3 + d4) / 4.0f;
  odoLateral += ( d1 - d2 - d3 + d4) / 4.0f;
  odoRumbo   += (-d1 + d2 - d3 + d4) / 4.0f;
}

float pulsosAGrados(float p) { return p * 180.0f / (float)pulsosGiro180; }
float rumboGrados()          { return pulsosAGrados(odoRumbo); }
float pulsosACm(float p)     { return p / pulsosPorCm; }
long  cmAPulsos(float cm)    { return (long)(cm * pulsosPorCm); }

bool controlRumboActivo() {
  return USAR_CONTROL_RUMBO && MODO != PRUEBA_ESQUIVA;
}

// Corrección de giro para mantener rumboObjetivo
int correccionRumbo() {
  if (!controlRumboActivo()) return 0;
  float errGrados = pulsosAGrados(rumboObjetivo - odoRumbo);
  return (int)constrain(KP_RUMBO * errGrados, (float)-W_MAX_RUMBO, (float)W_MAX_RUMBO);
}

// Corrección de avance para no alejarse de la línea al hacer strafe
int correccionAvance() {
  float err = avanceObjetivo - odoAvance;
  return (int)constrain(KP_AVANCE * err, (float)-MAX_CORRECCION_AVANCE, (float)MAX_CORRECCION_AVANCE);
}

// Fija la referencia actual como "recto" y "a esta distancia de la línea"
void fijarReferenciasAqui() {
  rumboObjetivo  = odoRumbo;
  avanceObjetivo = odoAvance;
}

// Devuelve true solo si el sensor lleva viendo negro LONGITUD_MIN_LINEA_CM
bool filtrarNegro(FiltroNegro &f, bool crudo) {
  if (!crudo) { f.viendo = false; return false; }
  if (!f.viendo) {
    f.viendo = true;
    f.desdeAvance  = odoAvance;
    f.desdeLateral = odoLateral;
  }
  float recorridoCm = pulsosACm(fabs(odoAvance - f.desdeAvance) +
                                fabs(odoLateral - f.desdeLateral));
  return recorridoCm >= LONGITUD_MIN_LINEA_CM;
}

// ==========================================
// 8. GESTIÓN DE MODOS Y TRANSICIONES
// ==========================================
const char* nombreEstado(EstadoFSM e) {
  switch (e) {
    case NAVEGAR:              return "NAVEGAR";
    case ESQUIVAR_PISCINA:     return "ESQUIVAR_PISCINA";
    case VERIFICAR_LIBERACION: return "VERIFICAR_LIBERACION";
    case ESCUADRAR:            return "ESCUADRAR";
    case HOMING_MARGEN:        return "HOMING_MARGEN";
    case BARRIDO:              return "BARRIDO";
    case CENTRAR_FILA:         return "CENTRAR_FILA";
    case ALINEAR_Y_CAPTURAR:   return "ALINEAR_Y_CAPTURAR";
    case GIRO_180:             return "GIRO_180";
    case POSICIONAR_DESCARGA:  return "POSICIONAR_DESCARGA";
    case RETROCEDER_DESCARGA:  return "RETROCEDER_DESCARGA";
    case DESCARGA_PAYLOAD:     return "DESCARGA_PAYLOAD";
    case CALIB_AVANCE:         return "CALIB_AVANCE";
    case FIN_MISION:           return "FIN_MISION";
    case ESTADO_ERROR:         return "ESTADO_ERROR";
  }
  return "?";
}

EstadoFSM estadoInicial() {
  switch (MODO) {
    case PRUEBA_SENSORES: return FIN_MISION;
    case PRUEBA_ESQUIVA:  return NAVEGAR;
    case PRUEBA_DEMO:     return NAVEGAR;
    case PRUEBA_GIRO:     return FIN_MISION;   // Quieto hasta 'g' o 'a'
    case PRUEBA_IDA:      return NAVEGAR;
    case PRUEBA_BARRIDO:  return ESCUADRAR;
    case PRUEBA_REGRESO:  return GIRO_180;
    case PRUEBA_RUTINA:   return NAVEGAR;
    case COMPETENCIA:     return NAVEGAR;
  }
  return FIN_MISION;
}

bool esEstadoDeNavegacion(EstadoFSM e) {
  return (e == NAVEGAR || e == ESQUIVAR_PISCINA || e == VERIFICAR_LIBERACION);
}

bool estadoPermitido(EstadoFSM e) {
  if (e == FIN_MISION || e == ESTADO_ERROR) return true;

  switch (MODO) {
    case PRUEBA_SENSORES: return false;
    case PRUEBA_ESQUIVA:  return esEstadoDeNavegacion(e);
    case PRUEBA_DEMO:     return esEstadoDeNavegacion(e) || e == GIRO_180;
    case PRUEBA_GIRO:     return (e == GIRO_180 || e == CALIB_AVANCE);
    case PRUEBA_IDA:      return esEstadoDeNavegacion(e) || e == ESCUADRAR || e == HOMING_MARGEN;
    case PRUEBA_BARRIDO:  return (e == ESCUADRAR || e == HOMING_MARGEN || e == BARRIDO || e == CENTRAR_FILA);
    case PRUEBA_REGRESO:  return esEstadoDeNavegacion(e) || e == GIRO_180 || e == ESCUADRAR ||
                                 e == POSICIONAR_DESCARGA || e == RETROCEDER_DESCARGA ||
                                 e == DESCARGA_PAYLOAD;
    case PRUEBA_RUTINA:
    case COMPETENCIA:     return true;
  }
  return false;
}

// parar = true detiene motores al cambiar; false mantiene el movimiento.
void cambiarEstado(EstadoFSM nuevoEstado, bool parar) {
  if (nuevoEstado == estadoActual) return;

  if (!estadoPermitido(nuevoEstado)) {
    Serial.printf("[PRUEBA] %s no incluido en este modo -> FIN\n", nombreEstado(nuevoEstado));
    nuevoEstado = FIN_MISION;
    parar = true;
  }

  if (parar) pararMotores();

  Serial.printf("[FSM] %s -> %s  (t=%lu ms)\n",
                nombreEstado(estadoActual), nombreEstado(nuevoEstado),
                millis() - tiempoInicioMision);

  estadoActual = nuevoEstado;
  estadoIniciado = false;
  tiempoInicioEstado = millis();
}

// Prepara y lanza un giro de 180° indicando a dónde seguir después
void iniciarGiro180(EstadoFSM siguiente) {
  estadoTrasGiro = siguiente;
  cambiarEstado(GIRO_180, true);
}

// Qué hacer al detectar la línea al frente
void alDetectarLinea() {
  if (MODO == PRUEBA_DEMO) {
    iniciarGiro180(NAVEGAR);
  } else {
    cambiarEstado(ESCUADRAR, true);
  }
}

// ==========================================
// 9. ACTUADORES SECUNDARIOS
// ==========================================
void moverElevadorANivel(int nivel) {
  pararMotores();
  if (!USAR_ELEVADOR) {
    Serial.printf("[ELEVADOR] Nivel %d (simulado)\n", nivel);
    nivelActualH = nivel;
    return;
  }
  if (nivel == nivelActualH) return;

  mcp.digitalWrite(pinDirElevador, (nivel > nivelActualH) ? HIGH : LOW);
  int pasos = abs(nivel - nivelActualH) * 400; // TODO: calcular con el piñón real

  for (int i = 0; i < pasos; i++) {
    mcp.digitalWrite(pinStepElevador, HIGH);
    delayMicroseconds(800);
    mcp.digitalWrite(pinStepElevador, LOW);
    delayMicroseconds(800);
  }
  nivelActualH = nivel;

  while (Serial2.available() > 0) Serial2.read();
  ultimaLecturaCamara = millis();
}

void abrirCompuertaDescarga() {
  if (!USAR_SERVO) {
    Serial.println("[DESCARGA] Compuerta abierta 2 s (simulado)");
    delay(500);
    return;
  }
  servoCompuerta.write(90);
  delay(2000);
  servoCompuerta.write(0);
  while (Serial2.available() > 0) Serial2.read();
}

void ejecutarMecanismoRecolector() {
  delay(250);
  while (Serial2.available() > 0) Serial2.read();
}

// ==========================================
// 10. UART ESP32-CAM
// ==========================================
void parsearTrama(char* trama) {
  int errX, errY, centrado;
  if (sscanf(trama, "%d,%d,%d", &errX, &errY, &centrado) == 3) {
    errorCamaraX = errX;
    errorCamaraY = errY;
    pelotaCentrada = (centrado == 1);
    ultimaLecturaCamara = millis();
  }
}

void procesarCamaraVision() {
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    if (c == '\n') {
      bufferUART[idxBuffer] = '\0';
      parsearTrama(bufferUART);
      idxBuffer = 0;
    } else if (c != '\r') {
      if (idxBuffer < sizeof(bufferUART) - 1) bufferUART[idxBuffer++] = c;
      else idxBuffer = 0;
    }
  }
}

// ==========================================
// 11. DIAGNÓSTICO, MONITOR Y COMANDOS SERIAL
// ==========================================
// Mapeo (Infrarojos.ino): PB0 = frontal der, PB1 = frontal izq,
//                         PB2 = lateral izq, PB3 = lateral der
const unsigned long INTERVALO_DIAGNOSTICO_MS = 200;
unsigned long ultimoDiagnostico = 0;

void imprimirDiagnosticoSensores(int distFrontal, int distLateral) {
  if (millis() - ultimoDiagnostico < INTERVALO_DIAGNOSTICO_MS) return;
  ultimoDiagnostico = millis();

  uint8_t puertoB = mcp.readGPIOB();
  int rawPB0 = (puertoB >> 0) & 1;
  int rawPB1 = (puertoB >> 1) & 1;
  int rawPB2 = (puertoB >> 2) & 1;
  int rawPB3 = (puertoB >> 3) & 1;

  float posLinea = calcularErrorLinea();

  Serial.printf("CRUDO FrIzq(PB1)=%d FrDer(PB0)=%d LatIzq(PB2)=%d LatDer(PB3)=%d",
                rawPB1, rawPB0, rawPB2, rawPB3);
  Serial.printf("  | NEGRO? fr=%d %d lat=%d %d pos=",
                irFrontalIzq(), irFrontalDer(),
                detectoMargenIzquierdo() ? 1 : 0, detectoMargenDerecho() ? 1 : 0);
  if (posLinea >= 999.0f) Serial.print(" ---");
  else                    Serial.printf("%+.2f", posLinea);
  Serial.printf("  | US F=%3d L=%3d | rumbo=%+.1f°\n", distFrontal, distLateral, rumboGrados());
}

// Monitor de los modos en movimiento (cada 300 ms)
void imprimirMonitor(int distFrontal, int distLateral, bool lineaCruda,
                     bool margenIzq, bool margenDer) {
  if (millis() - ultimoMonitor < 300) return;
  ultimoMonitor = millis();
  Serial.printf("[MON] %-20s %s N%d | linea=%d mIzq=%d mDer=%d | US F=%3d L=%3d | rumbo=%+6.1f° av=%+6.1fcm lat=%+6.1fcm\n",
                nombreEstado(estadoActual), (fase == FASE_IDA) ? "IDA  " : "VUELTA",
                nivelBarrido, lineaCruda ? 1 : 0, margenIzq ? 1 : 0, margenDer ? 1 : 0,
                distFrontal, distLateral, rumboGrados(),
                pulsosACm(odoAvance), pulsosACm(odoLateral));
}

// PRUEBA_GIRO:
//   'g' = girar 180°      'a' = avanzar recto PULSOS_CALIBRACION_AVANCE
//   '+'/'-' = +/-100 pulsos de giro    '>'/'<' = +/-20 pulsos de giro
void procesarComandosSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    long antes = pulsosGiro180;
    if (c == '+') pulsosGiro180 += 100;
    if (c == '-') pulsosGiro180 = max(100L, pulsosGiro180 - 100);
    if (c == '>') pulsosGiro180 += 20;
    if (c == '<') pulsosGiro180 = max(100L, pulsosGiro180 - 20);
    if (pulsosGiro180 != antes) Serial.printf(">>> pulsosGiro180 = %ld\n", pulsosGiro180);

    if (MODO == PRUEBA_GIRO && estadoActual == FIN_MISION) {
      if (c == 'g') iniciarGiro180(FIN_MISION);
      if (c == 'a') cambiarEstado(CALIB_AVANCE, true);
    }
  }
}

void autocalibrarTrack() {
  int altos = 0;
  for (int i = 0; i < 10; i++) {
    if (mcp.digitalRead(pinTrackLinea) == HIGH) altos++;
    delay(5);
  }
  int valorBlanco = (altos >= 5) ? HIGH : LOW;
  trackValorNegro = (valorBlanco == HIGH) ? LOW : HIGH;
  Serial.printf("[TRACK] Autocalibrado: blanco=%d, negro=%d (el robot debe arrancar sobre blanco)\n",
                valorBlanco, trackValorNegro);
}

// ==========================================
// 12. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 19);

  Wire.begin(21, 22);
  if (!mcp.begin_I2C()) {
    Serial.println("Error al iniciar MCP23017");
    while (1);
  }

  setupMotores();
  setupUltrasonidos();
  pararMotores();
  iniciarOdometria();

  if (LEER_IR) setupInfrarrojos();

  if (USAR_ELEVADOR) {
    mcp.pinMode(pinStepElevador, OUTPUT);
    mcp.pinMode(pinDirElevador, OUTPUT);
  }

  if (USAR_SERVO) {
    servoCompuerta.attach(pinServoCompuerta);
    servoCompuerta.write(0);
  }

  if (MODO == PRUEBA_DEMO) {
    mcp.pinMode(pinTrackLinea, INPUT_PULLUP);
    delay(20);
    if (AUTO_CALIBRAR_TRACK) autocalibrarTrack();
    else Serial.printf("[TRACK] Valor de negro fijo: %d\n", trackValorNegro);
  }

  // En PRUEBA_REGRESO el robot arranca en la línea del cultivo, ya de vuelta
  if (MODO == PRUEBA_REGRESO) {
    fase = FASE_VUELTA;
    estadoTrasGiro = NAVEGAR;
  }

  if (MODO == PRUEBA_GIRO) {
    estadoActual = FIN_MISION;
    Serial.println("--- CALIBRACIÓN ---");
    Serial.println("'g' = giro 180 | '+'/'-' = +/-100 pulsos | '>'/'<' = +/-20 pulsos");
    Serial.println("'a' = avance recto para calibrar pulsos por cm");
    Serial.printf("pulsosGiro180 actual = %ld\n", pulsosGiro180);
  } else if (MODO == PRUEBA_SENSORES) {
    estadoActual = FIN_MISION;
    Serial.println("--- MODO SENSORES: robot quieto, diagnóstico cada 200 ms ---");
    Serial.println("Esperado: 1 = negro, 0 = blanco. Ultrasonidos en cm (999 = sin eco).");
  } else {
    // Sin botón: arranca sola después de encender el interruptor de control
    Serial.printf("--- Arranque en %lu ms ---\n", RETARDO_ARRANQUE_MS);
    delay(RETARDO_ARRANQUE_MS);
    iniciarOdometria();
    estadoActual = estadoInicial();
    tiempoInicioMision = millis();
    tiempoInicioEstado = millis();
    tiempoArmadoLinea  = millis();
    Serial.printf("--- MODO PRUEBA: arranca en %s ---\n", nombreEstado(estadoActual));
  }
}

// ==========================================
// 13. LOOP PRINCIPAL
// ==========================================
void loop() {
  procesarComandosSerial();
  actualizarControlMotores();
  actualizarLecturasUltrasonido();   // Puede bloquear hasta ~15 ms (pulseIn)
  procesarCamaraVision();
  actualizarOdometria();             // Después del ultrasonido: odometría al día para la línea

  if (millis() - ultimaLecturaCamara > WATCHDOG_CAMARA) {
    pelotaCentrada = false;
    errorCamaraX = 0;
    errorCamaraY = 0;
  }

  // --- Ultrasonidos ---
  int distFrontal = leerUltrasonidoAlto();
  int distLateral = leerUltrasonidoLateral();

  int distParedEfectiva = 999;
  if (distFrontal < 999 && distLateral < 999) distParedEfectiva = min(distFrontal, distLateral);
  else if (distFrontal < 999)                 distParedEfectiva = distFrontal;
  else if (distLateral < 999)                 distParedEfectiva = distLateral;

  // --- Infrarrojos (filtrados por distancia para ignorar puntos del piso) ---
  bool lineaCruda = false, margenIzq = false, margenDer = false;
  if (LEER_IR) {
    lineaCruda = estaSobreLineaFrontal();
    margenIzq  = filtrarNegro(filtroMargenIzq, detectoMargenIzquierdo());
    margenDer  = filtrarNegro(filtroMargenDer, detectoMargenDerecho());
  }
  if (MODO == PRUEBA_DEMO) {
    trackCrudo = mcp.digitalRead(pinTrackLinea);
    lineaCruda = (trackCrudo == trackValorNegro);
  }
  bool lineaFrontal = filtrarNegro(filtroLinea, lineaCruda);
  bool lineaArmada  = (millis() - tiempoArmadoLinea > VENTANA_IGNORAR_LINEA_MS);

  if (MODO == PRUEBA_SENSORES) {
    imprimirDiagnosticoSensores(distFrontal, distLateral);
    return;
  }

  // PRUEBA_GIRO: reporte del giro completo, incluida la inercia tras frenar
  if (finGiroCalibracion != 0 && millis() - finGiroCalibracion > 800) {
    float total = odoRumbo - rumboInicioGiro;
    Serial.printf("   Giro total ya detenido: %.0f pulsos.\n", total);
    Serial.printf("   Si el robot giró A grados reales: PULSOS_GIRO_180 = %.0f * 180 / A\n", total);
    finGiroCalibracion = 0;
  }

  if (MODO != PRUEBA_ESQUIVA && estadoActual != FIN_MISION) {
    imprimirMonitor(distFrontal, distLateral, lineaCruda, margenIzq, margenDer);
  }

  int wRumbo = correccionRumbo();

  // ==========================================
  // MÁQUINA DE ESTADOS
  // ==========================================
  switch (estadoActual) {

    // ---------- NAVEGACIÓN (ida y vuelta) ----------
    case NAVEGAR: {
      // Si un lateral pisa el margen, se aleja de él sin dejar de avanzar
      int vxMargen = 0;
      if (margenIzq && !margenDer) vxMargen =  V_REPELER_MARGEN;
      if (margenDer && !margenIzq) vxMargen = -V_REPELER_MARGEN;
      moverOmni(vxMargen, VELOCIDAD_BASE, wRumbo);

      if ((distFrontal >= 3 && distFrontal < UMBRAL_PISCINA_CM) ||
          (USAR_LATERAL_EN_DETECCION && distLateral >= 3 && distLateral < UMBRAL_PISCINA_CM)) {
        Serial.printf("-> Obstáculo detectado. Frontal: %d cm | Lateral: %d cm. Iniciando esquiva...\n",
                      distFrontal, distLateral);
        direccionEsquiva = 1;      // Por defecto a la derecha
        inicioEsquiva = millis();
        cambiarEstado(ESQUIVAR_PISCINA, true);
        break;
      }
      if (lineaFrontal && lineaArmada) {
        alDetectarLinea();
      }
      break;
    }

    case ESQUIVAR_PISCINA: {
      if (direccionEsquiva > 0) {
        int correccionPD = obtenerCorreccionParedPD(distParedEfectiva);
        moverOmni(VELOCIDAD_BASE, correccionPD, wRumbo);
      } else {
        moverOmni(-VELOCIDAD_BASE, 0, wRumbo); // A la izquierda solo con el frontal
      }

      if (lineaFrontal && lineaArmada) {
        alDetectarLinea();
        break;
      }

      // Robustez (no aplica en PRUEBA_ESQUIVA para conservar la prueba original)
      if (MODO != PRUEBA_ESQUIVA) {
        bool margenAlFrente = (direccionEsquiva > 0) ? margenDer : margenIzq;
        bool sinSalida = (millis() - inicioEsquiva > TIMEOUT_ESQUIVA_MS);
        if (margenAlFrente || sinSalida) {
          direccionEsquiva = -direccionEsquiva;
          inicioEsquiva = millis();
          Serial.printf("-> Esquiva: %s. Cambiando a la %s\n",
                        margenAlFrente ? "margen alcanzado" : "timeout",
                        (direccionEsquiva > 0) ? "derecha" : "izquierda");
          pararMotores();
          break;
        }
      }

      if (distFrontal > UMBRAL_LIBRE_CM && distFrontal < 300) {
        Serial.println("-> Frente despejado. Verificando liberación...");
        cambiarEstado(VERIFICAR_LIBERACION, false);
      }
      break;
    }

    case VERIFICAR_LIBERACION: {
      if (direccionEsquiva > 0) {
        int correccionPD = obtenerCorreccionParedPD(distParedEfectiva);
        moverOmni(VELOCIDAD_BASE, correccionPD, wRumbo);
      } else {
        moverOmni(-VELOCIDAD_BASE, 0, wRumbo);
      }

      if (lineaFrontal && lineaArmada) {
        alDetectarLinea();
        break;
      }
      if (distFrontal >= 3 && distFrontal <= UMBRAL_LIBRE_CM) {
        cambiarEstado(ESQUIVAR_PISCINA, false);
        break;
      }
      if (millis() - tiempoInicioEstado > TIEMPO_CONFIRMAR_LIBRE_MS) {
        Serial.println("-> Obstáculo superado. Reanudando avance...");
        cambiarEstado(NAVEGAR, true);
      }
      break;
    }

    // ---------- ESCUADRADO EN LA LÍNEA ----------
    // Con los 2 IR frontales: gira hasta que ambos vean la línea.
    // W > 0 gira a la izquierda (adelanta el lado derecho).
    case ESCUADRAR: {
      if (!estadoIniciado) {
        estadoIniciado = true;
        contadorEscuadre = 0;
      }
      int izq = irFrontalIzq();
      int der = irFrontalDer();

      if (izq && der) {
        contadorEscuadre++;
        pararMotores();
      } else {
        contadorEscuadre = 0;
        if (izq && !der)       moverOmni(0, 0,  SIGNO_ESCUADRE * W_ESCUADRE); // Adelanta el lado derecho
        else if (!izq && der)  moverOmni(0, 0, -SIGNO_ESCUADRE * W_ESCUADRE); // Adelanta el lado izquierdo
        else                   moverOmni(0, V_LENTO, 0);      // Ninguno: avanza despacio
      }

      bool listo = (contadorEscuadre >= LECTURAS_CONFIRMAR_ESCUADRE);
      bool porTimeout = (millis() - tiempoInicioEstado > TIMEOUT_ESCUADRE_MS);
      if (listo || porTimeout) {
        fijarReferenciasAqui();  // La línea define "recto" y la distancia de trabajo
        Serial.printf("-> Escuadrado %s. Rumbo de referencia = %+.1f°\n",
                      listo ? "OK" : "por TIMEOUT", rumboGrados());
        if (fase == FASE_IDA) cambiarEstado(HOMING_MARGEN, true);
        else                  cambiarEstado(POSICIONAR_DESCARGA, true);
      }
      break;
    }

    // ---------- HOMING AL MARGEN IZQUIERDO ----------
    case HOMING_MARGEN: {
      if (!estadoIniciado) {
        estadoIniciado = true;
        contadorMargen = 0;
      }
      contadorMargen = margenIzq ? contadorMargen + 1 : 0;
      bool listo = (contadorMargen >= LECTURAS_CONFIRMAR_MARGEN);
      bool porTimeout = (millis() - tiempoInicioEstado > TIMEOUT_HOMING_MS);

      if (listo || porTimeout) {
        refLateral = odoLateral;   // Origen lateral: margen izquierdo
        latMargenIzq = odoLateral;
        margenDerMedido = false;
        nivelBarrido = 1;
        direccionBarrido = 1;
        Serial.printf("-> Homing %s. Origen lateral fijado.\n", listo ? "OK" : "por TIMEOUT");
        moverElevadorANivel(1);
        cambiarEstado(BARRIDO, true);
        break;
      }
      moverOmni(-V_HOMING, correccionAvance(), wRumbo);
      break;
    }

    // ---------- BARRIDO FRENTE A LOS ÁRBOLES ----------
    case BARRIDO: {
      if (!estadoIniciado) {
        estadoIniciado = true;
        contadorMargen = 0;
        frenteAArbol = false;
        Serial.printf("-> Barrido nivel %d hacia la %s\n", nivelBarrido,
                      (direccionBarrido > 0) ? "derecha" : "izquierda");
      }

      // Límite de tiempo de la ronda: cortar y regresar
      if (millis() - tiempoInicioMision > TIEMPO_LIMITE_RECOLECCION_MS) {
        Serial.println("-> Tiempo límite de recolección. Regresando.");
        fase = FASE_VUELTA;
        moverElevadorANivel(1);
        iniciarGiro180(NAVEGAR);
        break;
      }

      // Registro de árboles (aquí se integrará la visión: parar y capturar)
      bool arbolAhora = (distFrontal >= 3 && distFrontal < UMBRAL_ARBOL_CM);
      if (arbolAhora && !frenteAArbol) {
        arbolesVistos++;
        Serial.printf("   [ÁRBOL] Inicio frente a árbol (F=%d cm, lat=%.1f cm)\n",
                      distFrontal, pulsosACm(odoLateral - refLateral));
      }
      if (!arbolAhora && frenteAArbol) Serial.println("   [ÁRBOL] Fin del árbol");
      frenteAArbol = arbolAhora;

      bool margenAlFrente = (direccionBarrido > 0) ? margenDer : margenIzq;
      contadorMargen = margenAlFrente ? contadorMargen + 1 : 0;
      bool finDeFila = (contadorMargen >= LECTURAS_CONFIRMAR_MARGEN) ||
                       (millis() - tiempoInicioEstado > TIMEOUT_BARRIDO_NIVEL_MS);

      if (finDeFila) {
        pararMotores();
        if (direccionBarrido > 0 && contadorMargen >= LECTURAS_CONFIRMAR_MARGEN) {
          latMargenDer = odoLateral;
          margenDerMedido = true;
        }
        if (nivelBarrido < NIVELES_BARRIDO) {
          nivelBarrido++;
          direccionBarrido = -direccionBarrido;
          moverElevadorANivel(nivelBarrido);
          estadoIniciado = false;          // Reinicia el nivel dentro del mismo estado
          tiempoInicioEstado = millis();
        } else {
          Serial.println("-> Barrido completo. Centrando en la fila antes de regresar.");
          fase = FASE_VUELTA;
          moverElevadorANivel(1);
          cambiarEstado(CENTRAR_FILA, true);
        }
        break;
      }
      moverOmni(direccionBarrido * V_BARRIDO, correccionAvance(), wRumbo);
      break;
    }

    // ---------- CENTRAR EN LA FILA ANTES DE REGRESAR ----------
    // Evita regresar pegado a un margen (con un pequeño desvío lo pisaría).
    case CENTRAR_FILA: {
      if (!margenDerMedido) {               // Sin dato del margen derecho: no centrar
        iniciarGiro180(NAVEGAR);
        break;
      }
      float objetivoLat = (latMargenIzq + latMargenDer) / 2.0f;
      float faltaCm = pulsosACm(objetivoLat - odoLateral);
      if (fabs(faltaCm) <= TOLERANCIA_CENTRAR_CM ||
          millis() - tiempoInicioEstado > TIMEOUT_CENTRAR_MS) {
        Serial.printf("-> Centrado en la fila (faltaban %.1f cm). Regresando.\n", faltaCm);
        iniciarGiro180(NAVEGAR);
        break;
      }
      int vx = (faltaCm > 0) ? V_BARRIDO : -V_BARRIDO;
      if (fabs(faltaCm) < 10.0f) vx /= 2;    // Más despacio al final
      moverOmni(vx, correccionAvance(), wRumbo);
      break;
    }

    // ---------- VISIÓN (reservado) ----------
    case ALINEAR_Y_CAPTURAR:
      cambiarEstado(BARRIDO, true);
      break;

    // ---------- GIRO 180° CONTROLADO ----------
    case GIRO_180: {
      if (millis() - tiempoInicioEstado < PAUSA_ANTES_GIRO_MS) break; // Deja frenar

      if (!estadoIniciado) {
        estadoIniciado = true;
        rumboInicioGiro = odoRumbo;
        inicioGiro = millis();
        Serial.printf("-> Girando 180° (objetivo %ld pulsos)...\n", pulsosGiro180);
      }

      float objetivo = rumboInicioGiro + (float)pulsosGiro180;
      float faltaGrados = pulsosAGrados(objetivo - odoRumbo);
      bool porTimeout = (millis() - inicioGiro > TIMEOUT_GIRO_MS);

      if (faltaGrados <= TOLERANCIA_GIRO_GRADOS || porTimeout) {
        pararMotores();
        rumboObjetivo  = objetivo;   // El nuevo "recto" es exactamente 180° más
        avanceObjetivo = odoAvance;
        Serial.printf("-> Giro terminado: %.0f pulsos en %lu ms%s\n",
                      odoRumbo - rumboInicioGiro, millis() - inicioGiro,
                      porTimeout ? "  (TIMEOUT: revisar encoders)" : "");
        tiempoArmadoLinea = millis();
        filtroLinea.viendo = false;
        if (MODO == PRUEBA_GIRO) finGiroCalibracion = millis();
        cambiarEstado(estadoTrasGiro, true);
        break;
      }

      // Velocidad proporcional: frena al acercarse al objetivo
      int w = (int)(faltaGrados * 3.0f);
      w = constrain(w, VELOCIDAD_GIRO_MIN, VELOCIDAD_GIRO_MAX);
      moverOmni(0, 0, w);
      break;
    }

    // ---------- POSICIONAMIENTO PARA DESCARGA ----------
    case POSICIONAR_DESCARGA: {
      if (!estadoIniciado) {
        estadoIniciado = true;
        refLateral = odoLateral;
        contadorMargen = 0;
      }
      bool margenLado = (LADO_RECIPIENTE < 0) ? margenIzq : margenDer;
      contadorMargen = margenLado ? contadorMargen + 1 : 0;

      float recorridoCm = fabs(pulsosACm(odoLateral - refLateral));
      bool llegoDistancia = (DESPLAZAMIENTO_DESCARGA_CM > 0.0f &&
                             recorridoCm >= DESPLAZAMIENTO_DESCARGA_CM);
      bool llegoMargen = (contadorMargen >= LECTURAS_CONFIRMAR_MARGEN);
      bool porTimeout  = (millis() - tiempoInicioEstado > TIMEOUT_POSICIONAR_MS);

      if (llegoDistancia || llegoMargen || porTimeout) {
        Serial.printf("-> Posicionado (%s, %.1f cm). Girando para descargar por detrás.\n",
                      llegoMargen ? "margen" : (llegoDistancia ? "distancia" : "TIMEOUT"),
                      recorridoCm);
        iniciarGiro180(RETROCEDER_DESCARGA);
        break;
      }
      moverOmni(LADO_RECIPIENTE * V_HOMING, correccionAvance(), wRumbo);
      break;
    }

    case RETROCEDER_DESCARGA: {
      if (!estadoIniciado) {
        estadoIniciado = true;
        refAvance = odoAvance;
      }
      float retrocedidoCm = fabs(pulsosACm(odoAvance - refAvance));
      bool porTimeout = (millis() - tiempoInicioEstado > TIMEOUT_RETROCESO_MS);
      if (retrocedidoCm >= RETROCESO_DESCARGA_CM || porTimeout) {
        cambiarEstado(DESCARGA_PAYLOAD, true);
        break;
      }
      moverOmni(0, -V_LENTO, wRumbo);
      break;
    }

    case DESCARGA_PAYLOAD:
      pararMotores();
      abrirCompuertaDescarga();
      Serial.printf("-> Rutina completa en %.1f s. Árboles registrados: %d\n",
                    (millis() - tiempoInicioMision) / 1000.0f, arbolesVistos);
      cambiarEstado(FIN_MISION, true);
      break;

    // ---------- CALIBRACIÓN DE DISTANCIA (PRUEBA_GIRO, 'a') ----------
    case CALIB_AVANCE: {
      if (!estadoIniciado) {
        estadoIniciado = true;
        refAvance = odoAvance;
        rumboObjetivo = odoRumbo;
        Serial.printf("-> Avanzando %ld pulsos en línea recta...\n", PULSOS_CALIBRACION_AVANCE);
      }
      float avanzado = odoAvance - refAvance;
      if (avanzado >= PULSOS_CALIBRACION_AVANCE ||
          millis() - tiempoInicioEstado > 8000) {
        pararMotores();
        Serial.printf("-> Avance: %.0f pulsos | desvío de rumbo: %+.1f°\n",
                      avanzado, pulsosAGrados(odoRumbo - rumboObjetivo));
        Serial.println("   Mide la distancia recorrida en cm: pulsosPorCm = pulsos / cm");
        cambiarEstado(FIN_MISION, true);
        break;
      }
      moverOmni(0, VELOCIDAD_BASE, correccionRumbo());
      break;
    }

    case FIN_MISION:
    case ESTADO_ERROR:
      pararMotores();
      break;
  }
}