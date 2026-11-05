// ==========================================
// PESTAÑA PRINCIPAL: Robot-Larc.ino
// FSM única + modos de prueba incrementales
// Navegación completa + recolección: ida con esquiva, escuadrado, homing,
// barrido de 3 niveles con captura pelota a pelota, regreso y descarga.
// La decisión de capturar cada pelota se toma en el estado PELOTA_DECIDIR:
// hoy "todas" o manual por Serial; luego la visión de la ESP32-CAM.
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

// Tipo para el filtro de tiempo de los sensores infrarrojos (Infrarojos.ino)
struct FiltroTiempoIR {
  bool enNegroCrudo = false;
  unsigned long inicioNegroMs = 0;
  unsigned long ultimoConfirmadoMs = 0;
  bool confirmado = false;
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
  ARBOL_REFINAR,          // Mide el borde del árbol con precisión (ida y vuelta lenta)
  PELOTA_IR,              // Strafe hasta la posición de la pelota en el árbol
  PELOTA_DECIDIR,         // ¿Se captura? (todas / Serial / cámara)
  PELOTA_ACERCAR,         // Avanza hacia el árbol
  PELOTA_CAPTURAR,        // Secuencia de la garra (y elevador en nivel 3)
  PELOTA_RETIRAR,         // Vuelve a la distancia de la línea
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
  PRUEBA_MECANISMO,     // Robot quieto: elevador, garra y compuerta por Serial
  PRUEBA_CAPTURA,       // Frente a una pelota: decide, acerca, captura, retira ('r' repite)
  PRUEBA_RECOLECCION,   // Frente a la línea del cultivo: barrido de 3 niveles CON capturas
  PRUEBA_RUTINA,        // Rutina completa (con monitor)
  COMPETENCIA           // Rutina completa: arranca sola tras RETARDO_ARRANQUE_MS
};

// >>>>>>>>>> CAMBIAR AQUÍ EL MODO Y VOLVER A SUBIR <<<<<<<<<<
const ModoPrueba MODO = PRUEBA_SENSORES;

// Subsistemas según el modo
const bool LEER_IR = (MODO != PRUEBA_ESQUIVA && MODO != PRUEBA_DEMO && MODO != PRUEBA_GIRO &&
                      MODO != PRUEBA_MECANISMO);

// Actuadores. Con HABILITAR_* = false se simulan por Serial (útil si un
// mecanismo no está montado). En los modos que no los usan no se activan.
const bool HABILITAR_SERVOS   = false;
const bool HABILITAR_ELEVADOR = false;
const bool MODO_CON_MECANISMO = (MODO == PRUEBA_MECANISMO || MODO == PRUEBA_CAPTURA ||
                                 MODO == PRUEBA_RECOLECCION || MODO == PRUEBA_RUTINA ||
                                 MODO == COMPETENCIA);
const bool USAR_SERVO    = HABILITAR_SERVOS   && (MODO_CON_MECANISMO || MODO == PRUEBA_REGRESO);
const bool USAR_ELEVADOR = HABILITAR_ELEVADOR && MODO_CON_MECANISMO;
// Recolectar en el barrido (en PRUEBA_BARRIDO solo se navega)
const bool RECOLECTAR    = false;//(MODO == PRUEBA_RECOLECCION || MODO == PRUEBA_RUTINA || MODO == COMPETENCIA);

// Decisión de captura en cada pelota
enum ModoDecision { DECISION_TODAS, DECISION_SERIAL, DECISION_CAMARA };
const ModoDecision MODO_DECISION = DECISION_TODAS;

EstadoFSM estadoActual  = FIN_MISION;
EstadoFSM estadoTrasGiro = NAVEGAR;   // A dónde ir al terminar GIRO_180
FaseMision fase          = FASE_IDA;

// ==========================================
// 3. PINES Y ACTUADORES ADICIONALES
// ==========================================
// Servo garra: GPIO 19 | Servo compuerta: GPIO 0   (Recoleccion.ino)
// Elevador 28BYJ-48: MCP PB4-PB7                   (Elevador.ino)
// Cámara: RX GPIO 16, TX GPIO 17
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

// --- Línea y salida de zona inicial ---
const unsigned long VENTANA_IGNORAR_LINEA_MS     = 1800; // Tiempo mínimo absoluto al salir/girar
const unsigned long TIEMPO_BLANCO_TRAS_SALIDA_MS = 400;  // Tiempo viendo blanco tras cruzar la 1ra línea
const unsigned long TIEMPO_ARMADO_RESPALDO_MS    = 3500; // Si no vio 1ra línea en este tiempo, arma igual
const int LECTURAS_CONFIRMAR_LINEA               = 2;
const float LONGITUD_MIN_LINEA_CM                = 0.8f;

// --- Control de rumbo (odometría de encoders) ---
const bool  USAR_CONTROL_RUMBO = false;
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
const int V_LENTO                      = 60;
const int W_ESCUADRE                   = 65;   // Velocidad de giro para escuadrar
const int SIGNO_ESCUADRE               = 1;
const int LECTURAS_CONFIRMAR_ESCUADRE  = 2;
const unsigned long PAUSA_FRENO_LINEA_MS = 150; // Deja asentar el frenado antes de corregir
const unsigned long MAX_GIRO_ESCUADRE_MS = 600; // Tope máximo de corrección angular (nunca gira sin fin)
const unsigned long TIMEOUT_ESCUADRE_MS  = 1200;

// --- Homing y barrido ---
const int V_HOMING                   = 110;
const int V_BARRIDO                  = 100;
const float KP_AVANCE                = 0.5f;
const int   MAX_CORRECCION_AVANCE    = 40;
const int LECTURAS_CONFIRMAR_MARGEN  = 2;
const unsigned long VENTANA_INICIO_HOMING_MS = 300; // Ignora lecturas residuales al iniciar el strafe
const unsigned long TIMEOUT_HOMING_MS       = 8000;
const unsigned long TIMEOUT_BARRIDO_NIVEL_MS = 12000;
const unsigned long TIMEOUT_CENTRAR_MS       = 6000;
const float TOLERANCIA_CENTRAR_CM            = 3.0f;
const int V_REPELER_MARGEN                   = 60;
const int NIVELES_BARRIDO            = 3;
const int UMBRAL_ARBOL_CM            = 25;
const unsigned long TIEMPO_LIMITE_RECOLECCION_MS = 150000;

// --- Descarga (AJUSTAR con la geometría real de la cancha) ---
const int   LADO_RECIPIENTE              = -1;   // -1 = izquierda, +1 = derecha
const float DESPLAZAMIENTO_DESCARGA_CM   = 0.0f; // 0 = ir hasta el margen
const unsigned long TIMEOUT_POSICIONAR_MS = 8000;
const float RETROCESO_DESCARGA_CM        = 10.0f;
const unsigned long TIMEOUT_RETROCESO_MS = 4000;

// --- Recolección (MEDIR en la cancha y ajustar) ---
const float DISTANCIA_NIVEL_MM     = 150.0f;           // 390 -> 540 -> 690 mm
const int   PELOTAS_POR_NIVEL[3]   = {5, 6, 5};        // Por árbol (reglamento)
const float ANCHO_ARBOL_CM         = 30.0f;            // MEDIR: ancho que ve el ultrasonido
const float ESPACIO_PELOTAS_CM     = 5.0f;             // MEDIR: separación entre pelotas
const float OFFSET_RAMPA_CM        = 0.0f;             // Rampa respecto al ultrasonido frontal (+ = derecha)
const float TOLERANCIA_PELOTA_CM   = 0.7f;
const int   V_REFINO               = 50;               // Velocidad lenta para medir el borde
const unsigned long TIMEOUT_REFINO_MS = 3000;
const int   V_PELOTA_MAX           = 80;
const int   V_PELOTA_MIN           = 40;
const unsigned long TIMEOUT_PELOTA_IR_MS = 4000;
const float ACERCAMIENTO_CM        = 6.0f;             // MEDIR con PRUEBA_CAPTURA
const int   DIST_MIN_CAPTURA_CM    = 3;                // Freno de seguridad por ultrasonido
const int   V_ACERCAR              = 70;
const unsigned long TIMEOUT_ACERCAR_MS  = 2500;
const unsigned long TIMEOUT_RETIRAR_MS  = 2500;
const float BAJADA_NIVEL3_MM       = 150.0f;           // Probar menos si la pelota no cae lejos
const int   CAPACIDAD_ALMACEN      = 0;                // 0 = sin límite
// Decisión
const unsigned long DECISION_TIMEOUT_MS     = 15000;   // Serial: sin respuesta -> por defecto
const bool          DECISION_POR_DEFECTO    = true;
const unsigned long TIEMPO_ESPERA_CAMARA_MS = 1500;    // Cámara: sin pelota centrada -> no
// PRUEBA_CAPTURA
const int NIVEL_PRUEBA_CAPTURA = 1;                    // 1, 2 o 3

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

struct FiltroNegro {
  bool  viendo = false;
  int   lecturas = 0;
  float desdeAvance = 0.0f;
  float desdeLateral = 0.0f;
};
FiltroNegro filtroLinea, filtroMargenIzq, filtroMargenDer;

// Control de salida de la zona inicial (para ignorar la 1ra línea negra)
// 0 = dentro del recuadro inicial (esperando cruzar 1ra línea)
// 1 = cruzando la 1ra línea negra (esperando volver a piso blanco)
// 2 = fuera del recuadro inicial -> línea del cultivo ARMADA
int etapaSalidaLinea             = 0;
unsigned long ultimoNegroSalida  = 0;

// Variables para escuadrado acotado en la línea frontal
long desfaseFrontalesMs          = 0;
unsigned long duracionGiroEscuadreMs = 0;
int  dirGiroEscuadre             = 0;

int contadorEscuadre  = 0;
int contadorMargen    = 0;
int trackCrudo        = 0;
int direccionEsquiva  = 1;   // +1 derecha, -1 izquierda
int direccionBarrido  = 1;   // +1 derecha, -1 izquierda
int nivelBarrido      = 1;
bool frenteAArbol     = false;
int arbolesVistos     = 0;
int pelotaIdx         = 0;
int pelotasCapturadas = 0;
float bordeArbolLat   = 0.0f;   // Odometría lateral del borde del árbol
float latPerdido      = 0.0f;   // ARBOL_REFINAR: donde se dejó de ver el árbol
int   faseRefino      = 0;
int decisionSerial    = -1;     // -1 = sin respuesta, 0 = no, 1 = sí

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
long diferenciaTiempoFrontalesMs();
void limpiarHistorialLineaFrontal();

void setupElevador();
void elevadorIrAMM(float objetivoMM);
float posicionElevadorMM();
void fijarCeroElevador();
void apagarBobinasElevador();
extern unsigned long retardoPasoUs;

void setupRecoleccion();
void garraA(int angulo);
void compuertaA(int angulo);
void capturaEstandar();
void capturaNivel3Bajar(float bajadaMM);
void descargarCompuerta();

void setupUltrasonidos();
void actualizarLecturasUltrasonido();
int leerUltrasonidoAlto();
int leerUltrasonidoLateral();
int obtenerCorreccionParedPD(int distMedida);

// ==========================================
// 7. ODOMETRÍA Y CONTROL DE RUMBO
// ==========================================
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

int correccionRumbo() {
  if (!controlRumboActivo()) return 0;
  float errGrados = pulsosAGrados(rumboObjetivo - odoRumbo);
  return (int)constrain(KP_RUMBO * errGrados, (float)-W_MAX_RUMBO, (float)W_MAX_RUMBO);
}

int correccionAvance() {
  if (!controlRumboActivo()) return 0;
  float err = avanceObjetivo - odoAvance;
  return (int)constrain(KP_AVANCE * err, (float)-MAX_CORRECCION_AVANCE, (float)MAX_CORRECCION_AVANCE);
}

void fijarReferenciasAqui() {
  rumboObjetivo  = odoRumbo;
  avanceObjetivo = odoAvance;
}

// El filtrado por tiempo continuo (>= 25 ms) ya se realiza en Infrarojos.ino
bool filtrarNegro(FiltroNegro &f, bool crudo) {
  if (!crudo) {
    f.viendo = false;
    f.lecturas = 0;
    return false;
  }
  f.viendo = true;
  if (f.lecturas < 100) f.lecturas++;
  return f.lecturas >= 1;
}

void rearmarSalidaLinea(bool ignorarPrimeraLinea) {
  tiempoArmadoLinea = millis();
  filtroLinea.viendo = false;
  filtroLinea.lecturas = 0;
  limpiarHistorialLineaFrontal();
  etapaSalidaLinea = ignorarPrimeraLinea ? 0 : 2;
  ultimoNegroSalida = millis();
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
    case ARBOL_REFINAR:        return "ARBOL_REFINAR";
    case PELOTA_IR:            return "PELOTA_IR";
    case PELOTA_DECIDIR:       return "PELOTA_DECIDIR";
    case PELOTA_ACERCAR:       return "PELOTA_ACERCAR";
    case PELOTA_CAPTURAR:      return "PELOTA_CAPTURAR";
    case PELOTA_RETIRAR:       return "PELOTA_RETIRAR";
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
    case PRUEBA_MECANISMO:   return FIN_MISION;   // Quieto, comandos por Serial
    case PRUEBA_CAPTURA:     return PELOTA_DECIDIR;
    case PRUEBA_RECOLECCION: return ESCUADRAR;
    case PRUEBA_RUTINA:   return NAVEGAR;
    case COMPETENCIA:     return NAVEGAR;
  }
  return FIN_MISION;
}

bool esEstadoDeNavegacion(EstadoFSM e) {
  return (e == NAVEGAR || e == ESQUIVAR_PISCINA || e == VERIFICAR_LIBERACION);
}

bool esEstadoDePelota(EstadoFSM e) {
  return (e == ARBOL_REFINAR || e == PELOTA_IR || e == PELOTA_DECIDIR || e == PELOTA_ACERCAR ||
          e == PELOTA_CAPTURAR || e == PELOTA_RETIRAR);
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
    case PRUEBA_MECANISMO: return false;
    case PRUEBA_CAPTURA:   return esEstadoDePelota(e);
    case PRUEBA_RECOLECCION:
      return (e == ESCUADRAR || e == HOMING_MARGEN || e == BARRIDO || e == CENTRAR_FILA ||
              esEstadoDePelota(e));
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
  desfaseFrontalesMs = diferenciaTiempoFrontalesMs();
  Serial.printf("-> [LÍNEA] Línea frontal confirmada por AMBOS sensores (desfase=%ld ms)\n",
                desfaseFrontalesMs);
  if (MODO == PRUEBA_DEMO) {
    iniciarGiro180(NAVEGAR);
  } else {
    cambiarEstado(ESCUADRAR, true);
  }
}

// ==========================================
// 9. ACTUADORES SECUNDARIOS
// ==========================================
float alturaNivelMM(int nivel) {
  return (nivel - 1) * DISTANCIA_NIVEL_MM;
}

void moverElevadorANivel(int nivel) {
  pararMotores();
  nivelActualH = nivel;
  if (!USAR_ELEVADOR) {
    Serial.printf("[ELEVADOR] Nivel %d (simulado)\n", nivel);
    return;
  }
  Serial.printf("[ELEVADOR] Moviendo a nivel %d (%.0f mm)...\n", nivel, alturaNivelMM(nivel));
  elevadorIrAMM(alturaNivelMM(nivel));
  while (Serial2.available() > 0) Serial2.read();   // Descarta tramas viejas de la cámara
  ultimaLecturaCamara = millis();
}

void abrirCompuertaDescarga() {
  if (!USAR_SERVO) {
    Serial.println("[DESCARGA] Compuerta abierta 2.5 s (simulado)");
    delay(500);
    return;
  }
  descargarCompuerta();
  while (Serial2.available() > 0) Serial2.read();
}

// Secuencia de captura según el nivel (bloqueante, robot detenido)
void ejecutarCaptura(int nivel) {
  if (!USAR_SERVO) {
    Serial.printf("[CAPTURA] Nivel %d (simulado)\n", nivel);
    delay(nivel < 3 ? 1500 : 3000);
    return;
  }
  if (nivel < 3) {
    capturaEstandar();
  } else {
    if (USAR_ELEVADOR) capturaNivel3Bajar(BAJADA_NIVEL3_MM);
    else               capturaEstandar();
  }
  while (Serial2.available() > 0) Serial2.read();
}

// ---------- Control de la recolección ----------
bool recoleccionSinTiempo() {
  return (MODO != PRUEBA_CAPTURA) &&
         (millis() - tiempoInicioMision > TIEMPO_LIMITE_RECOLECCION_MS);
}

// Corta la recolección: baja el elevador y va a regresar
void terminarRecoleccion(const char* motivo) {
  Serial.printf("-> %s. Centrando en la fila antes de regresar.\n", motivo);
  fase = FASE_VUELTA;
  moverElevadorANivel(1);
  cambiarEstado(CENTRAR_FILA, true);
}

// Prepara un nivel del barrido (se llama al terminar el homing y al cambiar de nivel)
void iniciarNivelBarrido() {
  contadorMargen = 0;
  frenteAArbol = false;
  Serial.printf("-> Barrido nivel %d hacia la %s\n", nivelBarrido,
                (direccionBarrido > 0) ? "derecha" : "izquierda");
}

// Pasa a la siguiente pelota del árbol, o vuelve al barrido si ya terminó
void siguientePelota() {
  pelotaIdx++;
  if (CAPACIDAD_ALMACEN > 0 && pelotasCapturadas >= CAPACIDAD_ALMACEN) {
    terminarRecoleccion("Almacén lleno");
    return;
  }
  if (recoleccionSinTiempo()) {
    terminarRecoleccion("Tiempo límite de recolección");
    return;
  }
  if (pelotaIdx < PELOTAS_POR_NIVEL[nivelBarrido - 1]) {
    cambiarEstado(PELOTA_IR, true);
  } else {
    Serial.printf("   Árbol terminado en el nivel %d\n", nivelBarrido);
    cambiarEstado(BARRIDO, true);   // frenteAArbol sigue en true: no se repite este árbol
  }
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
  Serial.printf("[MON] %-20s %s E%d | linea=%d mIzq=%d | US F=%3d L=%3d\n",
                nombreEstado(estadoActual), (fase == FASE_IDA) ? "IDA  " : "VUELTA",
                etapaSalidaLinea, lineaCruda ? 1 : 0, margenIzq ? 1 : 0,
                distFrontal, distLateral);
}

void imprimirMenuMecanismo() {
  Serial.println("--- PRUEBA MECANISMO (robot quieto) ---");
  Serial.println(" '1' '2' '3' = elevador a nivel | 'u' +10 mm | 'j' -10 mm | 'z' fijar cero aquí");
  Serial.println(" 'f' elevador más rápido | 'l' más lento");
  Serial.println(" 'c' captura estándar | 'e' captura nivel 3 (baja, suelta y vuelve a subir)");
  Serial.println(" 'a' abrir y cerrar compuerta");
}

void comandoMecanismo(char c) {
  bool movio = true;
  if (c == '1' || c == '2' || c == '3') {
    moverElevadorANivel(c - '0');
  } else if (c == 'u' || c == 'j') {
    if (USAR_ELEVADOR) elevadorIrAMM(posicionElevadorMM() + ((c == 'u') ? 10.0f : -10.0f));
  } else if (c == 'z') {
    fijarCeroElevador();
    Serial.println(">>> Posición actual fijada como 0 mm (nivel 1)");
  } else if (c == 'f') {
    retardoPasoUs = max(600UL, retardoPasoUs - 100);
  } else if (c == 'l') {
    retardoPasoUs += 100;
  } else if (c == 'c') {
    ejecutarCaptura(1);
  } else if (c == 'e') {
    float altura = posicionElevadorMM();
    ejecutarCaptura(3);
    if (USAR_ELEVADOR) elevadorIrAMM(altura);
  } else if (c == 'a') {
    abrirCompuertaDescarga();
  } else {
    movio = false;
  }
  if (movio) Serial.printf(">>> Elevador: %.1f mm | retardo de paso: %lu us\n",
                           posicionElevadorMM(), retardoPasoUs);
}

void procesarComandosSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (MODO == PRUEBA_GIRO) {
      long antes = pulsosGiro180;
      if (c == '+') pulsosGiro180 += 100;
      if (c == '-') pulsosGiro180 = max(100L, pulsosGiro180 - 100);
      if (c == '>') pulsosGiro180 += 20;
      if (c == '<') pulsosGiro180 = max(100L, pulsosGiro180 - 20);
      if (pulsosGiro180 != antes) Serial.printf(">>> pulsosGiro180 = %ld\n", pulsosGiro180);
      if (estadoActual == FIN_MISION) {
        if (c == 'g') iniciarGiro180(FIN_MISION);
        if (c == 'a') cambiarEstado(CALIB_AVANCE, true);
      }
    }

    if (MODO == PRUEBA_MECANISMO) comandoMecanismo(c);

    if (MODO == PRUEBA_CAPTURA && estadoActual == FIN_MISION && c == 'r') {
      pelotaIdx = 0;
      cambiarEstado(PELOTA_DECIDIR, true);
    }

    if (estadoActual == PELOTA_DECIDIR && MODO_DECISION == DECISION_SERIAL) {
      if (c == 's') decisionSerial = 1;
      if (c == 'n') decisionSerial = 0;
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
  Serial2.begin(115200, SERIAL_8N1, 16, 17);   // Cámara: RX 16, TX 17

  Wire.begin(21, 22);
  Wire.setClock(400000);   // I2C rápido (probado por el equipo): motores y elevador
  if (!mcp.begin_I2C()) {
    Serial.println("Error al iniciar MCP23017");
    while (1);
  }

  setupMotores();
  setupUltrasonidos();
  pararMotores();
  iniciarOdometria();

  if (LEER_IR) setupInfrarrojos();

  setupElevador();
  if (USAR_SERVO) setupRecoleccion();

  if (MODO == PRUEBA_DEMO) {
    mcp.pinMode(pinTrackLinea, INPUT_PULLUP);
    delay(20);
    if (AUTO_CALIBRAR_TRACK) autocalibrarTrack();
    else Serial.printf("[TRACK] Valor de negro fijo: %d\n", trackValorNegro);
  }

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
  } else if (MODO == PRUEBA_MECANISMO) {
    estadoActual = FIN_MISION;
    imprimirMenuMecanismo();
    Serial.println("El elevador debe estar ABAJO al encender (0 mm = nivel 1).");
  } else if (MODO == PRUEBA_SENSORES) {
    estadoActual = FIN_MISION;
    Serial.println("--- MODO SENSORES: robot quieto, diagnóstico cada 200 ms ---");
    Serial.println("Esperado: 1 = negro, 0 = blanco. Ultrasonidos en cm (999 = sin eco).");
  } else {
    if (MODO == PRUEBA_CAPTURA) {
      nivelBarrido = NIVEL_PRUEBA_CAPTURA;
      moverElevadorANivel(NIVEL_PRUEBA_CAPTURA);
      Serial.println("--- PRUEBA CAPTURA: al terminar envía 'r' para repetir ---");
    }
    Serial.printf("--- Arranque en %lu ms ---\n", RETARDO_ARRANQUE_MS);
    delay(RETARDO_ARRANQUE_MS);
    iniciarOdometria();
    estadoActual = estadoInicial();
    tiempoInicioMision = millis();
    tiempoInicioEstado = millis();
    bool ignorarPrimera = (estadoActual == NAVEGAR && fase == FASE_IDA);
    rearmarSalidaLinea(ignorarPrimera);
    Serial.printf("--- MODO PRUEBA: arranca en %s (etapaSalidaLinea=%d) ---\n",
                  nombreEstado(estadoActual), etapaSalidaLinea);
  }
}

// ==========================================
// 13. LOOP PRINCIPAL
// ==========================================
void loop() {
  procesarComandosSerial();
  actualizarControlMotores();

  // Durante ESCUADRAR y HOMING_MARGEN no se dispara el ultrasonido (pulseIn bloquea
  // hasta 15 ms) para que la lectura de los IR por I2C corra a máxima velocidad (>500 Hz).
  bool usarUltrasonidoAhora = (estadoActual != ESCUADRAR && estadoActual != HOMING_MARGEN);
  if (usarUltrasonidoAhora || MODO == PRUEBA_SENSORES) {
    actualizarLecturasUltrasonido();
  }

  procesarCamaraVision();
  actualizarOdometria();

  if (millis() - ultimaLecturaCamara > WATCHDOG_CAMARA) {
    pelotaCentrada = false;
    errorCamaraX = 0;
    errorCamaraY = 0;
  }

  // --- Ultrasonidos ---
  int distFrontal = usarUltrasonidoAhora ? leerUltrasonidoAlto()    : 999;
  int distLateral = usarUltrasonidoAhora ? leerUltrasonidoLateral() : 999;

  int distParedEfectiva = 999;
  if (distFrontal < 999 && distLateral < 999) distParedEfectiva = min(distFrontal, distLateral);
  else if (distFrontal < 999)                 distParedEfectiva = distFrontal;
  else if (distLateral < 999)                 distParedEfectiva = distLateral;

  // --- Infrarrojos (filtrados por tiempo continuo >= 25 ms y coincidencia de ambos frontales) ---
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

  // --- Gestión de salida del recuadro inicial (ignorar 1ra línea negra) ---
  if (esEstadoDeNavegacion(estadoActual)) {
    unsigned long tDesdeArmado = millis() - tiempoArmadoLinea;
    // Alguno de los frontales filtrado (>= 25 ms) ve negro
    bool algunFrontalFiltrado = (irFrontalIzq() == 1 || irFrontalDer() == 1);

    if (etapaSalidaLinea == 0) {
      if (lineaFrontal) {
        etapaSalidaLinea = 1;
        ultimoNegroSalida = millis();
        Serial.println("-> [SALIDA] 1ra línea negra (recuadro inicial) detectada. Ignorando...");
      } else if (tDesdeArmado > TIEMPO_ARMADO_RESPALDO_MS || estadoActual == ESQUIVAR_PISCINA) {
        etapaSalidaLinea = 2;
        limpiarHistorialLineaFrontal();
        Serial.println("-> [SALIDA] Zona inicial superada. Detección de línea de cultivo ARMADA.");
      }
    } else if (etapaSalidaLinea == 1) {
      if (algunFrontalFiltrado) {
        ultimoNegroSalida = millis();
      } else if ((millis() - ultimoNegroSalida > TIEMPO_BLANCO_TRAS_SALIDA_MS) &&
                 (tDesdeArmado > VENTANA_IGNORAR_LINEA_MS)) {
        etapaSalidaLinea = 2;
        limpiarHistorialLineaFrontal();
        filtroLinea.viendo = false;
        filtroLinea.lecturas = 0;
        lineaFrontal = false;
        Serial.println("-> [SALIDA] 1ra línea negra superada. Detección de línea de cultivo ARMADA.");
      }
    }
  }
  bool lineaArmada = (etapaSalidaLinea >= 2) &&
                     (millis() - tiempoArmadoLinea > VENTANA_IGNORAR_LINEA_MS);

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
      moverOmni(0, VELOCIDAD_BASE, wRumbo);

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
      if (MODO != PRUEBA_ESQUIVA && lineaArmada) {
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
    // Como estaSobreLineaFrontal() exige que AMBOS sensores frontales hayan tocado la línea,
    // aquí sabemos exactamente cuántos ms tardó un sensor respecto al otro (desfaseFrontalesMs).
    // Si llegaron casi al tiempo (<= 70 ms), ya está escuadrado y pasa directo al homing.
    // Si llegó inclinado, hace un pulso de giro proporcional y acotado (máx 600 ms) sin quedarse girando.
    case ESCUADRAR: {
      if (!estadoIniciado) {
        estadoIniciado = true;
        long absDesfase = labs(desfaseFrontalesMs);
        if (absDesfase <= 70) {
          dirGiroEscuadre = 0;
          duracionGiroEscuadreMs = 0;
        } else {
          // desfaseFrontalesMs > 0: izquierdo vio primero -> adelantar lado derecho (+SIGNO_ESCUADRE)
          dirGiroEscuadre = (desfaseFrontalesMs > 0) ? SIGNO_ESCUADRE : -SIGNO_ESCUADRE;
          duracionGiroEscuadreMs = constrain((unsigned long)(absDesfase * 1.3f), 120UL, MAX_GIRO_ESCUADRE_MS);
        }
        Serial.printf("   [ESCUADRAR] desfase=%ld ms -> giro correctivo=%lu ms (dir=%d)\n",
                      desfaseFrontalesMs, duracionGiroEscuadreMs, dirGiroEscuadre);
      }

      unsigned long tEnEstado = millis() - tiempoInicioEstado;

      // 1. Pausa breve para asentar la inercia del frenado
      if (tEnEstado < PAUSA_FRENO_LINEA_MS) {
        pararMotores();
        break;
      }

      // 2. Pulso acotado de corrección angular (solo si entró inclinado y aún no están ambos en 1)
      unsigned long tGiro = tEnEstado - PAUSA_FRENO_LINEA_MS;
      bool ambosEnNegroAhora = (irFrontalIzq() == 1 && irFrontalDer() == 1);

      if (tGiro < duracionGiroEscuadreMs && !ambosEnNegroAhora) {
        moverOmni(0, 0, dirGiroEscuadre * W_ESCUADRE);
        break;
      }

      // 3. Listo: frena y pasa a HOMING_MARGEN
      pararMotores();
      delay(150);
      fijarReferenciasAqui();
      Serial.println("-> Escuadrado completado. Iniciando homing al margen izquierdo.");
      if (fase == FASE_IDA) cambiarEstado(HOMING_MARGEN, true);
      else                  cambiarEstado(POSICIONAR_DESCARGA, true);
      break;
    }

    // ---------- HOMING AL MARGEN IZQUIERDO ----------
    case HOMING_MARGEN: {
      if (!estadoIniciado) {
        estadoIniciado = true;
        contadorMargen = 0;
      }

      unsigned long tEnHoming = millis() - tiempoInicioEstado;
      bool vioMargen = false;
      if (tEnHoming > VENTANA_INICIO_HOMING_MS) {
        vioMargen = detectoMargenIzquierdo();
      }

      contadorMargen = vioMargen ? contadorMargen + 1 : 0;
      bool listo = (contadorMargen >= LECTURAS_CONFIRMAR_MARGEN);
      bool porTimeout = (tEnHoming > TIMEOUT_HOMING_MS);

      if (listo || porTimeout) {
        pararMotores();
        refLateral = odoLateral;   // Origen lateral: margen izquierdo
        latMargenIzq = odoLateral;
        margenDerMedido = false;
        nivelBarrido = 1;
        direccionBarrido = 1;
        Serial.printf("-> Homing %s. Encuadre finalizado en margen izquierdo.\n",
                      listo ? "OK" : "por TIMEOUT");

        if (!estadoPermitido(BARRIDO)) {
          cambiarEstado(FIN_MISION, true);
          break;
        }
        moverElevadorANivel(1);
        iniciarNivelBarrido();
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
      }

      if (recoleccionSinTiempo()) {
        terminarRecoleccion("Tiempo límite de recolección");
        break;
      }

      bool arbolAhora = (distFrontal >= 3 && distFrontal < UMBRAL_ARBOL_CM);
      if (arbolAhora && !frenteAArbol) {
        frenteAArbol = true;
        arbolesVistos++;
        Serial.printf("   [ÁRBOL] Inicio frente a árbol (F=%d cm, lat=%.1f cm)\n",
                      distFrontal, pulsosACm(odoLateral - refLateral));
        if (RECOLECTAR) {
          cambiarEstado(ARBOL_REFINAR, true);
          break;
        }
      }
      if (!arbolAhora && frenteAArbol) {
        frenteAArbol = false;
        Serial.println("   [ÁRBOL] Fin del árbol");
      }

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
          iniciarNivelBarrido();
          estadoIniciado = false;
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
    case CENTRAR_FILA: {
      if (!margenDerMedido) {
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
      if (fabs(faltaCm) < 10.0f) vx /= 2;
      moverOmni(vx, correccionAvance(), wRumbo);
      break;
    }

    // ---------- RECOLECCIÓN: BORDE PRECISO DEL ÁRBOL ----------
    case ARBOL_REFINAR: {
      if (!estadoIniciado) {
        estadoIniciado = true;
        faseRefino = 0;
      }
      bool arbolVisto = (distFrontal >= 3 && distFrontal < UMBRAL_ARBOL_CM);

      if (millis() - tiempoInicioEstado > TIMEOUT_REFINO_MS) {
        bordeArbolLat = odoLateral;
        pelotaIdx = 0;
        Serial.println("   [ÁRBOL] Refinado por TIMEOUT: se usa la posición actual");
        cambiarEstado(PELOTA_IR, true);
        break;
      }
      if (faseRefino == 0) {
        if (!arbolVisto) { latPerdido = odoLateral; faseRefino = 1; }
        else moverOmni(-direccionBarrido * V_REFINO, correccionAvance(), wRumbo);
      }
      if (faseRefino == 1) {
        if (arbolVisto) {
          bordeArbolLat = (latPerdido + odoLateral) / 2.0f;
          pelotaIdx = 0;
          cambiarEstado(PELOTA_IR, true);
          break;
        }
        moverOmni(direccionBarrido * V_REFINO, correccionAvance(), wRumbo);
      }
      break;
    }

    // ---------- RECOLECCIÓN: PELOTA A PELOTA ----------
    case PELOTA_IR: {
      if (recoleccionSinTiempo()) {
        terminarRecoleccion("Tiempo límite de recolección");
        break;
      }
      int k = PELOTAS_POR_NIVEL[nivelBarrido - 1];
      float offsetPrimeraCm = (ANCHO_ARBOL_CM - (k - 1) * ESPACIO_PELOTAS_CM) / 2.0f;
      float posCm = offsetPrimeraCm + pelotaIdx * ESPACIO_PELOTAS_CM;
      float objetivo = bordeArbolLat + cmAPulsos(direccionBarrido * posCm - OFFSET_RAMPA_CM);
      float faltaCm = pulsosACm(objetivo - odoLateral);

      if (fabs(faltaCm) <= TOLERANCIA_PELOTA_CM ||
          millis() - tiempoInicioEstado > TIMEOUT_PELOTA_IR_MS) {
        cambiarEstado(PELOTA_DECIDIR, true);
        break;
      }
      int vx = (int)constrain(faltaCm * 8.0f, (float)-V_PELOTA_MAX, (float)V_PELOTA_MAX);
      if (abs(vx) < V_PELOTA_MIN) vx = (faltaCm > 0) ? V_PELOTA_MIN : -V_PELOTA_MIN;
      moverOmni(vx, correccionAvance(), wRumbo);
      break;
    }

    case PELOTA_DECIDIR: {
      int k = PELOTAS_POR_NIVEL[nivelBarrido - 1];
      if (!estadoIniciado) {
        estadoIniciado = true;
        pararMotores();
        decisionSerial = -1;
        if (MODO_DECISION == DECISION_SERIAL) {
          Serial.printf("?? Nivel %d, pelota %d de %d: ¿capturar? (s/n)\n",
                        nivelBarrido, pelotaIdx + 1, k);
        }
      }

      int decision = -1;
      switch (MODO_DECISION) {
        case DECISION_TODAS:
          decision = 1;
          break;
        case DECISION_SERIAL:
          if (decisionSerial >= 0) decision = decisionSerial;
          else if (millis() - tiempoInicioEstado > DECISION_TIMEOUT_MS) {
            decision = DECISION_POR_DEFECTO ? 1 : 0;
            Serial.println("   Sin respuesta: se usa la decisión por defecto");
          }
          break;
        case DECISION_CAMARA:
          if (pelotaCentrada) decision = 1;
          else if (millis() - tiempoInicioEstado > TIEMPO_ESPERA_CAMARA_MS) decision = 0;
          break;
      }
      if (decision < 0) break;

      if (decision == 1) {
        cambiarEstado(PELOTA_ACERCAR, true);
      } else {
        Serial.printf("   Pelota %d del nivel %d omitida\n", pelotaIdx + 1, nivelBarrido);
        if (MODO == PRUEBA_CAPTURA) cambiarEstado(FIN_MISION, true);
        else                        siguientePelota();
      }
      break;
    }

    case PELOTA_ACERCAR: {
      if (!estadoIniciado) {
        estadoIniciado = true;
        refAvance = odoAvance;
      }
      float avanzadoCm = pulsosACm(odoAvance - refAvance);
      bool muyCerca = (distFrontal >= 2 && distFrontal <= DIST_MIN_CAPTURA_CM);
      if (avanzadoCm >= ACERCAMIENTO_CM || muyCerca ||
          millis() - tiempoInicioEstado > TIMEOUT_ACERCAR_MS) {
        cambiarEstado(PELOTA_CAPTURAR, true);
        break;
      }
      moverOmni(0, V_ACERCAR, wRumbo);
      break;
    }

    case PELOTA_CAPTURAR:
      pararMotores();
      Serial.printf("-> Capturando pelota %d del nivel %d\n", pelotaIdx + 1, nivelBarrido);
      ejecutarCaptura(nivelBarrido);
      pelotasCapturadas++;
      cambiarEstado(PELOTA_RETIRAR, true);
      break;

    case PELOTA_RETIRAR: {
      float faltaCm = pulsosACm(avanceObjetivo - odoAvance);
      if (faltaCm >= -0.5f || millis() - tiempoInicioEstado > TIMEOUT_RETIRAR_MS) {
        pararMotores();
        if (nivelBarrido == 3 && USAR_ELEVADOR) {
          elevadorIrAMM(alturaNivelMM(3));
        }
        if (MODO == PRUEBA_CAPTURA) {
          Serial.println("-> Captura terminada. Envía 'r' para repetir.");
          cambiarEstado(FIN_MISION, true);
        } else {
          siguientePelota();
        }
        break;
      }
      moverOmni(0, -V_ACERCAR, wRumbo);
      break;
    }

    // ---------- GIRO 180° CONTROLADO ----------
    case GIRO_180: {
      if (millis() - tiempoInicioEstado < PAUSA_ANTES_GIRO_MS) break;

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
        rumboObjetivo  = objetivo;
        avanceObjetivo = odoAvance;
        Serial.printf("-> Giro terminado: %.0f pulsos en %lu ms%s\n",
                      odoRumbo - rumboInicioGiro, millis() - inicioGiro,
                      porTimeout ? "  (TIMEOUT: revisar encoders)" : "");
        rearmarSalidaLinea(false);
        if (MODO == PRUEBA_GIRO) finGiroCalibracion = millis();
        cambiarEstado(estadoTrasGiro, true);
        break;
      }

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
      Serial.printf("-> Rutina completa en %.1f s. Pelotas capturadas: %d\n",
                    (millis() - tiempoInicioMision) / 1000.0f, pelotasCapturadas);
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
