// ==========================================
// PESTAÑA PRINCIPAL: Robot-Larc.ino
// ==========================================

// 1. DEFINICIÓN DE ESTADOS DE NAVEGACIÓN
enum EstadoNavegacion {
  IR_A_CULTIVO,          // Avanza directo en Vy buscando la línea de la zona de cultivo
  ESQUIVAR_PISCINA_X,    // Desplazamiento lateral puro en Vx para liberar la piscina
  HOMING_ESQUINA,        // Traslación en X hasta tocar la línea lateral perimetral (Punto de origen)
  ESCANEAR_ARBOLES,      // Desplazamiento a lo largo de los árboles usando el ultrasonido alto
  ALINEAR_Y_RECOLECTAR,  // Ajuste fino con datos de la ESP32-CAM y activación del recolector
  REGRESO_TOLVAS         // Retorno directo a la zona de descarga
};

EstadoNavegacion estadoActual = IR_A_CULTIVO;

// 2. VARIABLES GLOBALES DE TIEMPO Y CONTROL (NO BLOQUEANTE)
unsigned long tiempoInicioEstado = 0;
bool estadoIniciado = false;

// Configuración de esquiva (1 = Derecha, -1 = Izquierda)
int direccionEsquiva = 1; 

// Variables globales para alineación con cámara
int errorCamaraX = 0;
int errorCamaraY = 0;
bool pelotaCentrada = false;

// 3. SETUP PRINCIPAL
void setup() {
  Serial.begin(115200);   // Monitor Serie (Debugging)
  Serial2.begin(115200);  // Comunicación UART con la ESP32-CAM (Pines Rx2/Tx2)

  // Inicialización de subsistemas
  setupMotores();
  setupUltrasonidos();
  setupSensoresPiso();
  
  pararMotores();
}

// 4. BUCLE PRINCIPAL (Frecuencia Ultra-Rápida)
void loop() {
  // A. TAREAS GLOBALES DE ALTA FRECUENCIA (Nunca bloquean el loop)
  procesarCamaraVision(); // Lee puerto Serial2 de forma asíncrona

  // B. LECTURA DE SENSORES
  int distFrontalBaja = leerUltrasonidoFrontalBajo();
  int distAlto         = leerUltrasonidoAlto();
  bool lineaPisoFrente = leerSensorPisoFrente();
  bool lineaPisoLateral= leerSensorPisoLateral();

  // C. MÁQUINA DE ESTADOS FINITOS (FSM)
  switch (estadoActual) {

    // -------------------------------------------------------------
    // ESTADO 1: AVANCE DIRECTO A LA ZONA DE CULTIVO
    // -------------------------------------------------------------
    case IR_A_CULTIVO:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      // Condición 1: Se topa con piscina en la Zona Intermedia
      if (distFrontalBaja > 0 && distFrontalBaja < 20) {
        pararMotores();
        estadoIniciado = false;
        estadoActual = ESQUIVAR_PISCINA_X;
        break;
      }

      // Condición 2: Llegó a la línea negra que delimita los árboles
      if (lineaPisoFrente) {
        pararMotores();
        estadoIniciado = false;
        estadoActual = HOMING_ESQUINA;
        break;
      }

      // Acción continua: Avanzar directo hacia el frente
      moverOmni(0, VELOCIDAD_CRUCERO, 0);
      break;

    // -------------------------------------------------------------
    // ESTADO 2: ESQUIVA LATERAL DE PISCINA (TRASLACIÓN EN X)
    // -------------------------------------------------------------
    case ESQUIVAR_PISCINA_X:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      // Condición de salida: El ultrasonido frontal vuelve a ver camino despejado
      if (distFrontalBaja > 35 || distFrontalBaja == 0) {
        // Margen de seguridad de 400ms para sobrepasar la esquina de la piscina
        if (millis() - tiempoInicioEstado > 400) { 
          pararMotores();
          estadoIniciado = false;
          estadoActual = IR_A_CULTIVO; // Reanuda avance en Vy
        }
        break;
      }

      // Acción continua: Desplazamiento de lado sin rotar el chasis
      moverOmni(direccionEsquiva * VELOCIDAD_CRUCERO, 0, 0);
      break;

    // -------------------------------------------------------------
    // ESTADO 3: REUBICACIÓN EN LA ESQUINA (HOMING PERIMETRAL)
    // -------------------------------------------------------------
    case HOMING_ESQUINA:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      // Condición de salida: Sensor IR lateral toca la línea perimetral exterior
      if (lineaPisoLateral) {
        pararMotores();
        resetEncoders(); // Establece el origen X=0, Y=0 de la zona de cultivo
        estadoIniciado = false;
        estadoActual = ESCANEAR_ARBOLES;
        break;
      }

      // Acción continua: Desplazamiento lateral hacia el borde de la pista
      moverOmni(-VELOCIDAD_CRUCERO, 0, 0); 
      break;

    // -------------------------------------------------------------
    // ESTADO 4: ESCANEO DE ÁRBOLES
    // -------------------------------------------------------------
    case ESCANEAR_ARBOLES:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      // Condición de salida: El ultrasonido elevado detecta la estructura/tronco del árbol
      if (distAlto > 0 && distAlto < 25) {
        pararMotores();
        estadoIniciado = false;
        estadoActual = ALINEAR_Y_RECOLECTAR;
        break;
      }

      // Acción continua: Desplazamiento lateral suave barriendo la fila
      moverOmni(120, 0, 0); 
      break;

    // -------------------------------------------------------------
    // ESTADO 5: ALINEACIÓN POR CÁMARA Y COSECHA
    // -------------------------------------------------------------
    case ALINEAR_Y_RECOLECTAR:
      if (!estadoIniciado) {
        tiempoInicioEstado = millis();
        estadoIniciado = true;
      }

      if (pelotaCentrada) {
        pararMotores();
        ejecutarMecanismoRecolector(); // Función que acciona servomotores / garras
        estadoIniciado = false;
        estadoActual = ESCANEAR_ARBOLES; // Sigue al siguiente árbol
      } else {
        // Corrección de posición fina enviada por la ESP32-CAM
        moverOmni(errorCamaraX, errorCamaraY, 0);
      }
      break;

    // -------------------------------------------------------------
    // ESTADO 6: RETORNO A ZONA DE DEPÓSITOS
    // -------------------------------------------------------------
    case REGRESO_TOLVAS:
      moverOmni(0, -VELOCIDAD_CRUCERO, 0);
      break;
  }
}

// 5. RECEPCION DE DATOS DE LA ESP32-CAM (UART ASÍNCRONA)
void procesarCamaraVision() {
  while (Serial2.available() > 0) {
    String datoRecibido = Serial2.readStringUntil('\n');
    datoRecibido.trim();

    // Ejemplo de trama esperada desde ESP32-CAM: "COLOR,ERR_X,ERR_Y,CENTRADOS"
    // Formato sugerido: "ROJO,-12,5,1"
    if (datoRecibido.length() > 0) {
      // Parseo básico de coordenadas...
      // Se actualizan las variables globales: errorCamaraX, errorCamaraY, pelotaCentrada
    }
  }
}