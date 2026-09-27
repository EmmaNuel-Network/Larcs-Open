enum EstadoNavegacion {
  SEGUIDOR_LINEA,
  GIRO_DESVIO,
  SEGUIDOR_PARED,
  SOBREPASO,
  BUSQUEDA_LINEA
};

// Prototipos externos
int calcularErrorLinea();
bool sensorIR_detectaLinea();
void resetControlPD(); // Prototipo de la función agregada en Motores.ino

EstadoNavegacion estadoActual = SEGUIDOR_LINEA;

const int trigFrenteBajo = A0;
const int echoFrenteBajo = A1;
const int trigFrenteAlto = A2;
const int echoFrenteAlto = A3;
const int trigLateral    = A4;
const int echoLateral    = A5;

const int PULSOS_GIRO_90 = 250; 

// Prototipos externos
int calcularErrorLinea();
bool sensorIR_detectaLinea();

void setup() {
  Serial.begin(115200);

  pinMode(trigFrenteBajo, OUTPUT);
  pinMode(echoFrenteBajo, INPUT);
  pinMode(trigFrenteAlto, OUTPUT);
  pinMode(echoFrenteAlto, INPUT);
  pinMode(trigLateral, OUTPUT);
  pinMode(echoLateral, INPUT);

  setupMotores();
}

void loop() {
  switch (estadoActual) {

    case SEGUIDOR_LINEA: {
      aplicarControlPD(calcularErrorLinea()); 

      int distFrente = leerDistancia(trigFrenteBajo, echoFrenteBajo);
      if (distFrente < 18) {
        pararMotores();
        resetControlPD(); // Preparemos el PD para cuando vuelva a usarse
        estadoActual = GIRO_DESVIO;
      }
      break;
    }

    case GIRO_DESVIO:
      girarDerechaPorPulsos(PULSOS_GIRO_90);
      pararMotores();
      resetControlPD(); // <-- IMPORTANTE: Prepara el PD para el inicio de la pared
      estadoActual = SEGUIDOR_PARED;
      break;

    case SEGUIDOR_PARED: {
      int distLateral = leerDistancia(trigLateral, echoLateral);

      aplicarControlPD(calcularErrorPared(distLateral));

      if (distLateral > 35 && distLateral != 999) {
        resetControlPD(); // Prepara el PD para después de la evasión
        estadoActual = SOBREPASO;
      }
      break;
    }

    case SOBREPASO:
      avanzar();
      delay(350); 
      girarIzquierdaPorPulsos(PULSOS_GIRO_90);
      resetControlPD(); // Prepara el PD para engancharse a la línea
      estadoActual = BUSQUEDA_LINEA;
      break;

    case BUSQUEDA_LINEA:
      avanzar();
      if (sensorIR_detectaLinea()) {
        resetControlPD(); // <-- IMPORTANTE: Suaviza el enganche a la línea
        estadoActual = SEGUIDOR_LINEA;
      }
      break;
  }
}