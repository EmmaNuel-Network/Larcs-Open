// ==========================================
// PESTAÑA: Motores.ino (Arduino UNO)
// ==========================================

// 1. ASIGNACIÓN DE PINES (Respetando restricciones de Arduino UNO)
const int pinEncoderIzqA = 2; // INT0 (Obligatorio Interrupción)
const int pinEncoderIzqB = 4;
const int pinEncoderDerA = 3; // INT1 (Obligatorio Interrupción)
const int pinEncoderDerB = 7;

const int pinIN1 = 8;
const int pinIN2 = 9;
const int pinENA = 5; // Salida PWM

const int pinIN3 = 10;
const int pinIN4 = 11;
const int pinENB = 6; // Salida PWM

// 2. VARIABLES GLOBALES DE MOTORES Y ENCODERS
volatile long pulsosIzquierda = 0;
volatile long pulsosDerecha = 0;

const int VELOCIDAD_CRUCERO = 170; // Rango de 0 a 255
const int VELOCIDAD_GIRO = 150;

// Parámetros del Control PD Genérico
float kp = 4.5;
float kd = 1.8;
int errorAnterior = 0;
bool primerCicloPD = true; // <-- Novedad: Bandera para transferencia suave (Bumpless Transfer)

// 3. RUTINAS DE INTERRUPCIÓN (ISR)
void contarPulsosIzq() {
  if (digitalRead(pinEncoderIzqB) == HIGH) {
    pulsosIzquierda++;
  } else {
    pulsosIzquierda--;
  }
}

void contarPulsosDer() {
  if (digitalRead(pinEncoderDerB) == HIGH) {
    pulsosDerecha++;
  } else {
    pulsosDerecha--;
  }
}

// 4. INICIALIZACIÓN
void setupMotores() {
  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
  pinMode(pinENA, OUTPUT);
  pinMode(pinIN3, OUTPUT);
  pinMode(pinIN4, OUTPUT);
  pinMode(pinENB, OUTPUT);

  pinMode(pinEncoderIzqA, INPUT);
  pinMode(pinEncoderIzqB, INPUT);
  pinMode(pinEncoderDerA, INPUT);
  pinMode(pinEncoderDerB, INPUT);

  // Interrupciones registradas en Pines 2 y 3
  attachInterrupt(digitalPinToInterrupt(pinEncoderIzqA), contarPulsosIzq, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncoderDerA), contarPulsosDer, RISING);

  pararMotores();
}

void resetEncoders() {
  pulsosIzquierda = 0;
  pulsosDerecha = 0;
}

// 5. FUNCIONES DE MOVIMIENTO BÁSICO
void pararMotores() {
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, LOW);
  digitalWrite(pinIN3, LOW);
  digitalWrite(pinIN4, LOW);
  analogWrite(pinENA, 0);
  analogWrite(pinENB, 0);
}

void avanzar() {
  digitalWrite(pinIN1, HIGH);
  digitalWrite(pinIN2, LOW);
  digitalWrite(pinIN3, HIGH);
  digitalWrite(pinIN4, LOW);
  analogWrite(pinENA, VELOCIDAD_CRUCERO);
  analogWrite(pinENB, VELOCIDAD_CRUCERO);
}

// 6. GIROS DE PRECISIÓN POR ENCODER
void girarDerechaPorPulsos(int pulsosObjetivo) {
  resetEncoders();
  digitalWrite(pinIN1, HIGH);
  digitalWrite(pinIN2, LOW);
  digitalWrite(pinIN3, LOW);
  digitalWrite(pinIN4, HIGH);
  analogWrite(pinENA, VELOCIDAD_GIRO);
  analogWrite(pinENB, VELOCIDAD_GIRO);

  unsigned long tiempoInicio = millis();
  // Sale por pulsos O por timeout defensivo de 2.5 segundos
  while (abs(pulsosIzquierda) < pulsosObjetivo && (millis() - tiempoInicio < 2500)) {
    // Espera no bloqueante permanente
  }
  pararMotores();
}

void girarIzquierdaPorPulsos(int pulsosObjetivo) {
  resetEncoders();
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, HIGH);
  digitalWrite(pinIN3, HIGH);
  digitalWrite(pinIN4, LOW);
  analogWrite(pinENA, VELOCIDAD_GIRO);
  analogWrite(pinENB, VELOCIDAD_GIRO);

  unsigned long tiempoInicio = millis();
  // Sale por alcanzado de pulsos O por timeout defensivo de 2.5 segundos
  while (abs(pulsosDerecha) < pulsosObjetivo && (millis() - tiempoInicio < 2500)) {
    // Espera con tiempo límite de seguridad
  }
  pararMotores();
}

// Función para reiniciar el PD cuando cambiamos de estado en la FSM
void resetControlPD() {
  primerCicloPD = true;
}


// 7. CONTROL PD APLICADO A LOS MOTORES
void aplicarControlPD(int error) {

  // Si es el primer ciclo tras una transición, igualamos para que la derivada sea 0
  if (primerCicloPD) {
    errorAnterior = error;
    primerCicloPD = false;
  }
  
  int derivada = error - errorAnterior;
  errorAnterior = error;

  int correccion = (kp * error) + (kd * derivada);

  int velIzq = constrain(VELOCIDAD_CRUCERO + correccion, 80, 230);
  int velDer = constrain(VELOCIDAD_CRUCERO - correccion, 80, 230);

  digitalWrite(pinIN1, HIGH);
  digitalWrite(pinIN2, LOW);
  digitalWrite(pinIN3, HIGH);
  digitalWrite(pinIN4, LOW);

  analogWrite(pinENA, velIzq);
  analogWrite(pinENB, velDer);
}