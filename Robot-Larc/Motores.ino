// ==========================================
// PESTAÑA: Motores.ino (Refactorizado ESP32)
// ==========================================

// 1. ASIGNACIÓN DE PINES EN ESP32 (Modificables según tu PCB/shield)
// Motor 1: Frontal Izquierdo
const int pinM1_IN1 = 12;
const int pinM1_IN2 = 13;
const int pinM1_PWM = 14;
const int pinEncM1A = 34; // Entrada interrupción
const int pinEncM1B = 35; // Canal B cuadratura

// Motor 2: Frontal Derecho
const int pinM2_IN1 = 25;
const int pinM2_IN2 = 26;
const int pinM2_PWM = 27;
const int pinEncM2A = 36;
const int pinEncM2B = 39;

// Motor 3: Trasero Izquierdo
const int pinM3_IN1 = 16;
const int pinM3_IN2 = 17;
const int pinM3_PWM = 4;
const int pinEncM3A = 32;
const int pinEncM3B = 33;

// Motor 4: Trasero Derecho
const int pinM4_IN1 = 18;
const int pinM4_IN2 = 19;
const int pinM4_PWM = 21;
const int pinEncM4A = 22;
const int pinEncM4B = 23;

// 2. VARIABLES GLOBALES DE ENCODERS (volatile para manejo seguro en ISR)
volatile long pulsosM1 = 0;
volatile long pulsosM2 = 0;
volatile long pulsosM3 = 0;
volatile long pulsosM4 = 0;

// Constantes de velocidad por defecto
const int VELOCIDAD_CRUCERO = 180; // Escala 0-255
const int VELOCIDAD_GIRO    = 150;

// 3. RUTINAS DE INTERRUPCIÓN (ISR) EN CUADRATURA
// En ESP32 las ISR deben llevar el atributo IRAM_ATTR
void IRAM_ATTR contarM1() {
  if (digitalRead(pinEncM1B) == HIGH) pulsosM1++;
  else pulsosM1--;
}

void IRAM_ATTR contarM2() {
  if (digitalRead(pinEncM2B) == HIGH) pulsosM2++;
  else pulsosM2--;
}

void IRAM_ATTR contarM3() {
  if (digitalRead(pinEncM3B) == HIGH) pulsosM3++;
  else pulsosM3--;
}

void IRAM_ATTR contarM4() {
  if (digitalRead(pinEncM4B) == HIGH) pulsosM4++;
  else pulsosM4--;
}

// 4. INICIALIZACIÓN
void setupMotores() {
  // Pines de control de motores
  pinMode(pinM1_IN1, OUTPUT); pinMode(pinM1_IN2, OUTPUT); pinMode(pinM1_PWM, OUTPUT);
  pinMode(pinM2_IN1, OUTPUT); pinMode(pinM2_IN2, OUTPUT); pinMode(pinM2_PWM, OUTPUT);
  pinMode(pinM3_IN1, OUTPUT); pinMode(pinM3_IN2, OUTPUT); pinMode(pinM3_PWM, OUTPUT);
  pinMode(pinM4_IN1, OUTPUT); pinMode(pinM4_IN2, OUTPUT); pinMode(pinM4_PWM, OUTPUT);

  // Configuración de encoders
  pinMode(pinEncM1A, INPUT); pinMode(pinEncM1B, INPUT);
  pinMode(pinEncM2A, INPUT); pinMode(pinEncM2B, INPUT);
  pinMode(pinEncM3A, INPUT); pinMode(pinEncM3B, INPUT);
  pinMode(pinEncM4A, INPUT); pinMode(pinEncM4B, INPUT);

  // Registro de interrupciones en Canal A de cada motor
  attachInterrupt(digitalPinToInterrupt(pinEncM1A), contarM1, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncM2A), contarM2, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncM3A), contarM3, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncM4A), contarM4, RISING);

  pararMotores();
}

void resetEncoders() {
  // Lectura/Escritura atómica en ESP32 para evitar corrupción
  portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
  taskENTER_CRITICAL(&myMutex);
  pulsosM1 = 0;
  pulsosM2 = 0;
  pulsosM3 = 0;
  pulsosM4 = 0;
  taskEXIT_CRITICAL(&myMutex);
}

// 5. CONTROL INDIVIDUAL DE MOTORES (Actuador de bajo nivel)
void setDriverMotor(int pinIN1, int pinIN2, int pinPWM, int velocidad) {
  if (velocidad > 0) {
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);
  } else if (velocidad < 0) {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
  } else {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, LOW);
  }
  analogWrite(pinPWM, abs(velocidad));
}

// 6. CINEMÁTICA HOLONÓMICA (Función principal de movimiento omnidireccional)
// Vx: Movimiento Lateral (+ Derecha, - Izquierda)
// Vy: Movimiento Longitudinal (+ Avanzar, - Retroceder)
// W:  Rotación sobre el propio eje (+ Hora, - Anti-hora)
void moverOmni(int Vx, int Vy, int W) {
  // Ecuaciones cinemáticas para chasis omnidireccional/Mecanum a 45°
  int vM1 = Vy + Vx + W; // Frontal Izquierdo
  int vM2 = Vy - Vx - W; // Frontal Derecho
  int vM3 = Vy - Vx + W; // Trasero Izquierdo
  int vM4 = Vy + Vx - W; // Trasero Derecho

  // Normalización de señales al rango máximo PWM (255)
  int maxVel = max(max(abs(vM1), abs(vM2)), max(abs(vM3), abs(vM4)));
  if (maxVel > 255) {
    vM1 = (vM1 * 255) / maxVel;
    vM2 = (vM2 * 255) / maxVel;
    vM3 = (vM3 * 255) / maxVel;
    vM4 = (vM4 * 255) / maxVel;
  }

  // Aplicación directa a los puentes H
  setDriverMotor(pinM1_IN1, pinM1_IN2, pinM1_PWM, vM1);
  setDriverMotor(pinM2_IN1, pinM2_IN2, pinM2_PWM, vM2);
  setDriverMotor(pinM3_IN1, pinM3_IN2, pinM3_PWM, vM3);
  setDriverMotor(pinM4_IN1, pinM4_IN2, pinM4_PWM, vM4);
}

void pararMotores() {
  moverOmni(0, 0, 0);
}