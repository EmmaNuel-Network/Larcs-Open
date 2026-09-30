// ==========================================
// PESTAÑA: Motores.ino (Refactorizado)
// ==========================================
#include <Arduino.h>

// 1. ASIGNACIÓN DE PINES EN ESP32
// Motor 1: Frontal Izquierdo
const int pinM1_IN1 = 5;
const int pinM1_IN2 = 13;
const int pinM1_PWM = 14;
const int pinEncM1A = 34; // GPI: Requiere resistencia Pull-Up física de 10k a 3.3V
const int pinEncM1B = 35; // GPI: Requiere resistencia Pull-Up física de 10k a 3.3V

// Motor 2: Frontal Derecho
const int pinM2_IN1 = 25;
const int pinM2_IN2 = 26;
const int pinM2_PWM = 27;
const int pinEncM2A = 36; // GPI: Requiere Pull-Up física de 10k
const int pinEncM2B = 39; // GPI: Requiere Pull-Up física de 10k

// Motor 3: Trasero Izquierdo
// REASIGNADO: Se cambian GPIO 16 y 17 por GPIO 2 y 15 para liberar Serial2 (UART)
const int pinM3_IN1 = 26;
const int pinM3_IN2 = 15;
const int pinM3_PWM = 4;
const int pinEncM3A = 32;
const int pinEncM3B = 33;

// Motor 4: Trasero Derecho
const int pinM4_IN1 = 18;
const int pinM4_IN2 = 19;
const int pinM4_PWM = 21;
const int pinEncM4A = 22;
const int pinEncM4B = 23;

// 2. VARIABLES GLOBALES Y SECCIÓN CRÍTICA
volatile long pulsosM1 = 0;
volatile long pulsosM2 = 0;
volatile long pulsosM3 = 0;
volatile long pulsosM4 = 0;

static portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

const int VELOCIDAD_CRUCERO = 180;
const int VELOCIDAD_GIRO    = 150;
const int MIN_PWM_DEADBAND  = 35; 

// 3. RUTINAS DE INTERRUPCIÓN (ISR) CON PROTECCIÓN DE MULTINÚCLEO
void IRAM_ATTR contarM1() {
  portENTER_CRITICAL_ISR(&encoderMux);
  if (gpio_get_level((gpio_num_t)pinEncM1B)) pulsosM1++;
  else pulsosM1--;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR contarM2() {
  portENTER_CRITICAL_ISR(&encoderMux);
  if (gpio_get_level((gpio_num_t)pinEncM2B)) pulsosM2++;
  else pulsosM2--;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR contarM3() {
  portENTER_CRITICAL_ISR(&encoderMux);
  if (gpio_get_level((gpio_num_t)pinEncM3B)) pulsosM3++;
  else pulsosM3--;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR contarM4() {
  portENTER_CRITICAL_ISR(&encoderMux);
  if (gpio_get_level((gpio_num_t)pinEncM4B)) pulsosM4++;
  else pulsosM4--;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

// 4. FUNCIONES ATÓMICAS DE LECTURA DE ENCODERS
long getPulsosM1() { taskENTER_CRITICAL(&encoderMux); long p = pulsosM1; taskEXIT_CRITICAL(&encoderMux); return p; }
long getPulsosM2() { taskENTER_CRITICAL(&encoderMux); long p = pulsosM2; taskEXIT_CRITICAL(&encoderMux); return p; }
long getPulsosM3() { taskENTER_CRITICAL(&encoderMux); long p = pulsosM3; taskEXIT_CRITICAL(&encoderMux); return p; }
long getPulsosM4() { taskENTER_CRITICAL(&encoderMux); long p = pulsosM4; taskEXIT_CRITICAL(&encoderMux); return p; }

void resetEncoders() {
  taskENTER_CRITICAL(&encoderMux);
  pulsosM1 = 0;
  pulsosM2 = 0;
  pulsosM3 = 0;
  pulsosM4 = 0;
  taskEXIT_CRITICAL(&encoderMux);
}

// 5. INICIALIZACIÓN
void setupMotores() {
  pinMode(pinM1_IN1, OUTPUT); pinMode(pinM1_IN2, OUTPUT); pinMode(pinM1_PWM, OUTPUT);
  pinMode(pinM2_IN1, OUTPUT); pinMode(pinM2_IN2, OUTPUT); pinMode(pinM2_PWM, OUTPUT);
  pinMode(pinM3_IN1, OUTPUT); pinMode(pinM3_IN2, OUTPUT); pinMode(pinM3_PWM, OUTPUT);
  pinMode(pinM4_IN1, OUTPUT); pinMode(pinM4_IN2, OUTPUT); pinMode(pinM4_PWM, OUTPUT);

  pinMode(pinEncM1A, INPUT); pinMode(pinEncM1B, INPUT);
  pinMode(pinEncM2A, INPUT); pinMode(pinEncM2B, INPUT);
  
  pinMode(pinEncM3A, INPUT_PULLUP); pinMode(pinEncM3B, INPUT_PULLUP);
  pinMode(pinEncM4A, INPUT_PULLUP); pinMode(pinEncM4B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pinEncM1A), contarM1, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncM2A), contarM2, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncM3A), contarM3, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncM4A), contarM4, RISING);

  pararMotores();
}

// 6. COMPENSACIÓN DE ZONA MUERTA (DEADBAND)
int aplicarDeadband(int pwm) {
  if (pwm == 0) return 0;
  int pwmAbs = abs(pwm);
  if (pwmAbs < MIN_PWM_DEADBAND) pwmAbs = MIN_PWM_DEADBAND;
  return (pwm > 0) ? pwmAbs : -pwmAbs;
}

// 7. CONTROL INDIVIDUAL DE MOTORES
void setDriverMotor(int pinIN1, int pinIN2, int pinPWM, int velocidad) {
  velocidad = aplicarDeadband(velocidad);
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
  analogWrite(pinPWM, min(abs(velocidad), 255));
}

// 8. CINEMÁTICA HOLONÓMICA CORREGIDA (Convención Estándar CCW para W)
void moverOmni(int Vx, int Vy, int W) {
  int vM1 = Vy + Vx - W; // Frontal Izquierdo
  int vM2 = Vy - Vx + W; // Frontal Derecho
  int vM3 = Vy - Vx - W; // Trasero Izquierdo
  int vM4 = Vy + Vx + W; // Trasero Derecho

  int maxVel = max(max(abs(vM1), abs(vM2)), max(abs(vM3), abs(vM4)));
  if (maxVel > 255) {
    vM1 = (vM1 * 255) / maxVel;
    vM2 = (vM2 * 255) / maxVel;
    vM3 = (vM3 * 255) / maxVel;
    vM4 = (vM4 * 255) / maxVel;
  }

  setDriverMotor(pinM1_IN1, pinM1_IN2, pinM1_PWM, vM1);
  setDriverMotor(pinM2_IN1, pinM2_IN2, pinM2_PWM, vM2);
  setDriverMotor(pinM3_IN1, pinM3_IN2, pinM3_PWM, vM3);
  setDriverMotor(pinM4_IN1, pinM4_IN2, pinM4_PWM, vM4);
}

void pararMotores() {
  moverOmni(0, 0, 0);
}