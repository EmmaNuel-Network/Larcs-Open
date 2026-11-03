// ============================================================================
// PESTAÑA: Motores.ino
// Controladores DC (PID), Motor Paso a Paso (Fast-I2C) y Servos Hardware
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <ESP32Servo.h>

extern Adafruit_MCP23X17 mcp;

// CREACIÓN E INSTANCIACIÓN REAL EN MEMORIA (Elimina error 'undefined reference')
ControlPD pdM1, pdM2, pdM3, pdM4;

const uint8_t MCP_I2C_ADDR = 0x20;
const uint8_t REG_GPIOB    = 0x13;

// Servomotores Hardware
Servo servoGarra;     // Servo 1
Servo servoCompuerta; // Servo 2

// Control MCP23017 Puerto A (Sentidos de Tracción DC)
const int pinM1_IN1 = 0;  const int pinM1_IN2 = 1;  const int pinM1_PWM = 12; // Front-Izq
const int pinM3_IN1 = 2;  const int pinM3_IN2 = 3;  const int pinM3_PWM = 13; // Rear-Izq
const int pinM2_IN1 = 4;  const int pinM2_IN2 = 5;  const int pinM2_PWM = 14; // Front-Der
const int pinM4_IN1 = 6;  const int pinM4_IN2 = 7;  const int pinM4_PWM = 27; // Rear-Der

// Encoders Tracción DC
const int pinEncM1A = 34; const int pinEncM1B = 2;
const int pinEncM2A = 35; const int pinEncM2B = 5;
const int pinEncM3A = 4;  const int pinEncM3B = 15;
const int pinEncM4A = 32; const int pinEncM4B = 33;

// Pines Servomotores Directos ESP32
const int PIN_SERVO_GARRA     = 19;
const int PIN_SERVO_COMPUERTA = 17;

const int ANG_GARRA_REPOSO   = 0;
const int ANG_GARRA_20       = 20;
const int ANG_GARRA_CAPTURA  = 80;

const int ANG_COMPUERTA_CERRADA = 140;
const int ANG_COMPUERTA_ABIERTA  = 90;

// Configuración Paso a Paso Fast-I2C (PB4 a PB7)
const byte pasoMedioBytes[8] = {
  0b00010000, 0b00110000, 0b00100000, 0b01100000,
  0b01000000, 0b11000000, 0b10000000, 0b10010000
};

const unsigned long RETARDO_PASO_US = 1800;
const float PASOS_POR_MM            = 30.0f;
int pasoIndex = 0;

const int VELOCIDAD_ACERCO                        = 85;
const unsigned long TIEMPO_ACERCO_MS              = 450;
const int VELOCIDAD_LATERAL                        = 95;
const unsigned long TIEMPO_DESPLAZAMIENTO_PELOTAS_MS = 850;
const unsigned long TIEMPO_ENTRE_ARBOLES_MS       = 2200;

volatile long pulsosM1 = 0, pulsosM2 = 0, pulsosM3 = 0, pulsosM4 = 0;
static portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

const int MIN_PWM_DEADBAND  = 35;
const float MAX_PULSOS_20MS  = 60.0f;
unsigned long ultimoTiempoPD = 0;
const unsigned long INTERVALO_PD_MS = 20;

void IRAM_ATTR contarM1() { portENTER_CRITICAL_ISR(&encoderMux); if (gpio_get_level((gpio_num_t)pinEncM1B)) pulsosM1++; else pulsosM1--; portEXIT_CRITICAL_ISR(&encoderMux); }
void IRAM_ATTR contarM2() { portENTER_CRITICAL_ISR(&encoderMux); if (gpio_get_level((gpio_num_t)pinEncM2B)) pulsosM2++; else pulsosM2--; portEXIT_CRITICAL_ISR(&encoderMux); }
void IRAM_ATTR contarM3() { portENTER_CRITICAL_ISR(&encoderMux); if (gpio_get_level((gpio_num_t)pinEncM3B)) pulsosM3++; else pulsosM3--; portEXIT_CRITICAL_ISR(&encoderMux); }
void IRAM_ATTR contarM4() { portENTER_CRITICAL_ISR(&encoderMux); if (gpio_get_level((gpio_num_t)pinEncM4B)) pulsosM4++; else pulsosM4--; portEXIT_CRITICAL_ISR(&encoderMux); }

long getPulsosM1() { taskENTER_CRITICAL(&encoderMux); long p = pulsosM1; taskEXIT_CRITICAL(&encoderMux); return p; }
long getPulsosM2() { taskENTER_CRITICAL(&encoderMux); long p = pulsosM2; taskEXIT_CRITICAL(&encoderMux); return p; }
long getPulsosM3() { taskENTER_CRITICAL(&encoderMux); long p = pulsosM3; taskEXIT_CRITICAL(&encoderMux); return p; }
long getPulsosM4() { taskENTER_CRITICAL(&encoderMux); long p = pulsosM4; taskEXIT_CRITICAL(&encoderMux); return p; }

void resetEncoders() {
  taskENTER_CRITICAL(&encoderMux);
  pulsosM1 = 0; pulsosM2 = 0; pulsosM3 = 0; pulsosM4 = 0;
  taskEXIT_CRITICAL(&encoderMux);

  pdM1.pulsosAnteriores = 0; pdM1.errorPrev = 0;
  pdM2.pulsosAnteriores = 0; pdM2.errorPrev = 0;
  pdM3.pulsosAnteriores = 0; pdM3.errorPrev = 0;
  pdM4.pulsosAnteriores = 0; pdM4.errorPrev = 0;
}

int aplicarDeadband(int pwm) {
  if (pwm == 0) return 0;
  int pwmAbs = abs(pwm);
  if (pwmAbs < MIN_PWM_DEADBAND) pwmAbs = MIN_PWM_DEADBAND;
  return (pwm > 0) ? pwmAbs : -pwmAbs;
}

void setDriverMotor(int pinIN1, int pinIN2, int pinPWM, int velocidad) {
  velocidad = aplicarDeadband(velocidad);
  if (velocidad > 0) {
    mcp.digitalWrite(pinIN1, HIGH); mcp.digitalWrite(pinIN2, LOW);
  } else if (velocidad < 0) {
    mcp.digitalWrite(pinIN1, LOW); mcp.digitalWrite(pinIN2, HIGH);
  } else {
    mcp.digitalWrite(pinIN1, LOW); mcp.digitalWrite(pinIN2, LOW);
  }
  analogWrite(pinPWM, min(abs(velocidad), 255));
}

int calcularPDMotor(ControlPD &pd, long pulsosActuales, float dt) {
  if (pd.setpoint == 0.0f) {
    pd.errorPrev = 0.0f;
    pd.pulsosAnteriores = pulsosActuales;
    return 0;
  }

  float velocidadMedida = (float)(pulsosActuales - pd.pulsosAnteriores);
  pd.pulsosAnteriores = pulsosActuales;

  float pulsosObjetivo = (pd.setpoint / 255.0f) * MAX_PULSOS_20MS;
  float error = pulsosObjetivo - velocidadMedida;
  float derivativa = (dt > 0.0f) ? ((error - pd.errorPrev) / dt) : 0.0f;
  pd.errorPrev = error;

  float pwmCorregido = pd.setpoint + (pd.Kp * error) + (pd.Kd * derivativa);
  return (int)constrain(pwmCorregido, -255.0f, 255.0f);
}

void actualizarControlMotores() {
  unsigned long tiempoActual = millis();
  if (tiempoActual - ultimoTiempoPD >= INTERVALO_PD_MS) {
    float dt = (tiempoActual - ultimoTiempoPD) / 1000.0f;
    ultimoTiempoPD = tiempoActual;

    pdM1.outputPWM = calcularPDMotor(pdM1, getPulsosM1(), dt);
    pdM2.outputPWM = calcularPDMotor(pdM2, getPulsosM2(), dt);
    pdM3.outputPWM = calcularPDMotor(pdM3, getPulsosM3(), dt);
    pdM4.outputPWM = calcularPDMotor(pdM4, getPulsosM4(), dt);

    setDriverMotor(pinM1_IN1, pinM1_IN2, pinM1_PWM, pdM1.outputPWM);
    setDriverMotor(pinM2_IN1, pinM2_IN2, pinM2_PWM, -pdM2.outputPWM);
    setDriverMotor(pinM3_IN1, pinM3_IN2, pinM3_PWM, pdM3.outputPWM);
    setDriverMotor(pinM4_IN1, pinM4_IN2, pinM4_PWM, -pdM4.outputPWM);
  }
}

void moverOmni(int Vx, int Vy, int W) {
  int vM1 = Vy + Vx - W;
  int vM2 = Vy - Vx + W;
  int vM3 = Vy - Vx - W;
  int vM4 = Vy + Vx + W;

  int maxVel = max(max(abs(vM1), abs(vM2)), max(abs(vM3), abs(vM4)));
  if (maxVel > 255) {
    vM1 = (vM1 * 255) / maxVel; vM2 = (vM2 * 255) / maxVel;
    vM3 = (vM3 * 255) / maxVel; vM4 = (vM4 * 255) / maxVel;
  }

  pdM1.setpoint = (float)vM1; pdM2.setpoint = (float)vM2;
  pdM3.setpoint = (float)vM3; pdM4.setpoint = (float)vM4;
}

void girar180Grados() {
  unsigned long inicio = millis();
  while (millis() - inicio < 1100) {
    moverOmni(0, 0, 140);
    actualizarControlMotores();
    delay(2);
  }
  pararMotores();
}

void pararMotores() {
  pdM1.setpoint = 0; pdM1.errorPrev = 0;
  pdM2.setpoint = 0; pdM2.errorPrev = 0;
  pdM3.setpoint = 0; pdM3.errorPrev = 0;
  pdM4.setpoint = 0; pdM4.errorPrev = 0;

  setDriverMotor(pinM1_IN1, pinM1_IN2, pinM1_PWM, 0);
  setDriverMotor(pinM2_IN1, pinM2_IN2, pinM2_PWM, 0);
  setDriverMotor(pinM3_IN1, pinM3_IN2, pinM3_PWM, 0);
  setDriverMotor(pinM4_IN1, pinM4_IN2, pinM4_PWM, 0);
}

void apagarBobinasElevador() {
  Wire.beginTransmission(MCP_I2C_ADDR);
  Wire.write(REG_GPIOB);
  Wire.write(0x00);
  Wire.endTransmission();
}

void darPasoDirecto(int sentido) {
  pasoIndex += sentido;
  if (pasoIndex > 7) pasoIndex = 0;
  if (pasoIndex < 0) pasoIndex = 7;

  Wire.beginTransmission(MCP_I2C_ADDR);
  Wire.write(REG_GPIOB);
  Wire.write(pasoMedioBytes[pasoIndex]);
  Wire.endTransmission();

  delayMicroseconds(RETARDO_PASO_US);
}

void moverMM(float mm, bool subir) {
  long pasosTotales = abs((long)(mm * PASOS_POR_MM));
  int sentido = subir ? 1 : -1;

  for (long i = 0; i < pasosTotales; i++) {
    darPasoDirecto(sentido);
  }
  apagarBobinasElevador();
}

void moverServoSuave(Servo &servo, int angInicio, int angFin, int msPaso = 12) {
  int paso = (angFin >= angInicio) ? 1 : -1;
  for (int a = angInicio; (paso > 0) ? (a <= angFin) : (a >= angFin); a += paso) {
    servo.write(a);
    actualizarControlMotores(); // Mantiene activa la tracción mientras se mueve el servo
    delay(msPaso);
  }
}

void acercarAPelota() {
  unsigned long inicio = millis();
  while (millis() - inicio < TIEMPO_ACERCO_MS) {
    moverOmni(0, VELOCIDAD_ACERCO, 0);
    actualizarControlMotores();
    delay(2);
  }
  pararMotores();
  delay(150);
}

void retirarDePelota() {
  unsigned long inicio = millis();
  while (millis() - inicio < TIEMPO_ACERCO_MS) {
    moverOmni(0, -VELOCIDAD_ACERCO, 0);
    actualizarControlMotores();
    delay(2);
  }
  pararMotores();
  delay(150);
}

void cosecharEstandar() {
  moverServoSuave(servoGarra, ANG_GARRA_REPOSO, ANG_GARRA_CAPTURA, 12);
  delay(400);
  moverServoSuave(servoGarra, ANG_GARRA_CAPTURA, ANG_GARRA_REPOSO, 12);
  delay(300);
}

void cosecharEspecialNivel3() {
  acercarAPelota();
  moverServoSuave(servoGarra, ANG_GARRA_REPOSO, ANG_GARRA_20, 12);
  delay(150);
  moverMM(150.0f, false);
  delay(150);
  moverServoSuave(servoGarra, ANG_GARRA_20, ANG_GARRA_CAPTURA, 12);
  delay(500);
  moverServoSuave(servoGarra, ANG_GARRA_CAPTURA, ANG_GARRA_REPOSO, 12);
  delay(300);
  retirarDePelota();
  moverMM(150.0f, true);
  delay(200);
}

void vaciarCompuertaTrasera() {
  moverServoSuave(servoCompuerta, ANG_COMPUERTA_CERRADA, ANG_COMPUERTA_ABIERTA, 15);
  delay(2500);
  moverServoSuave(servoCompuerta, ANG_COMPUERTA_ABIERTA, ANG_COMPUERTA_CERRADA, 15);
  delay(500);
}

void desplazarseALaSiguientePelota(int direccion) {
  unsigned long inicio = millis();
  while (millis() - inicio < TIEMPO_DESPLAZAMIENTO_PELOTAS_MS) {
    moverOmni(direccion * VELOCIDAD_LATERAL, 0, 0);
    actualizarControlMotores();
    delay(2);
  }
  pararMotores();
  delay(250);
}

void desplazarseEntreArboles() {
  unsigned long inicio = millis();
  while (millis() - inicio < TIEMPO_ENTRE_ARBOLES_MS) {
    moverOmni(VELOCIDAD_LATERAL, 0, 0);
    actualizarControlMotores();
    delay(2);
  }
  pararMotores();
  delay(300);
}

void setupMotores() {
  pdM1.Kp = 1.2f; pdM1.Kd = 0.15f;
  pdM2.Kp = 1.2f; pdM2.Kd = 0.15f;
  pdM3.Kp = 1.2f; pdM3.Kd = 0.15f;
  pdM4.Kp = 1.2f; pdM4.Kd = 0.15f;

  mcp.pinMode(pinM1_IN1, OUTPUT); mcp.pinMode(pinM1_IN2, OUTPUT);
  mcp.pinMode(pinM2_IN1, OUTPUT); mcp.pinMode(pinM2_IN2, OUTPUT);
  mcp.pinMode(pinM3_IN1, OUTPUT); mcp.pinMode(pinM3_IN2, OUTPUT);
  mcp.pinMode(pinM4_IN1, OUTPUT); mcp.pinMode(pinM4_IN2, OUTPUT);

  mcp.pinMode(12, OUTPUT); mcp.pinMode(13, OUTPUT);
  mcp.pinMode(14, OUTPUT); mcp.pinMode(15, OUTPUT);

  pinMode(pinM1_PWM, OUTPUT); pinMode(pinM2_PWM, OUTPUT);
  pinMode(pinM3_PWM, OUTPUT); pinMode(pinM4_PWM, OUTPUT);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoGarra.setPeriodHertz(50);
  servoCompuerta.setPeriodHertz(50);

  servoGarra.attach(PIN_SERVO_GARRA, 500, 2400);
  servoCompuerta.attach(PIN_SERVO_COMPUERTA, 500, 2400);

  servoGarra.write(ANG_GARRA_REPOSO);
  servoCompuerta.write(ANG_COMPUERTA_CERRADA);

  pinMode(pinEncM1A, INPUT); pinMode(pinEncM1B, INPUT_PULLUP);
  pinMode(pinEncM2A, INPUT); pinMode(pinEncM2B, INPUT_PULLUP);
  pinMode(pinEncM3A, INPUT_PULLUP); pinMode(pinEncM3B, INPUT_PULLUP);
  pinMode(pinEncM4A, INPUT_PULLUP); pinMode(pinEncM4B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pinEncM1A), contarM1, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncM2A), contarM2, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncM3A), contarM3, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncM4A), contarM4, RISING);

  pararMotores();
  apagarBobinasElevador();
}