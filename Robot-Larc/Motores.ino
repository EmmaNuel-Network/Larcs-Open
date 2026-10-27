// ==========================================
// PESTAÑA: Motores.ino
// Cinemática Holonómica, Encoders y Servomotores (GPIO 25 y 26)
// ==========================================
#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

extern Adafruit_MCP23X17 mcp;

extern ControlPD pdM1, pdM2, pdM3, pdM4;
extern const int pinServo25;
extern const int pinServo26;

// Drivers DC
const int pinM1_IN1 = 0;  const int pinM1_IN2 = 1;  const int pinM1_PWM = 12; // Front-Izq
const int pinM3_IN1 = 2;  const int pinM3_IN2 = 3;  const int pinM3_PWM = 13; // Rear-Izq
const int pinM2_IN1 = 4;  const int pinM2_IN2 = 5;  const int pinM2_PWM = 14; // Front-Der
const int pinM4_IN1 = 6;  const int pinM4_IN2 = 7;  const int pinM4_PWM = 27; // Rear-Der

// Encoders
const int pinEncM1A = 34; const int pinEncM1B = 2;
const int pinEncM2A = 35; const int pinEncM2B = 5;
const int pinEncM3A = 4;  const int pinEncM3B = 15;
const int pinEncM4A = 17; const int pinEncM4B = 33;

volatile long pulsosM1 = 0, pulsosM2 = 0, pulsosM3 = 0, pulsosM4 = 0;
static portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

const int MIN_PWM_DEADBAND = 35;
const float MAX_PULSOS_20MS = 60.0f;
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

void setupMotores() {
  pdM1.Kp = 1.2f; pdM1.Kd = 0.15f;
  pdM2.Kp = 1.2f; pdM2.Kd = 0.15f;
  pdM3.Kp = 1.2f; pdM3.Kd = 0.15f;
  pdM4.Kp = 1.2f; pdM4.Kd = 0.15f;

  mcp.pinMode(pinM1_IN1, OUTPUT); mcp.pinMode(pinM1_IN2, OUTPUT);
  mcp.pinMode(pinM2_IN1, OUTPUT); mcp.pinMode(pinM2_IN2, OUTPUT);
  mcp.pinMode(pinM3_IN1, OUTPUT); mcp.pinMode(pinM3_IN2, OUTPUT);
  mcp.pinMode(pinM4_IN1, OUTPUT); mcp.pinMode(pinM4_IN2, OUTPUT);

  pinMode(pinM1_PWM, OUTPUT); pinMode(pinM2_PWM, OUTPUT);
  pinMode(pinM3_PWM, OUTPUT); pinMode(pinM4_PWM, OUTPUT);

  pinMode(pinEncM1A, INPUT); pinMode(pinEncM1B, INPUT_PULLUP);
  pinMode(pinEncM2A, INPUT); pinMode(pinEncM2B, INPUT_PULLUP);
  pinMode(pinEncM3A, INPUT_PULLUP); pinMode(pinEncM3B, INPUT_PULLUP);
  pinMode(pinEncM4A, INPUT_PULLUP); pinMode(pinEncM4B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pinEncM1A), contarM1, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncM2A), contarM2, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncM3A), contarM3, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncM4A), contarM4, RISING);

  resetEncoders();
  pararMotores();
}

void moverPulsoServo(int pinGPIO, int angulo) {
  int duracionUs = map(angulo, 0, 180, 500, 2400);
  digitalWrite(pinGPIO, HIGH);
  delayMicroseconds(duracionUs);
  digitalWrite(pinGPIO, LOW);
  delayMicroseconds(20000 - duracionUs);
}

void accionarRecolectorYClasificar(bool esValida) {
  pararMotores();

  int anguloCompuerta = esValida ? 0 : 90;
  for (int i = 0; i < 15; i++) {
    moverPulsoServo(pinServo26, anguloCompuerta);
  }

  for (int pos = 0; pos <= 80; pos += 10) {
    for (int i = 0; i < 2; i++) moverPulsoServo(pinServo25, pos);
  }
  delay(100);

  for (int pos = 80; pos >= 0; pos -= 10) {
    for (int i = 0; i < 2; i++) moverPulsoServo(pinServo25, pos);
  }
}
