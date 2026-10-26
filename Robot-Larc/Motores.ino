// ==========================================
// PESTAÑA: Motores.ino (Control PD con Feedforward)
// ==========================================
#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

extern Adafruit_MCP23X17 mcp;
// Declaración única de las instancias globales
ControlPD pdM1, pdM2, pdM3, pdM4;

// ==========================================
// ASIGNACIÓN DE PINES CORREGIDA EN Motores.ino
// ==========================================
const int pinM1_IN1 = 0;  // PA0
const int pinM1_IN2 = 1;  // PA1
const int pinM1_PWM = 12; // ENA (GPIO 12)

const int pinM3_IN1 = 2;  // PA2 (IN3 en Driver Izq)
const int pinM3_IN2 = 3;  // PA3 (IN4 en Driver Izq)
const int pinM3_PWM = 13; // ENB (GPIO 13)

const int pinM2_IN1 = 4;  // PA4 (IN1 en Driver Der)
const int pinM2_IN2 = 5;  // PA5 (IN2 en Driver Der)
const int pinM2_PWM = 14; // ENA (GPIO 14)

const int pinM4_IN1 = 6;  // PA6 (IN3 en Driver Der)
const int pinM4_IN2 = 7;  // PA7 (IN4 en Driver Der)
const int pinM4_PWM = 27; // ENB (GPIO 27)

const int pinEncM1A = 34; const int pinEncM1B = 2;   // Delantero Izq
const int pinEncM2A = 35; const int pinEncM2B = 5;   // Delantero Der
const int pinEncM3A = 4;  const int pinEncM3B = 15;  // Trasero Izq
const int pinEncM4A = 17; const int pinEncM4B = 33;  // Trasero Der

// 3. VARIABLES GLOBALES Y SECCIÓN CRÍTICA
volatile long pulsosM1 = 0;
volatile long pulsosM2 = 0;
volatile long pulsosM3 = 0;
volatile long pulsosM4 = 0;

static portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

const int VELOCIDAD_CRUCERO = 180;
const int VELOCIDAD_GIRO    = 150;
const int MIN_PWM_DEADBAND  = 35; 

const float MAX_PULSOS_20MS = 60.0f; 

unsigned long ultimoTiempoPD = 0;
const unsigned long INTERVALO_PD_MS = 20; // 50 Hz

// 4. RUTINAS DE INTERRUPCIÓN (ISR)
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

// 5. LECTURAS ATÓMICAS Y REANUDACIÓN DE ENCODERS
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

// 6. INICIALIZACIÓN
void setupMotores() {
  // Las asignaciones van DENTRO de la función setup
  pdM1.Kp = 1.2f; pdM1.Kd = 0.15f;
  pdM2.Kp = 1.2f; pdM2.Kd = 0.15f;
  pdM3.Kp = 1.2f; pdM3.Kd = 0.15f;
  pdM4.Kp = 1.2f; pdM4.Kd = 0.15f;

  mcp.pinMode(pinM1_IN1, OUTPUT); mcp.pinMode(pinM1_IN2, OUTPUT);
  mcp.pinMode(pinM2_IN1, OUTPUT); mcp.pinMode(pinM2_IN2, OUTPUT);
  mcp.pinMode(pinM3_IN1, OUTPUT); mcp.pinMode(pinM3_IN2, OUTPUT);
  mcp.pinMode(pinM4_IN1, OUTPUT); mcp.pinMode(pinM4_IN2, OUTPUT);

  pinMode(pinM1_PWM, OUTPUT);
  pinMode(pinM2_PWM, OUTPUT);
  pinMode(pinM3_PWM, OUTPUT);
  pinMode(pinM4_PWM, OUTPUT);
  
  // -- ESTOS SON LOS QUE FALTAN --
  pinMode(pinEncM1A, INPUT);        // Requiere Pull-Up físico
  pinMode(pinEncM1B, INPUT_PULLUP); 
  pinMode(pinEncM2A, INPUT);        // Requiere Pull-Up físico
  pinMode(pinEncM2B, INPUT_PULLUP); 
  // ------------------------------

  pinMode(pinEncM3A, INPUT_PULLUP); pinMode(pinEncM3B, INPUT_PULLUP);
  pinMode(pinEncM4A, INPUT_PULLUP); pinMode(pinEncM4B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(pinEncM1A), contarM1, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncM2A), contarM2, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncM3A), contarM3, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncM4A), contarM4, RISING);

  pararMotores();
}

// 7. COMPENSACIÓN DE ZONA MUERTA
int aplicarDeadband(int pwm) {
  if (pwm == 0) return 0;
  int pwmAbs = abs(pwm);
  if (pwmAbs < MIN_PWM_DEADBAND) pwmAbs = MIN_PWM_DEADBAND;
  return (pwm > 0) ? pwmAbs : -pwmAbs;
}

// 8. CONTROL FÍSICO DE DRIVERS
void setDriverMotor(int pinIN1, int pinIN2, int pinPWM, int velocidad) {
  velocidad = aplicarDeadband(velocidad);
  if (velocidad > 0) {
    mcp.digitalWrite(pinIN1, HIGH);
    mcp.digitalWrite(pinIN2, LOW);
  } else if (velocidad < 0) {
    mcp.digitalWrite(pinIN1, LOW);
    mcp.digitalWrite(pinIN2, HIGH);
  } else {
    mcp.digitalWrite(pinIN1, LOW);
    mcp.digitalWrite(pinIN2, LOW);
  }
  analogWrite(pinPWM, min(abs(velocidad), 255));
}

// 9. CÁLCULO PD CON FEEDFORWARD DE VELOCIDAD
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

// 10. BUCLE PERIÓDICO (Llamar continuamente en el loop principal)
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

// 11. CINEMÁTICA HOLONÓMICA
void moverOmni(int Vx, int Vy, int W) {
  int vM1 = Vy + Vx - W;
  int vM2 = Vy - Vx + W;
  int vM3 = Vy - Vx - W;
  int vM4 = Vy + Vx + W;

  int maxVel = max(max(abs(vM1), abs(vM2)), max(abs(vM3), abs(vM4)));
  if (maxVel > 255) {
    vM1 = (vM1 * 255) / maxVel;
    vM2 = (vM2 * 255) / maxVel;
    vM3 = (vM3 * 255) / maxVel;
    vM4 = (vM4 * 255) / maxVel;
  }

  pdM1.setpoint = (float)vM1;
  pdM2.setpoint = (float)vM2;
  pdM3.setpoint = (float)vM3;
  pdM4.setpoint = (float)vM4;
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