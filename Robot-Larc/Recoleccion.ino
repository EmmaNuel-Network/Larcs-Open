// ==========================================
// PESTAÑA: Recoleccion.ino
// Servo de la garra/rampa (GPIO 19) y de la compuerta trasera (GPIO 0).
// Secuencias de captura basadas en las del equipo.
// ==========================================
#include <Arduino.h>
#include <ESP32Servo.h>

void actualizarControlMotores();
void elevadorIrAMM(float objetivoMM);
float posicionElevadorMM();

const int PIN_SERVO_GARRA     = 19;
const int PIN_SERVO_COMPUERTA = 0;

// Ángulos (valores del equipo)
const int ANG_GARRA_REPOSO      = 0;
const int ANG_GARRA_SUJETAR     = 20;   // Nivel 3: sostiene la pelota mientras baja
const int ANG_GARRA_CAPTURA     = 80;
const int ANG_COMPUERTA_CERRADA = 140;
const int ANG_COMPUERTA_ABIERTA = 90;

const int MS_PASO_GARRA     = 12;   // ms por grado (movimiento suave)
const int MS_PASO_COMPUERTA = 15;

Servo servoGarra;
Servo servoCompuerta;
int anguloGarra     = ANG_GARRA_REPOSO;
int anguloCompuerta = ANG_COMPUERTA_CERRADA;

// Espera atendiendo el control de motores (robot detenido)
void esperaActiva(unsigned long ms) {
  unsigned long inicio = millis();
  while (millis() - inicio < ms) {
    actualizarControlMotores();
    delay(2);
  }
}

void setupRecoleccion() {
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
  anguloGarra = ANG_GARRA_REPOSO;
  anguloCompuerta = ANG_COMPUERTA_CERRADA;
}

// Mueve un servo grado a grado desde su ángulo actual hasta el destino
void moverServoSuave(Servo &servo, int &anguloActual, int destino, int msPorGrado) {
  int paso = (destino >= anguloActual) ? 1 : -1;
  while (anguloActual != destino) {
    anguloActual += paso;
    servo.write(anguloActual);
    actualizarControlMotores();
    delay(msPorGrado);
  }
}

void garraA(int angulo)     { moverServoSuave(servoGarra, anguloGarra, angulo, MS_PASO_GARRA); }
void compuertaA(int angulo) { moverServoSuave(servoCompuerta, anguloCompuerta, angulo, MS_PASO_COMPUERTA); }

// Niveles 1 y 2: la rampa se inclina y la pelota rueda al almacén
void capturaEstandar() {
  garraA(ANG_GARRA_CAPTURA);
  esperaActiva(400);
  garraA(ANG_GARRA_REPOSO);
  esperaActiva(300);
}

// Nivel 3, primera parte: sujeta la pelota, baja el elevador para que
// no caiga lejos y la suelta. La subida se hace después de retroceder.
void capturaNivel3Bajar(float bajadaMM) {
  garraA(ANG_GARRA_SUJETAR);
  esperaActiva(150);
  elevadorIrAMM(posicionElevadorMM() - bajadaMM);
  esperaActiva(150);
  garraA(ANG_GARRA_CAPTURA);
  esperaActiva(500);
  garraA(ANG_GARRA_REPOSO);
  esperaActiva(300);
}

void descargarCompuerta() {
  compuertaA(ANG_COMPUERTA_ABIERTA);
  esperaActiva(2500);
  compuertaA(ANG_COMPUERTA_CERRADA);
  esperaActiva(500);
}