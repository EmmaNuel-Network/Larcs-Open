// ==========================================
// PESTAÑA: Infrarrojos.ino (Arduino UNO)
// ==========================================

// 1. ASIGNACIÓN DE PINES E INFRARROJOS
// Utiliza la barra roja (3 sensores en arreglo) y los módulos individuales azules (LM393)

// Sensores de alineación e inspección lateral izquierda (Reubicación)
const int pinIR_IzqExt = A0; // Sensor exterior izquierdo (Barra roja / Módulo azul)
const int pinIR_IzqInt = A1; // Sensor interior izquierdo (Barra roja)

// Sensor central (Alineación / Guía frontal)
const int pinIR_Centro = A2; // Sensor central (Barra roja)

// Sensor de límite lateral derecho (Retorno a zona de descarga)
const int pinIR_DerExt = A3; // Sensor extremo derecho (Módulo azul individual)

// 2. CONFIGURACIÓN DE LÓGICA DE Detección
// En módulos LM393 / TCRT5000: 
// - Sobre fondo claro (reflejante): DO emite LOW
// - Sobre línea negra (sin reflexión): DO emite HIGH
const int LINEA_NEGRA = HIGH;

// 3. INICIALIZACIÓN DE SENSORES IR
void setupInfrarrojos() {
  pinMode(pinIR_IzqExt, INPUT);
  pinMode(pinIR_IzqInt, INPUT);
  pinMode(pinIR_Centro, INPUT);
  pinMode(pinIR_DerExt, INPUT);
}

// 4. FUNCIONES DE LECTURA INDIVIDUAL
bool irIzqExterior() {
  return digitalRead(pinIR_IzqExt) == LINEA_NEGRA;
}

bool irIzqInterior() {
  return digitalRead(pinIR_IzqInt) == LINEA_NEGRA;
}

bool irCentro() {
  return digitalRead(pinIR_Centro) == LINEA_NEGRA;
}

bool irDerExterior() {
  return digitalRead(pinIR_DerExt) == LINEA_NEGRA;
}

// 5. RUTINAS DE Detección Y NAVEGACIÓN OMNIDIRECCIONAL

// Detecta si el robot llegó al límite lateral izquierdo (Usado para reubicación)
bool detectaLineaIzquierda() {
  // Retorna verdadero cuando al menos uno de los dos sensores izquierdos toca la línea lateral
  return (irIzqExterior() || irIzqInterior());
}

// Verifica alineación escuadrada en la línea izquierda (ambos sensores alineados)
bool alineadoEnLineaIzquierda() {
  return (irIzqExterior() && irIzqInterior());
}

// Detecta si el robot llegó al límite extremo derecho de la zona de árboles
bool detectaLineaDerecha() {
  return irDerExterior();
}

// Función general para compatibilidad con la máquina de estados
bool sensorIR_detectaLinea() {
  return (irIzqExterior() || irIzqInterior() || irCentro() || irDerExterior());
}

// 6. CÁLCULO DE ERROR PONDERADO PARA SEGUIMIENTO / ALINEACIÓN
// Devuelve un error numérico para el control PD cuando el robot avanza o se desplaza lateralmente
int calcularErrorLinea() {
  bool izqE = irIzqExterior();
  bool izqI = irIzqInterior();
  bool cen  = irCentro();
  bool derE = irDerExterior();

  // Caso: Centrado perfecto
  if (cen) return 0;
  
  // Desviaciones a la izquierda (Error negativo)
  if (izqI) return -1;
  if (izqE) return -2;

  // Desviaciones a la derecha (Error positivo)
  if (derE) return 2;

  // Si se pierde la línea, mantiene la última dirección de corrección conocida
  return 0;
}