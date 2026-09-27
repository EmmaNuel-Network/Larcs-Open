const int DISTANCIA_DESEADA_PARED = 15; // cm ideal

int leerDistancia(int pinTrig, int pinEcho) {
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);

  // Timeout reducido a 10000 us (10 ms -> max ~1.7 metros) para no congelar el PD
  long duracion = pulseIn(pinEcho, HIGH, 10000); 

  if (duracion == 0) {
    return 999; 
  }

  return (int)(duracion * 0.034 / 2);
}

int calcularErrorPared(int distMedida) {
  if (distMedida >= 999) {
    return 0; // Sin pared detectable, mantiene rumbo recto
  }
  return distMedida - DISTANCIA_DESEADA_PARED;
}