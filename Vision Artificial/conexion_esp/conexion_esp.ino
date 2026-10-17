// Estructura que comparten la ESP32-CAM y la ESP32 Receptora
typedef struct struct_mensaje {
    int id_piscina;
    int x;
    int y;
    float angulo;
    bool confirmada;
} struct_mensaje;

struct_mensaje datosEnviar;

// Ejemplo de envío vía ESP-NOW desde la ESP32-CAM:
void enviarCoordenadas(int id, int x, int y, float ang, bool conf) {
    datosEnviar.id_piscina = id;
    datosEnviar.x = x;
    datosEnviar.y = y;
    datosEnviar.angulo = ang;
    datosEnviar.confirmada = conf;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &datosEnviar, sizeof(datosEnviar));
}