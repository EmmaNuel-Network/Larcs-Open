# ============================================================================
# ARCHIVO: cone_UART.py
# DESCRIPCIÓN: Controlador de comunicación serial bidireccional asíncrona
# ============================================================================
import serial
import time

class ConexionUART:
    def __init__(self, puerto='/dev/ttyS0', baudrate=115200):
        self.ser = None
        self.puerto = puerto
        self.baudrate = baudrate
        self.estado_actual_esp32 = "PISCINAS"  # Modo inicial por defecto
        self._conectar()

    def _conectar(self):
        try:
            self.ser = serial.Serial(self.puerto, self.baudrate, timeout=0.01)
            print(f"[UART] Conectado exitosamente a ESP32 en {self.puerto}")
        except Exception as e:
            print(f"[UART] Error al abrir puerto {self.puerto}: {e}")
            self.ser = None

    def enviar_error_camara(self, err_x, err_y, centrado):
        """Envía la trama formateada (Opción A): 'errX,errY,centrado\n'"""
        if self.ser and self.ser.is_open:
            val_centrado = 1 if centrado else 0
            mensaje = f"{int(err_x)},{int(err_y)},{val_centrado}\n"
            try:
                self.ser.write(mensaje.encode('utf-8'))
            except Exception as e:
                print(f"[UART] Error al transmitir trama: {e}")

    def leer_estado_esp32(self):
        """Lee mensajes entrantes enviados por la ESP32 para cambiar la FSM de Visión"""
        if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
            try:
                linea = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if linea in ["PISCINAS", "ARBOLES", "PELOTAS", "CONTENEDORES"]:
                    self.estado_actual_esp32 = linea
                    print(f"[UART] Cambio de estado notificado por ESP32: -> {linea}")
            except Exception as e:
                print(f"[UART] Error leyendo datos UART: {e}")
        return self.estado_actual_esp32

    def cerrar(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[UART] Puerto cerrado correctamente.")