import serial
import time

class ConexionUART:
    def __init__(self, puerto='/dev/ttyS0', baudrate=115200):
        try:
            self.ser = serial.Serial(puerto, baudrate, timeout=0.1)
            print(f"[UART] Conectado exitosamente a la ESP32 en {puerto}")
        except Exception as e:
            print(f"[UART] Error al abrir puerto serial: {e}")
            self.ser = None

    def enviar_error_camara(self, err_x, err_y, centrado):
        """
        Envía la trama formateada que espera Robot-Larc.ino: "errX,errY,centrado\n"
        """
        if self.ser and self.ser.is_open:
            val_centrado = 1 if centrado else 0
            mensaje = f"{int(err_x)},{int(err_y)},{val_centrado}\n"
            try:
                self.ser.write(mensaje.encode('utf-8'))
            except Exception as e:
                print(f"[UART] Error al enviar trama: {e}")

    def leer_modo_esp32(self):
        """
        Lee los comandos enviados por la ESP32 (PISCINAS, PELOTAS, CONTENEDORES)
        """
        if self.ser and self.ser.in_waiting > 0:
            try:
                linea = self.ser.readline().decode('utf-8').strip()
                return linea
            except Exception:
                pass
        return None

    def cerrar(self):
        if self.ser:
            self.ser.close()