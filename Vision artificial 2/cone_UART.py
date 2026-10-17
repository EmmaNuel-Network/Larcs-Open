import serial
import time

class ConexionUART:
    def __init__(self, puerto='/dev/ttyS0', baudrate=115200):
        try:
            # /dev/ttyS0 es el puerto UART por defecto en los pines GPIO de la Raspberry Pi
            self.ser = serial.Serial(puerto, baudrate, timeout=0.1)
            print(f"[UART] Conectado exitosamente a la ESP32 en {puerto}")
        except Exception as e:
            print(f"[UART] Error al abrir el puerto serial {puerto}: {e}")
            self.ser = None

    def enviar_error_camara(self, err_x, err_y, centrado):
        """
        Envía la trama formateada a la ESP32: "errX,errY,centrado\n"
        Ejemplo: "15,-5,1\n"
        """
        if self.ser and self.ser.is_open:
            val_centrado = 1 if centrado else 0
            mensaje = f"{int(err_x)},{int(err_y)},{val_centrado}\n"
            try:
                self.ser.write(mensaje.encode('utf-8'))
            except Exception as e:
                print(f"[UART] Error al transmitir trama: {e}")

    def cerrar(self):
        if self.ser:
            self.ser.close()