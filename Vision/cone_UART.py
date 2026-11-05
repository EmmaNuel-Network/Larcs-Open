# ============================================================================
# ARCHIVO: cone_UART.py
# DESCRIPCIÓN: Controlador de comunicación serial bidireccional asíncrona
# ============================================================================
import serial
import time

# Códigos de tipo de objeto para sincronización con ESP32:
TIPO_NINGUNO = 0
TIPO_PELOTA_MADURA = 1       # Rojo / Naranja / Amarillo (Destino: Contenedor Rojo)
TIPO_PELOTA_SOBREMADURA = 2  # Azul / Negro (Destino: Contenedor Azul)
TIPO_CONTENEDOR_ROJO = 3     # Depósito Maduros
TIPO_CONTENEDOR_AZUL = 4     # Depósito Sobremaduros
TIPO_ARBOL = 5               # Árbol Follaje
TIPO_PISCINA = 6             # Piscina de lavado

MODOS_PERMITIDOS = [
    "PISCINAS", "ARBOLES", "PELOTAS", "CONTENEDORES",
    "MADURAS", "SOBREMADURAS", "CONTENEDOR_ROJO", "CONTENEDOR_AZUL"
]

class ConexionUART:
    def __init__(self, puerto='/dev/serial0', baudrate=115200):
        self.ser = None
        self.puerto = puerto
        self.baudrate = baudrate
        self.estado_actual_esp32 = "PELOTAS"   # Modo inicial por defecto (detecta pelotas cosechables)
        self._conectar()

    def _conectar(self):
        try:
            self.ser = serial.Serial(self.puerto, self.baudrate, timeout=0.01)
            print(f"[UART] Conectado exitosamente a ESP32 en {self.puerto}")
        except Exception as e:
            print(f"[UART] Error al abrir puerto {self.puerto}: {e}")
            self.ser = None

    def enviar_error_camara(self, err_x, err_y, centrado, detectado, id_tipo=0):
        """
        Envía la trama formateada: 'errX,errY,centrado,detectado,id_tipo\n'
        - Compatible hacia atrás si el Arduino lee solo los primeros campos con sscanf.
        - Permite al Arduino conocer el tipo exacto de objeto alineado.
        """
        if self.ser and self.ser.is_open:
            val_centrado = 1 if centrado else 0
            val_detectado = 1 if detectado else 0
            mensaje = f"{int(err_x)},{int(err_y)},{val_centrado},{val_detectado},{int(id_tipo)}\n"
            try:
                self.ser.write(mensaje.encode('utf-8'))
            except Exception as e:
                print(f"[UART] Error al transmitir trama: {e}")

    def leer_estado_esp32(self):
        """
        Lee mensajes entrantes enviados por la ESP32 para cambiar la FSM de Visión.
        Vacía todo el buffer para quedarse con el último modo y no con uno atrasado.
        """
        while self.ser and self.ser.is_open and self.ser.in_waiting > 0:
            try:
                linea = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if linea in MODOS_PERMITIDOS:
                    self.estado_actual_esp32 = linea
                    print(f"[UART] Cambio de estado notificado por ESP32: -> {linea}")
            except Exception as e:
                print(f"[UART] Error leyendo datos UART: {e}")
                break
        return self.estado_actual_esp32

    def cerrar(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[UART] Puerto cerrado correctamente.")
