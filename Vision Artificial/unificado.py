import cv2 as cv
import numpy as np
import time

# Importamos las funciones de tus archivos existentes
from vision_general import TrackerObjetos, detectar_rectangulos_orientados, detectar_circulos, ESP32CamStream
from conexion_uart import ConexionUART

# --- SI USAS ESP32-CAM VÍA WI-FI LOCAL EN EL ROBOT ---
USAR_ESP32_CAM_STREAM = True  # Cambia a False si en el futuro conectas una Cámara USB
IP_ESP32 = "192.168.8.183"
URL_STREAM = f"http://{IP_ESP32}:81/stream"

# --- CONFIGURACIÓN DE PANTALLA ---
ANCHO_FRAME = 640
ALTO_FRAME = 480
CENTRO_X = ANCHO_FRAME // 2
CENTRO_Y = ALTO_FRAME // 2

UMBRAL_CENTRADO_X = 20
UMBRAL_CENTRADO_Y = 20

# --- RANGOS HSV ---
azul_lower = np.array([106, 38, 40])
azul_upper = np.array([125, 176, 200])

amarillo_lower = np.array([10, 35, 134])
amarillo_upper = np.array([33, 108, 270])

rojo_lower_1 = np.array([167, 90, 55])
rojo_upper_1 = np.array([179, 194, 200])
naranja_lower = np.array([171, 96, 70])
naranja_upper = np.array([179, 255, 255])

black_lower = np.array([120, 50, 15])
black_upper = np.array([180, 130, 100])

# --- INICIALIZACIÓN ---
uart = ConexionUART(puerto='/dev/ttyS0', baudrate=115200)

if USAR_ESP32_CAM_STREAM:
    stream = ESP32CamStream(URL_STREAM, IP_ESP32)
    stream.start()
else:
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, ANCHO_FRAME)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, ALTO_FRAME)

tracker = TrackerObjetos(tiempo_confirmacion=1.0)
modo_actual = "PELOTAS"

print(f"[Master Visión] Iniciado en modo: {modo_actual}")

try:
    while True:
        # 1. Leer comandos de cambio de modo desde la ESP32 de motores
        nuevo_modo = uart.leer_modo_esp32()
        if nuevo_modo in ["PISCINAS", "PELOTAS", "CONTENEDORES"]:
            modo_actual = nuevo_modo
            print(f"[Master Visión] Modo cambiado a: {modo_actual}")

        # 2. Capturar frame según el tipo de cámara
        if USAR_ESP32_CAM_STREAM:
            has_frame, frame = stream.read()
        else:
            has_frame, frame = cap.read()

        if not has_frame or frame is None:
            time.sleep(0.01)
            continue

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        hsv = cv.GaussianBlur(hsv, (5, 5), 0)

        detecciones = []

        # 3. Procesar según el modo ordenado por la ESP32
        if modo_actual == "PISCINAS":
            mascara_azul = cv.inRange(hsv, azul_lower, azul_upper)
            detecciones = detectar_rectangulos_orientados(
                mascara_azul, "Piscina", area_min=1200, aspect_ratio_range=(2.0, 5.0), solidez_min=0.65
            )

        elif modo_actual == "PELOTAS":
            mascara_azul = cv.inRange(hsv, azul_lower, azul_upper)
            mascara_amarillo = cv.inRange(hsv, amarillo_lower, amarillo_upper)
            mascara_rojo = cv.bitwise_or(
                cv.inRange(hsv, rojo_lower_1, rojo_upper_1),
                cv.inRange(hsv, naranja_lower, naranja_upper)
            )
            mascara_negra = cv.inRange(hsv, black_lower, black_upper)

            detecciones += detectar_circulos(mascara_azul, "Pelota Azul", 150, 0.77)
            detecciones += detectar_circulos(mascara_amarillo, "Pelota Amarilla", 150, 0.77)
            detecciones += detectar_circulos(mascara_rojo, "Pelota Roja", 150, 0.77)
            detecciones += detectar_circulos(mascara_negra, "Pelota Negra", 150, 0.77)

        elif modo_actual == "CONTENEDORES":
            mascara_azul = cv.inRange(hsv, azul_lower, azul_upper)
            mascara_roja = cv.bitwise_or(
                cv.inRange(hsv, rojo_lower_1, rojo_upper_1),
                cv.inRange(hsv, naranja_lower, naranja_upper)
            )

            detecciones += detectar_rectangulos_orientados(
                mascara_azul, "Contenedor Azul", 1000, (0.8, 1.25), 0.70
            )
            detecciones += detectar_rectangulos_orientados(
                mascara_roja, "Contenedor Rojo", 1000, (0.8, 1.25), 0.70
            )

        # 4. Actualizar seguimiento de objetos
        ahora = time.monotonic()
        detecciones = tracker.actualizar(detecciones, ahora)

        # 5. Calcular desviación y enviar coordenadas a la ESP32 por UART
        if detecciones:
            obj_target = detecciones[0]
            cx, cy = obj_target["centro"]

            error_x = cx - CENTRO_X
            error_y = cy - CENTRO_Y
            centrado = (abs(error_x) <= UMBRAL_CENTRADO_X) and (abs(error_y) <= UMBRAL_CENTRADO_Y)

            uart.enviar_error_camara(error_x, error_y, centrado)
        else:
            uart.enviar_error_camara(0, 0, False)

        time.sleep(0.03)

finally:
    if USAR_ESP32_CAM_STREAM:
        stream.stop()
    else:
        cap.release()
    uart.cerrar()