import cv2 as cv
import numpy as np
import time

# Importaciones de tu módulo de visión existente
from vision_general2 import ESP32CamStream, TrackerObjetos, detectar_rectangulos_orientados, detectar_circulos
from cone_UART import ConexionUART

# --- CONFIGURACIÓN DE CÁMARA ---
# Cambia a True si usas la ESP32-CAM por Wi-Fi, o False si usas una Cámara Web USB directa
USAR_ESP32_CAM = False  

IP_ESP32_CAM = "192.168.4.1"
URL_STREAM = f"http://{IP_ESP32_CAM}:81/stream"

ANCHO_FRAME = 640
ALTO_FRAME = 480
CENTRO_X = ANCHO_FRAME // 2
CENTRO_Y = ALTO_FRAME // 2

UMBRAL_CENTRADO_X = 20  # Tolerancia de centrado en píxeles
UMBRAL_CENTRADO_Y = 20

# --- RANGOS DE COLOR HSV ---
# Azules (Piscinas, Contenedores, Pelotas sobremaduras)
azul_lower = np.array([106, 38, 40])
azul_upper = np.array([125, 176, 200])

# Frutas Maduras (Rojo, Naranja y Amarillo agrupados)
amarillo_lower = np.array([10, 35, 134])
amarillo_upper = np.array([33, 108, 270])

rojo_lower_1 = np.array([167, 90, 55])
rojo_upper_1 = np.array([179, 194, 200])

naranja_lower = np.array([171, 96, 70])
naranja_upper = np.array([179, 255, 255])

# Frutas Sobremaduras (Negro)
black_lower = np.array([120, 50, 15])
black_upper = np.array([180, 130, 100])

# --- CRITERIOS GEOMÉTRICOS ---
# 1. Piscinas (720x200 mm -> Proporción ~3.6)
AREA_MIN_PISCINA = 1200
ASPECT_RATIO_PISCINA = (2.0, 5.0)

# 2. Contenedores (450x450 mm -> Proporción ~1.0)
AREA_MIN_CONTENEDOR = 1000
ASPECT_RATIO_CONTENEDOR = (0.8, 1.25)

# 3. Pelotas (Diámetro 42 mm)
AREA_MIN_PELOTA = 150
CIRCULARIDAD_MIN_PELOTA = 0.77

# --- INICIALIZACIÓN DE SERVICIOS ---
uart = ConexionUART(puerto='/dev/ttyS0', baudrate=115200)

if USAR_ESP32_CAM:
    stream = ESP32CamStream(URL_STREAM, IP_ESP32_CAM)
    stream.start()
else:
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, ANCHO_FRAME)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, ALTO_FRAME)

tracker = TrackerObjetos(tiempo_confirmacion=0.5)

print("[Visión Máster] Servicio de detección automática iniciado...")

try:
    while True:
        # 1. Captura de imagen
        if USAR_ESP32_CAM:
            has_frame, frame = stream.read()
        else:
            has_frame, frame = cap.read()

        if not has_frame or frame is None:
            time.sleep(0.01)
            continue

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        hsv = cv.GaussianBlur(hsv, (5, 5), 0)

        # 2. Máscaras de color
        mascara_azul = cv.inRange(hsv, azul_lower, azul_upper)
        
        # Unificación de colores maduros (Rojo + Naranja + Amarillo)
        mascara_maduros = cv.bitwise_or(
            cv.inRange(hsv, amarillo_lower, amarillo_upper),
            cv.bitwise_or(
                cv.inRange(hsv, rojo_lower_1, rojo_upper_1),
                cv.inRange(hsv, naranja_lower, naranja_upper)
            )
        )
        mascara_negra = cv.inRange(hsv, black_lower, black_upper)

        detecciones = []

        # 3. DETECCIÓN AUTOMÁTICA MULTIOBJETIVO
        
        # A. Detección de Pelotas (Maduras y Sobremaduras)
        detecciones += detectar_circulos(mascara_maduros, "Pelota Madura", AREA_MIN_PELOTA, CIRCULARIDAD_MIN_PELOTA)
        detecciones += detectar_circulos(mascara_azul, "Pelota Sobremadura", AREA_MIN_PELOTA, CIRCULARIDAD_MIN_PELOTA)
        detecciones += detectar_circulos(mascara_negra, "Pelota Sobremadura", AREA_MIN_PELOTA, CIRCULARIDAD_MIN_PELOTA)

        # B. Detección de Piscinas (Rectángulo largo azul)
        detecciones += detectar_rectangulos_orientados(
            mascara_azul, "Piscina", AREA_MIN_PISCINA, ASPECT_RATIO_PISCINA, solidez_min=0.65
        )

        # C. Detección de Contenedores (Cuadrados azul y rojo)
        detecciones += detectar_rectangulos_orientados(
            mascara_azul, "Contenedor Azul", AREA_MIN_CONTENEDOR, ASPECT_RATIO_CONTENEDOR, solidez_min=0.70
        )
        detecciones += detectar_rectangulos_orientados(
            mascara_maduros, "Contenedor Rojo", AREA_MIN_CONTENEDOR, ASPECT_RATIO_CONTENEDOR, solidez_min=0.70
        )

        # 4. Actualización del Tracker
        ahora = time.monotonic()
        detecciones = tracker.actualizar(detecciones, ahora)

        # 5. Envío de coordenadas y errores por UART a la ESP32
        if detecciones:
            # Seleccionar el primer objetivo detectado o el más cercano al centro
            target = detecciones[0]
            cx, cy = target["centro"]

            error_x = cx - CENTRO_X
            error_y = cy - CENTRO_Y
            centrado = (abs(error_x) <= UMBRAL_CENTRADO_X) and (abs(error_y) <= UMBRAL_CENTRADO_Y)

            uart.enviar_error_camara(error_x, error_y, centrado)
        else:
            # Si no hay objeto en vista, envía error 0
            uart.enviar_error_camara(0, 0, False)

        time.sleep(0.03) # Frecuencia de refresco ~30 FPS

finally:
    if USAR_ESP32_CAM:
        stream.stop()
    else:
        cap.release()
    uart.cerrar()