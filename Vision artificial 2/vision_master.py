# ============================================================================
# ARCHIVO: vision_master.py
# DESCRIPCIÓN: Controlador principal de Visión en Raspberry Pi 4 (Cámara USB)
# ============================================================================
import cv2 as cv
import numpy as np
import time

from vision_general2 import (
    detectar_circulos,
    detectar_follaje_arbol,
    detectar_rectangulos_orientados
)
from cone_UART import ConexionUART

# --- CONFIGURACIÓN DE CÁMARA USB HD 1080P ---
ANCHO_FRAME = 640
ALTO_FRAME = 480
CENTRO_X = ANCHO_FRAME // 2
CENTRO_Y = ALTO_FRAME // 2

TOLERANCIA_CENTRADO_X = 18  # Tolerancia de centrado en píxeles

# --- RANGOS DE COLOR HSV (Ajustar en calibrar_vision.py) ---
# Árbol (Verde)
verde_arbol_lower = np.array([35, 40, 40])
verde_arbol_upper = np.array([85, 255, 255])

# Piscinas y Contenedor Azul (Azul)
azul_lower = np.array([100, 80, 50])
azul_upper = np.array([130, 255, 255])

# Pelotas Maduras (Rojo / Naranja / Amarillo)
amarillo_lower = np.array([15, 80, 100])
amarillo_upper = np.array([35, 255, 255])

rojo_lower1 = np.array([0, 100, 100])
rojo_upper1 = np.array([10, 255, 255])
rojo_lower2 = np.array([160, 100, 100])
rojo_upper2 = np.array([179, 255, 255])

# Pelotas Sobremaduras (Negro)
negro_lower = np.array([0, 0, 0])
negro_upper = np.array([180, 255, 45])

# --- INICIALIZACIÓN ---
cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, ANCHO_FRAME)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, ALTO_FRAME)

uart = ConexionUART(puerto='/dev/ttyS0', baudrate=115200)

print("[VISION MASTER] Sistema iniciado. Esperando órdenes UART de ESP32...")

try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret or frame is None:
            time.sleep(0.01)
            continue

        # 1. Leer Modo de Visión solicitado por ESP32
        modo_activo = uart.leer_estado_esp32()

        # 2. Preprocesamiento de imagen
        blur = cv.GaussianBlur(frame, (5, 5), 0)
        hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

        err_x, err_y = 0, 0
        esta_centrado = False

        # 3. PROCESAMIENTO SEGÚN EL ESTADO DE NAVEGACIÓN
        if modo_activo == "ARBOLES":
            mascara_verde = cv.inRange(hsv, verde_arbol_lower, verde_arbol_upper)
            arbol = detectar_follaje_arbol(mascara_verde, area_min_follaje=15000)

            if arbol:
                cx, cy = arbol["centro"]
                err_x = cx - CENTRO_X
                err_y = cy - CENTRO_Y
                esta_centrado = (abs(err_x) <= 30)
                cv.circle(frame, (cx, cy), 10, (0, 255, 0), -1)

        elif modo_activo == "PELOTAS":
            # Segmentar solo pelotas cosechables (Maduras y Sobremaduras)
            mascara_roja = cv.bitwise_or(
                cv.inRange(hsv, rojo_lower1, rojo_upper1),
                cv.inRange(hsv, rojo_lower2, rojo_upper2)
            )
            mascara_amarilla = cv.inRange(hsv, amarillo_lower, amarillo_upper)
            mascara_negra = cv.inRange(hsv, negro_lower, negro_upper)
            mascara_azul = cv.inRange(hsv, azul_lower, azul_upper)

            mascara_cosechables = cv.bitwise_or(
                cv.bitwise_or(mascara_roja, mascara_amarilla),
                cv.bitwise_or(mascara_negra, mascara_azul)
            )

            pelotas = detectar_circulos(
                mascara_cosechables, "Pelota Cosechable", area_min=150, area_max=8000, circularidad_min=0.75
            )

            if pelotas:
                # Seleccionar la pelota más cercana al centro horizontal
                pelota_objetivo = min(pelotas, key=lambda p: abs(p["centro"][0] - CENTRO_X))
                cx, cy = pelota_objetivo["centro"]

                err_x = cx - CENTRO_X
                err_y = cy - CENTRO_Y
                esta_centrado = (abs(err_x) <= TOLERANCIA_CENTRADO_X)

                # Dibujar objetivo en pantalla
                cv.circle(frame, (cx, cy), pelota_objetivo["radio"], (0, 255, 255), 2)
                cv.line(frame, (CENTRO_X, CENTRO_Y), (cx, cy), (255, 0, 0), 2)

        elif modo_activo == "PISCINAS":
            mascara_azul = cv.inRange(hsv, azul_lower, azul_upper)
            piscinas = detectar_rectangulos_orientados(
                mascara_azul, "Piscina", area_min=1200, aspect_ratio_range=(2.0, 5.0)
            )
            if piscinas:
                cx, cy = piscinas[0]["centro"]
                err_x = cx - CENTRO_X

        elif modo_activo == "CONTENEDORES":
            mascara_azul = cv.inRange(hsv, azul_lower, azul_upper)
            contenedores = detectar_rectangulos_orientados(
                mascara_azul, "Contenedor Azul", area_min=1000, aspect_ratio_range=(0.8, 1.25)
            )
            if contenedores:
                cx, cy = contenedores[0]["centro"]
                err_x = cx - CENTRO_X

        # 4. Transmitir error y estado de centrado a ESP32 (Opción A)
        uart.enviar_error_camara(err_x, err_y, esta_centrado)

        # Visualización local para monitoreo
        cv.putText(frame, f"Modo: {modo_activo} | ErrX: {err_x} | Centrado: {esta_centrado}", 
                   (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv.imshow("Vision Raspberry Pi 4", frame)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.03) # ~30 FPS

finally:
    cap.release()
    cv.destroyAllWindows()
    uart.cerrar()