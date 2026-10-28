 #============================================================================
# ARCHIVO: probador_vision_todo.py
# DESCRIPCIÓN: Script de prueba simultánea para visualizar TODAS las detecciones
#              (Árboles, Pelotas, Piscinas y Contenedores) en tiempo real,
#              sin necesidad de conexión UART ni ESP32.
# ============================================================================
import cv2 as cv
import numpy as np
import time

from vision_general2 import (
    detectar_circulos,
    detectar_follaje_arbol,
    detectar_rectangulos_orientados
)

# --- CONFIGURACIÓN DE CÁMARA USB HD 1080P ---
ANCHO_FRAME = 640
ALTO_FRAME = 480
CENTRO_X = ANCHO_FRAME // 2
CENTRO_Y = ALTO_FRAME // 2

# --- RANGOS DE COLOR HSV (Actualizar según calibrar_vision.py) ---
# Árbol (Verde)
verde_arbol_lower = np.array([35, 40, 40])
verde_arbol_upper = np.array([85, 255, 255])

# Piscinas y Contenedores Azules
azul_lower = np.array([100, 80, 50])
azul_upper = np.array([130, 255, 255])

# Pelotas Amarillas
amarillo_lower = np.array([15, 80, 100])
amarillo_upper = np.array([35, 255, 255])

# Pelotas Rojas / Contenedores Rojos (Rango dividido en HSV)
rojo_lower1 = np.array([0, 100, 100])
rojo_upper1 = np.array([10, 255, 255])
rojo_lower2 = np.array([160, 100, 100])
rojo_upper2 = np.array([179, 255, 255])

# Pelotas Sobremaduras (Negras)
negro_lower = np.array([0, 0, 0])
negro_upper = np.array([180, 255, 45])

# --- INICIALIZACIÓN DE CÁMARA ---
cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, ANCHO_FRAME)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, ALTO_FRAME)

print("=" * 65)
print("[PROBADOR GLOBAL DE VISIÓN - DETECCIÓN SIMULTÁNEA]")
print(" Procesando Árboles, Pelotas, Piscinas y Contenedores a 640x480")
print(" Presiona 'q' para salir.")
print("=" * 65)

prev_time = time.time()

try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret or frame is None:
            time.sleep(0.01)
            continue

        # Preprocesamiento de la imagen
        blur = cv.GaussianBlur(frame, (5, 5), 0)
        hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

        # --------------------------------------------------------------------
        # 1. DETECCIÓN DE ÁRBOLES (Verde)
        # --------------------------------------------------------------------
        mascara_verde = cv.inRange(hsv, verde_arbol_lower, verde_arbol_upper)
        arbol = detectar_follaje_arbol(mascara_verde, area_min_follaje=15000)

        if arbol:
            cx, cy = arbol["centro"]
            # Dibujar contorno del follaje verde y centroide
            cv.drawContours(frame, [arbol["contorno"]], -1, (0, 255, 0), 2)
            cv.circle(frame, (cx, cy), 8, (0, 255, 0), -1)
            cv.putText(frame, f"ARBOL ({int(arbol['area'])})", (cx - 40, cy - 15),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # --------------------------------------------------------------------
        # 2. DETECCIÓN DE PELOTAS (Rojas, Amarillas, Negras, Azules)
        # --------------------------------------------------------------------
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
            mascara_cosechables, "Pelota", area_min=600, area_max=32000, circularidad_min=0.75
        )

        for p in pelotas:
            cx, cy = p["centro"]
            r = p["radio"]
            cv.circle(frame, (cx, cy), r, (0, 255, 255), 2)  # Círculo amarillo/amarillo claro
            cv.circle(frame, (cx, cy), 3, (0, 0, 255), -1)
            cv.putText(frame, f"PELOTA ({cx},{cy})", (cx - 30, cy - r - 5),
                       cv.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        # --------------------------------------------------------------------
        # 3. DETECCIÓN DE PISCINAS (Azules y Rectangulares: aspect 2.0 - 5.0)
        # --------------------------------------------------------------------
        piscinas = detectar_rectangulos_orientados(
            mascara_azul, "Piscina", area_min=4800, aspect_ratio_range=(2.0, 5.0)
        )

        for pisc in piscinas:
            box = pisc["box"]
            cx, cy = pisc["centro"]
            cv.drawContours(frame, [box], 0, (255, 100, 0), 2)  # Azul/Naranja
            cv.circle(frame, (cx, cy), 5, (255, 100, 0), -1)
            cv.putText(frame, "PISCINA", (cx - 30, cy),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 2)

        # --------------------------------------------------------------------
        # 4. DETECCIÓN DE CONTENEDORES (Azul o Rojo y Cuadrados: aspect 0.8 - 1.25)
        # --------------------------------------------------------------------
        mascara_contenedores = cv.bitwise_or(mascara_azul, mascara_roja)
        contenedores = detectar_rectangulos_orientados(
            mascara_contenedores, "Contenedor", area_min=4000, aspect_ratio_range=(0.8, 1.25)
        )

        for cont in contenedores:
            box = cont["box"]
            cx, cy = cont["centro"]
            cv.drawContours(frame, [box], 0, (255, 0, 255), 2)  # Magenta / Rosa
            cv.circle(frame, (cx, cy), 5, (255, 0, 255), -1)
            cv.putText(frame, "CONTENEDOR", (cx - 45, cy),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

        # --------------------------------------------------------------------
        # 5. INFORMACIÓN DE PANTALLA (HUD) Y LÍNEAS DE CENTRO
        # --------------------------------------------------------------------
        # Líneas de referencia del centro de la cámara
        cv.line(frame, (CENTRO_X, 0), (CENTRO_X, ALTO_FRAME), (100, 100, 100), 1)
        cv.line(frame, (0, CENTRO_Y), (ANCHO_FRAME, CENTRO_Y), (100, 100, 100), 1)

        # Cálculo de FPS
        curr_time = time.time()
        fps = 1 / (curr_time - prev_time) if (curr_time - prev_time) > 0 else 0
        prev_time = curr_time

        # Leyenda de conteo en la parte superior
        info_text = f"FPS: {int(fps)} | Pelotas: {len(pelotas)} | Piscinas: {len(piscinas)} | Contenedores: {len(contenedores)} | Arbol: {'SI' if arbol else 'NO'}"
        cv.putText(frame, info_text, (10, 25), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Mostrar video en vivo
        cv.imshow("Probador Global de Vision (Todos los Objetos)", frame)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv.destroyAllWindows()