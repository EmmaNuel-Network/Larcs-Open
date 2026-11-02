 #============================================================================
# ARCHIVO: probador_vision_todo.py
# DESCRIPCIÓN: Script de prueba simultánea para visualizar TODAS las detecciones
#              (Árboles, Pelotas, Piscinas y Contenedores) en tiempo real,
#              sin necesidad de conexión UART ni ESP32.
# ============================================================================
import cv2 as cv
import time

from vision_general2 import (
    detectar_circulos,
    detectar_follaje_arbol,
    detectar_rectangulos_orientados
)
from parametros import leer_float, leer_hsv, leer_int

# --- CONFIGURACIÓN DE CÁMARA USB HD 1080P ---
ANCHO_FRAME = 640
ALTO_FRAME = 480
CENTRO_X = ANCHO_FRAME // 2
CENTRO_Y = ALTO_FRAME // 2

# Exposición y balance de blancos fijos (definidos en .env)
EXPOSICION = leer_int("EXPOSICION")
TEMPERATURA_BLANCOS = leer_int("TEMPERATURA_BLANCOS")

# --- RANGOS DE COLOR HSV (definidos en .env, actualizar según calibrar_vision.py) ---
# Árbol (Verde)
verde_arbol_lower = leer_hsv("VERDE_ARBOL_LOWER")
verde_arbol_upper = leer_hsv("VERDE_ARBOL_UPPER")

# Piscinas y Contenedores Azules
azul_lower = leer_hsv("AZUL_LOWER")
azul_upper = leer_hsv("AZUL_UPPER")

# Pelotas Amarillas
amarillo_lower = leer_hsv("AMARILLO_LOWER")
amarillo_upper = leer_hsv("AMARILLO_UPPER")

# Pelotas Rojas / Contenedores Rojos (Rango dividido en HSV)
rojo_lower1 = leer_hsv("ROJO_LOWER1")
rojo_upper1 = leer_hsv("ROJO_UPPER1")
rojo_lower2 = leer_hsv("ROJO_LOWER2")
rojo_upper2 = leer_hsv("ROJO_UPPER2")

# Pelotas Sobremaduras (Negras)
negro_lower = leer_hsv("NEGRO_LOWER")
negro_upper = leer_hsv("NEGRO_UPPER")

# --- PARÁMETROS DE DETECCIÓN (definidos en .env) ---
ARBOL_AREA_MIN = leer_int("ARBOL_AREA_MIN")
PELOTA_AREA_MIN = leer_int("PELOTA_AREA_MIN")
PELOTA_AREA_MAX = leer_int("PELOTA_AREA_MAX")
PELOTA_CIRCULARIDAD_MIN = leer_float("PELOTA_CIRCULARIDAD_MIN")
PELOTA_RELLENO_MIN = leer_float("PELOTA_RELLENO_MIN")
PISCINA_AREA_MIN = leer_int("PISCINA_AREA_MIN")
PISCINA_ASPECT_RANGE = (leer_float("PISCINA_ASPECT_MIN"), leer_float("PISCINA_ASPECT_MAX"))
CONTENEDOR_AREA_MIN = leer_int("CONTENEDOR_AREA_MIN")
CONTENEDOR_ASPECT_RANGE = (leer_float("CONTENEDOR_ASPECT_MIN"), leer_float("CONTENEDOR_ASPECT_MAX"))
SOLIDEZ_MIN = leer_float("SOLIDEZ_MIN")

# --- INICIALIZACIÓN DE CÁMARA ---
cap = cv.VideoCapture(0)
if not cap.isOpened():
    raise SystemExit("[PROBADOR] Error: no se pudo abrir la cámara.")
cap.set(cv.CAP_PROP_FRAME_WIDTH, ANCHO_FRAME)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, ALTO_FRAME)
cap.set(cv.CAP_PROP_BUFFERSIZE, 1)     # Solo el frame más reciente, sin retraso acumulado
cap.set(cv.CAP_PROP_AUTO_EXPOSURE, 1)  # 1 = manual en V4L2
cap.set(cv.CAP_PROP_EXPOSURE, EXPOSICION)
cap.set(cv.CAP_PROP_AUTO_WB, 0)
cap.set(cv.CAP_PROP_WB_TEMPERATURE, TEMPERATURA_BLANCOS)

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
        arbol = detectar_follaje_arbol(mascara_verde, area_min_follaje=ARBOL_AREA_MIN)

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
            mascara_cosechables, "Pelota", area_min=PELOTA_AREA_MIN, area_max=PELOTA_AREA_MAX,
            circularidad_min=PELOTA_CIRCULARIDAD_MIN, relleno_min=PELOTA_RELLENO_MIN
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
            mascara_azul, "Piscina", area_min=PISCINA_AREA_MIN, aspect_ratio_range=PISCINA_ASPECT_RANGE,
            solidez_min=SOLIDEZ_MIN
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
            mascara_contenedores, "Contenedor", area_min=CONTENEDOR_AREA_MIN,
            aspect_ratio_range=CONTENEDOR_ASPECT_RANGE, solidez_min=SOLIDEZ_MIN
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