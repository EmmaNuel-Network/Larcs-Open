# ============================================================================
# ARCHIVO: todo.py
# DESCRIPCIÓN: Script de prueba simultánea para visualizar TODAS las detecciones
#              (Árboles, Pelotas Maduras vs Sobremaduras, Piscinas y Contenedores Rojo/Azul)
#              en tiempo real vía TightVNC sin necesidad de conexión UART ni ESP32.
# ============================================================================
import cv2 as cv
import time

from vision_general2 import (
    detectar_circulos,
    detectar_follaje_arbol,
    detectar_rectangulos_orientados
)
from parametros import leer_float, leer_hsv, leer_hsv_opcional, leer_int

# --- CONFIGURACIÓN DE CÁMARA USB HD 1080P ---
ANCHO_FRAME = 640
ALTO_FRAME = 480
CENTRO_X = ANCHO_FRAME // 2
CENTRO_Y = ALTO_FRAME // 2

# Exposición y balance de blancos fijos (definidos en .env)
EXPOSICION = leer_int("EXPOSICION")
TEMPERATURA_BLANCOS = leer_int("TEMPERATURA_BLANCOS")

# --- RANGOS DE COLOR HSV (definidos en .env) ---
# Árbol (Verde)
verde_arbol_lower = leer_hsv("VERDE_ARBOL_LOWER")
verde_arbol_upper = leer_hsv("VERDE_ARBOL_UPPER")

# Piscinas y Contenedores Azules
azul_lower = leer_hsv("AZUL_LOWER")
azul_upper = leer_hsv("AZUL_UPPER")

# Pelotas Amarillas (Maduras)
amarillo_lower = leer_hsv("AMARILLO_LOWER")
amarillo_upper = leer_hsv("AMARILLO_UPPER")

# Pelotas Naranjas (Maduras)
naranja_lower = leer_hsv_opcional("NARANJA_LOWER", default=leer_hsv("AMARILLO_LOWER"))
naranja_upper = leer_hsv_opcional("NARANJA_UPPER", default=leer_hsv("AMARILLO_UPPER"))

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
print(" Procesando Árboles, Pelotas (Maduras/Sobremaduras), Piscinas y Contenedores")
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
        # 1. DETECCIÓN DE ÁRBOLES (Follaje Verde)
        # --------------------------------------------------------------------
        mascara_verde = cv.inRange(hsv, verde_arbol_lower, verde_arbol_upper)
        arbol = detectar_follaje_arbol(mascara_verde, area_min_follaje=ARBOL_AREA_MIN, id_tipo=5)

        if arbol:
            cx, cy = arbol["centro"]
            cv.drawContours(frame, [arbol["contorno"]], -1, (0, 255, 0), 2)
            cv.circle(frame, (cx, cy), 8, (0, 255, 0), -1)
            cv.putText(frame, f"ARBOL ({int(arbol['area'])})", (cx - 40, cy - 15),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # --------------------------------------------------------------------
        # 2. DETECCIÓN DE PELOTAS: MADURAS (Rojo/Naranja/Amarillo) vs SOBREMADURAS (Azul/Negro)
        # --------------------------------------------------------------------
        mascara_roja = cv.bitwise_or(
            cv.inRange(hsv, rojo_lower1, rojo_upper1),
            cv.inRange(hsv, rojo_lower2, rojo_upper2)
        )
        mascara_amarilla = cv.inRange(hsv, amarillo_lower, amarillo_upper)
        mascara_naranja = cv.inRange(hsv, naranja_lower, naranja_upper)
        mascara_maduras = cv.bitwise_or(cv.bitwise_or(mascara_roja, mascara_amarilla), mascara_naranja)

        mascara_negra = cv.inRange(hsv, negro_lower, negro_upper)
        mascara_azul = cv.inRange(hsv, azul_lower, azul_upper)
        mascara_sobremaduras = cv.bitwise_or(mascara_azul, mascara_negra)

        pelotas_maduras = detectar_circulos(
            mascara_maduras, "Madura", area_min=PELOTA_AREA_MIN, area_max=PELOTA_AREA_MAX,
            circularidad_min=PELOTA_CIRCULARIDAD_MIN, relleno_min=PELOTA_RELLENO_MIN,
            id_tipo=1, subtipo="MADURA"
        )
        pelotas_sobremaduras = detectar_circulos(
            mascara_sobremaduras, "Sobremadura", area_min=PELOTA_AREA_MIN, area_max=PELOTA_AREA_MAX,
            circularidad_min=PELOTA_CIRCULARIDAD_MIN, relleno_min=PELOTA_RELLENO_MIN,
            id_tipo=2, subtipo="SOBREMADURA"
        )

        for p in pelotas_maduras:
            cx, cy = p["centro"]
            r = p["radio"]
            cv.circle(frame, (cx, cy), r, (0, 215, 255), 2)  # Dorado / Amarillo-Naranja
            cv.circle(frame, (cx, cy), 3, (0, 140, 255), -1)
            cv.putText(frame, f"MADURA ({cx},{cy})", (cx - 35, cy - r - 5),
                       cv.FONT_HERSHEY_SIMPLEX, 0.45, (0, 215, 255), 1)

        for p in pelotas_sobremaduras:
            cx, cy = p["centro"]
            r = p["radio"]
            cv.circle(frame, (cx, cy), r, (255, 128, 0), 2)  # Azul claro / Cyan
            cv.circle(frame, (cx, cy), 3, (255, 0, 0), -1)
            cv.putText(frame, f"SOBREMADURA ({cx},{cy})", (cx - 45, cy - r - 5),
                       cv.FONT_HERSHEY_SIMPLEX, 0.45, (255, 128, 0), 1)

        # --------------------------------------------------------------------
        # 3. DETECCIÓN DE PISCINAS (Azules y Rectangulares: aspect 2.0 - 5.0)
        # --------------------------------------------------------------------
        piscinas = detectar_rectangulos_orientados(
            mascara_azul, "Piscina", area_min=PISCINA_AREA_MIN,
            aspect_ratio_range=PISCINA_ASPECT_RANGE, solidez_min=SOLIDEZ_MIN, id_tipo=6
        )

        for pisc in piscinas:
            box = pisc["box"]
            cx, cy = pisc["centro"]
            cv.drawContours(frame, [box], 0, (255, 200, 0), 2)
            cv.circle(frame, (cx, cy), 5, (255, 200, 0), -1)
            cv.putText(frame, "PISCINA", (cx - 30, cy),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 2)

        # --------------------------------------------------------------------
        # 4. DETECCIÓN DE CONTENEDORES (Cuadrados: Rojo vs Azul)
        # --------------------------------------------------------------------
        contenedores_rojos = detectar_rectangulos_orientados(
            mascara_roja, "Contenedor Rojo", area_min=CONTENEDOR_AREA_MIN,
            aspect_ratio_range=CONTENEDOR_ASPECT_RANGE, solidez_min=SOLIDEZ_MIN, id_tipo=3
        )
        contenedores_azules = detectar_rectangulos_orientados(
            mascara_azul, "Contenedor Azul", area_min=CONTENEDOR_AREA_MIN,
            aspect_ratio_range=CONTENEDOR_ASPECT_RANGE, solidez_min=SOLIDEZ_MIN, id_tipo=4
        )

        for cont in contenedores_rojos:
            box = cont["box"]
            cx, cy = cont["centro"]
            cv.drawContours(frame, [box], 0, (0, 0, 255), 2)  # Rojo
            cv.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv.putText(frame, "CONTENEDOR ROJO", (cx - 65, cy),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        for cont in contenedores_azules:
            box = cont["box"]
            cx, cy = cont["centro"]
            cv.drawContours(frame, [box], 0, (255, 0, 0), 2)  # Azul
            cv.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
            cv.putText(frame, "CONTENEDOR AZUL", (cx - 65, cy),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # --------------------------------------------------------------------
        # 5. INFORMACIÓN DE PANTALLA (HUD) Y LÍNEAS DE CENTRO
        # --------------------------------------------------------------------
        cv.line(frame, (CENTRO_X, 0), (CENTRO_X, ALTO_FRAME), (100, 100, 100), 1)
        cv.line(frame, (0, CENTRO_Y), (ANCHO_FRAME, CENTRO_Y), (100, 100, 100), 1)

        curr_time = time.time()
        fps = 1 / (curr_time - prev_time) if (curr_time - prev_time) > 0 else 0
        prev_time = curr_time

        info_text = f"FPS: {int(fps)} | Maduras: {len(pelotas_maduras)} | Sobremad: {len(pelotas_sobremaduras)} | C.Rojo: {len(contenedores_rojos)} | C.Azul: {len(contenedores_azules)}"
        cv.putText(frame, info_text, (10, 25), cv.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 2)

        cv.imshow("Probador Global de Vision (TightVNC)", frame)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv.destroyAllWindows()
