# ============================================================================
# ARCHIVO: vision_master.py
# DESCRIPCIÓN: Controlador principal de Visión en Raspberry Pi 4 (Cámara USB)
# ============================================================================
import cv2 as cv
import time

from vision_general2 import (
    detectar_circulos,
    detectar_follaje_arbol,
    detectar_rectangulos_orientados
)
from parametros import leer_float, leer_hsv, leer_int
from cone_UART import ConexionUART

# --- CONFIGURACIÓN DE RENDIMIENTO E INTERFAZ ---
# Cambiar a True si deseas ver las ventanas gráficas durante pruebas de desarrollo
MOSTRAR_VIDEO = False

# --- CONFIGURACIÓN DE CÁMARA USB HD 1080P ---
ANCHO_FRAME = 640
ALTO_FRAME = 480
CENTRO_X = ANCHO_FRAME // 2
CENTRO_Y = ALTO_FRAME // 2

# Exposición y balance de blancos fijos (definidos en .env)
EXPOSICION = leer_int("EXPOSICION")
TEMPERATURA_BLANCOS = leer_int("TEMPERATURA_BLANCOS")

TOLERANCIA_CENTRADO_X = 18        # Tolerancia de centrado en píxeles
TOLERANCIA_CENTRADO_ARBOL_X = 30  # El follaje es más grande, se permite más margen

# --- FILTRADO TEMPORAL ---
ALFA_FILTRO = 0.5        # Suavizado del error (1.0 = sin filtro, menor = más suave)
FRAMES_CONFIRMACION = 3  # Frames seguidos con detección antes de darla por válida

# --- RANGOS DE COLOR HSV (definidos en .env, actualizar según calibrar_vision.py) ---
verde_arbol_lower = leer_hsv("VERDE_ARBOL_LOWER")
verde_arbol_upper = leer_hsv("VERDE_ARBOL_UPPER")

azul_lower = leer_hsv("AZUL_LOWER")
azul_upper = leer_hsv("AZUL_UPPER")

amarillo_lower = leer_hsv("AMARILLO_LOWER")
amarillo_upper = leer_hsv("AMARILLO_UPPER")

rojo_lower1 = leer_hsv("ROJO_LOWER1")
rojo_upper1 = leer_hsv("ROJO_UPPER1")
rojo_lower2 = leer_hsv("ROJO_LOWER2")
rojo_upper2 = leer_hsv("ROJO_UPPER2")

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

# --- INICIALIZACIÓN ---
cap = cv.VideoCapture(0)
if not cap.isOpened():
    raise SystemExit("[VISION MASTER] Error: no se pudo abrir la cámara.")
cap.set(cv.CAP_PROP_FRAME_WIDTH, ANCHO_FRAME)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, ALTO_FRAME)
cap.set(cv.CAP_PROP_BUFFERSIZE, 1)     # Solo el frame más reciente, sin retraso acumulado
cap.set(cv.CAP_PROP_AUTO_EXPOSURE, 1)  # 1 = manual en V4L2
cap.set(cv.CAP_PROP_EXPOSURE, EXPOSICION)
cap.set(cv.CAP_PROP_AUTO_WB, 0)
cap.set(cv.CAP_PROP_WB_TEMPERATURE, TEMPERATURA_BLANCOS)

uart = ConexionUART(puerto='/dev/serial0', baudrate=115200)

print("[VISION MASTER] Sistema iniciado a 640x480. Esperando órdenes UART de ESP32...")
print(f"[VISION MASTER] Modo gráfico activo: {MOSTRAR_VIDEO}")

modo_anterior = None
frames_detectado = 0
err_x_filtrado, err_y_filtrado = 0.0, 0.0

try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret or frame is None:
            time.sleep(0.01)
            continue

        # 1. Leer Modo de Visión solicitado por ESP32
        modo_activo = uart.leer_estado_esp32()
        if modo_activo != modo_anterior:
            frames_detectado = 0  # Nuevo modo: descartar el historial del objetivo anterior
            modo_anterior = modo_activo

        # 2. Preprocesamiento de imagen
        blur = cv.GaussianBlur(frame, (5, 5), 0)
        hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

        err_x, err_y = 0, 0
        detectado = False
        tolerancia_x = TOLERANCIA_CENTRADO_X

        # 3. PROCESAMIENTO SEGÚN EL ESTADO DE NAVEGACIÓN
        if modo_activo == "ARBOLES":
            tolerancia_x = TOLERANCIA_CENTRADO_ARBOL_X
            mascara_verde = cv.inRange(hsv, verde_arbol_lower, verde_arbol_upper)
            arbol = detectar_follaje_arbol(mascara_verde, area_min_follaje=ARBOL_AREA_MIN)

            if arbol:
                cx, cy = arbol["centro"]
                err_x = cx - CENTRO_X
                err_y = cy - CENTRO_Y
                detectado = True
                if MOSTRAR_VIDEO:
                    cv.circle(frame, (cx, cy), 10, (0, 255, 0), -1)

        elif modo_activo == "PELOTAS":
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
                mascara_cosechables, "Pelota Cosechable", area_min=PELOTA_AREA_MIN, area_max=PELOTA_AREA_MAX,
                circularidad_min=PELOTA_CIRCULARIDAD_MIN, relleno_min=PELOTA_RELLENO_MIN
            )

            if pelotas:
                pelota_objetivo = min(pelotas, key=lambda p: abs(p["centro"][0] - CENTRO_X))
                cx, cy = pelota_objetivo["centro"]

                err_x = cx - CENTRO_X
                err_y = cy - CENTRO_Y
                detectado = True

                if MOSTRAR_VIDEO:
                    cv.circle(frame, (cx, cy), pelota_objetivo["radio"], (0, 255, 255), 2)
                    cv.line(frame, (CENTRO_X, CENTRO_Y), (cx, cy), (255, 0, 0), 2)

        elif modo_activo == "PISCINAS":
            mascara_azul = cv.inRange(hsv, azul_lower, azul_upper)
            piscinas = detectar_rectangulos_orientados(
                mascara_azul, "Piscina", area_min=PISCINA_AREA_MIN, aspect_ratio_range=PISCINA_ASPECT_RANGE,
                solidez_min=SOLIDEZ_MIN
            )
            if piscinas:
                # La de mayor área es la más cercana
                cx, cy = max(piscinas, key=lambda p: p["area"])["centro"]
                err_x = cx - CENTRO_X
                detectado = True

        elif modo_activo == "CONTENEDORES":
            # Combinar máscaras azul y roja para detectar cualquiera de los dos depósitos
            mascara_azul = cv.inRange(hsv, azul_lower, azul_upper)
            mascara_roja = cv.bitwise_or(
                cv.inRange(hsv, rojo_lower1, rojo_upper1),
                cv.inRange(hsv, rojo_lower2, rojo_upper2)
            )
            mascara_contenedores = cv.bitwise_or(mascara_azul, mascara_roja)

            contenedores = detectar_rectangulos_orientados(
                mascara_contenedores, "Contenedor", area_min=CONTENEDOR_AREA_MIN,
                aspect_ratio_range=CONTENEDOR_ASPECT_RANGE, solidez_min=SOLIDEZ_MIN
            )
            if contenedores:
                # El de mayor área es el más cercano
                cx, cy = max(contenedores, key=lambda c: c["area"])["centro"]
                err_x = cx - CENTRO_X
                err_y = cy - CENTRO_Y
                detectado = True

        # 4. Filtrado temporal: confirmar la detección y suavizar el error
        if detectado:
            frames_detectado += 1
            if frames_detectado == 1:
                # Objetivo nuevo: arrancar el filtro sin arrastrar el error anterior
                err_x_filtrado, err_y_filtrado = err_x, err_y
            else:
                err_x_filtrado = ALFA_FILTRO * err_x + (1 - ALFA_FILTRO) * err_x_filtrado
                err_y_filtrado = ALFA_FILTRO * err_y + (1 - ALFA_FILTRO) * err_y_filtrado
        else:
            frames_detectado = 0

        confirmado = frames_detectado >= FRAMES_CONFIRMACION
        if confirmado:
            err_x, err_y = int(err_x_filtrado), int(err_y_filtrado)
            esta_centrado = (abs(err_x) <= tolerancia_x)
        else:
            err_x, err_y = 0, 0
            esta_centrado = False

        # 5. Transmitir error, estado de centrado y detección a ESP32 por UART
        uart.enviar_error_camara(err_x, err_y, esta_centrado, confirmado)

        # Visualización local opcional
        if MOSTRAR_VIDEO:
            cv.putText(frame, f"Modo: {modo_activo} | ErrX: {err_x} | Centrado: {esta_centrado}", 
                       (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv.imshow("Vision Raspberry Pi 4", frame)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            time.sleep(0.01)

finally:
    cap.release()
    if MOSTRAR_VIDEO:
        cv.destroyAllWindows()
    uart.cerrar()