# ============================================================================
# ARCHIVO: vision_master.py
# DESCRIPCIÓN: Controlador principal de Visión en Raspberry Pi 4 (Cámara USB)
#              Distingue tipo de pelotas (Maduras vs Sobremaduras) y
#              tipo de contenedores (Rojo vs Azul) para la competencia LARC 2025.
# ============================================================================
import cv2 as cv
import time

from vision_general2 import (
    detectar_circulos,
    detectar_follaje_arbol,
    detectar_rectangulos_orientados
)
from parametros import leer_float, leer_hsv, leer_hsv_opcional, leer_int
from cone_UART import (
    ConexionUART,
    TIPO_NINGUNO,
    TIPO_PELOTA_MADURA,
    TIPO_PELOTA_SOBREMADURA,
    TIPO_CONTENEDOR_ROJO,
    TIPO_CONTENEDOR_AZUL,
    TIPO_ARBOL,
    TIPO_PISCINA
)

# --- CONFIGURACIÓN DE RENDIMIENTO E INTERFAZ ---
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

# --- RANGOS DE COLOR HSV (definidos en .env) ---
verde_arbol_lower = leer_hsv("VERDE_ARBOL_LOWER")
verde_arbol_upper = leer_hsv("VERDE_ARBOL_UPPER")

azul_lower = leer_hsv("AZUL_LOWER")
azul_upper = leer_hsv("AZUL_UPPER")

amarillo_lower = leer_hsv("AMARILLO_LOWER")
amarillo_upper = leer_hsv("AMARILLO_UPPER")

# Naranja (Maduro): si no está en .env, usa rango por defecto entre rojo1 y amarillo
naranja_lower = leer_hsv_opcional("NARANJA_LOWER", default=leer_hsv("AMARILLO_LOWER"))
naranja_upper = leer_hsv_opcional("NARANJA_UPPER", default=leer_hsv("AMARILLO_UPPER"))

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

        # 2. Preprocesamiento de imagen (5x5 para eliminar ruido de alta frecuencia)
        blur = cv.GaussianBlur(frame, (5, 5), 0)
        hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

        err_x, err_y = 0, 0
        detectado = False
        id_tipo_detectado = TIPO_NINGUNO
        tolerancia_x = TOLERANCIA_CENTRADO_X

        # --------------------------------------------------------------------
        # 3. PROCESAMIENTO SEGÚN EL ESTADO DE NAVEGACIÓN
        # --------------------------------------------------------------------
        if modo_activo == "ARBOLES":
            tolerancia_x = TOLERANCIA_CENTRADO_ARBOL_X
            mascara_verde = cv.inRange(hsv, verde_arbol_lower, verde_arbol_upper)
            arbol = detectar_follaje_arbol(mascara_verde, area_min_follaje=ARBOL_AREA_MIN, id_tipo=TIPO_ARBOL)

            if arbol:
                cx, cy = arbol["centro"]
                err_x = cx - CENTRO_X
                err_y = cy - CENTRO_Y
                detectado = True
                id_tipo_detectado = TIPO_ARBOL
                if MOSTRAR_VIDEO:
                    cv.circle(frame, (cx, cy), 10, (0, 255, 0), -1)

        elif modo_activo in ["PELOTAS", "MADURAS", "SOBREMADURAS"]:
            # Máscara de pelotas Maduras (Rojo + Naranja + Amarillo -> Contenedor Rojo)
            mascara_roja = cv.bitwise_or(
                cv.inRange(hsv, rojo_lower1, rojo_upper1),
                cv.inRange(hsv, rojo_lower2, rojo_upper2)
            )
            mascara_amarilla = cv.inRange(hsv, amarillo_lower, amarillo_upper)
            mascara_naranja = cv.inRange(hsv, naranja_lower, naranja_upper)
            mascara_maduras = cv.bitwise_or(cv.bitwise_or(mascara_roja, mascara_amarilla), mascara_naranja)

            # Máscara de pelotas Sobremaduras (Azul + Negro -> Contenedor Azul)
            mascara_azul = cv.inRange(hsv, azul_lower, azul_upper)
            mascara_negra = cv.inRange(hsv, negro_lower, negro_upper)
            mascara_sobremaduras = cv.bitwise_or(mascara_azul, mascara_negra)

            # Detección clasificada de esferas
            pelotas_maduras = detectar_circulos(
                mascara_maduras, "Pelota Madura", area_min=PELOTA_AREA_MIN, area_max=PELOTA_AREA_MAX,
                circularidad_min=PELOTA_CIRCULARIDAD_MIN, relleno_min=PELOTA_RELLENO_MIN,
                id_tipo=TIPO_PELOTA_MADURA, subtipo="MADURA"
            )
            pelotas_sobremaduras = detectar_circulos(
                mascara_sobremaduras, "Pelota Sobremadura", area_min=PELOTA_AREA_MIN, area_max=PELOTA_AREA_MAX,
                circularidad_min=PELOTA_CIRCULARIDAD_MIN, relleno_min=PELOTA_RELLENO_MIN,
                id_tipo=TIPO_PELOTA_SOBREMADURA, subtipo="SOBREMADURA"
            )

            # Selección de candidatos según el requerimiento de la ESP32
            if modo_activo == "MADURAS":
                candidatos = pelotas_maduras
            elif modo_activo == "SOBREMADURAS":
                candidatos = pelotas_sobremaduras
            else:  # "PELOTAS": Detecta ambas categorías
                candidatos = pelotas_maduras + pelotas_sobremaduras

            if candidatos:
                # Elige la pelota más cercana al centro óptico en X (alineación horizontal para el elevador)
                pelota_objetivo = min(candidatos, key=lambda p: abs(p["centro"][0] - CENTRO_X))
                cx, cy = pelota_objetivo["centro"]

                err_x = cx - CENTRO_X
                err_y = cy - CENTRO_Y
                detectado = True
                id_tipo_detectado = pelota_objetivo["id_tipo"]

                if MOSTRAR_VIDEO:
                    color_vis = (0, 255, 0) if id_tipo_detectado == TIPO_PELOTA_MADURA else (255, 0, 0)
                    cv.circle(frame, (cx, cy), pelota_objetivo["radio"], color_vis, 2)
                    cv.line(frame, (CENTRO_X, CENTRO_Y), (cx, cy), (255, 0, 0), 2)
                    cv.putText(frame, pelota_objetivo["subtipo"], (cx - 30, cy - pelota_objetivo["radio"] - 5),
                               cv.FONT_HERSHEY_SIMPLEX, 0.45, color_vis, 1)

        elif modo_activo == "PISCINAS":
            mascara_azul = cv.inRange(hsv, azul_lower, azul_upper)
            piscinas = detectar_rectangulos_orientados(
                mascara_azul, "Piscina", area_min=PISCINA_AREA_MIN, aspect_ratio_range=PISCINA_ASPECT_RANGE,
                solidez_min=SOLIDEZ_MIN, id_tipo=TIPO_PISCINA
            )
            if piscinas:
                # La de mayor área es la piscina más cercana
                piscina_objetivo = max(piscinas, key=lambda p: p["area"])
                cx, cy = piscina_objetivo["centro"]
                err_x = cx - CENTRO_X
                err_y = cy - CENTRO_Y
                detectado = True
                id_tipo_detectado = TIPO_PISCINA

        elif modo_activo in ["CONTENEDORES", "CONTENEDOR_ROJO", "CONTENEDOR_AZUL"]:
            # Discriminación estricta entre Contenedor Rojo y Contenedor Azul
            mascara_roja = cv.bitwise_or(
                cv.inRange(hsv, rojo_lower1, rojo_upper1),
                cv.inRange(hsv, rojo_lower2, rojo_upper2)
            )
            mascara_azul = cv.inRange(hsv, azul_lower, azul_upper)

            contenedores_rojos = detectar_rectangulos_orientados(
                mascara_roja, "Contenedor Rojo", area_min=CONTENEDOR_AREA_MIN,
                aspect_ratio_range=CONTENEDOR_ASPECT_RANGE, solidez_min=SOLIDEZ_MIN,
                id_tipo=TIPO_CONTENEDOR_ROJO
            )
            contenedores_azules = detectar_rectangulos_orientados(
                mascara_azul, "Contenedor Azul", area_min=CONTENEDOR_AREA_MIN,
                aspect_ratio_range=CONTENEDOR_ASPECT_RANGE, solidez_min=SOLIDEZ_MIN,
                id_tipo=TIPO_CONTENEDOR_AZUL
            )

            if modo_activo == "CONTENEDOR_ROJO":
                candidatos = contenedores_rojos
            elif modo_activo == "CONTENEDOR_AZUL":
                candidatos = contenedores_azules
            else:  # "CONTENEDORES": Evalúa ambos y reporta el más relevante
                candidatos = contenedores_rojos + contenedores_azules

            if candidatos:
                # El de mayor área visible es el contenedor objetivo
                cont_objetivo = max(candidatos, key=lambda c: c["area"])
                cx, cy = cont_objetivo["centro"]
                err_x = cx - CENTRO_X
                err_y = cy - CENTRO_Y
                detectado = True
                id_tipo_detectado = cont_objetivo["id_tipo"]

        # --------------------------------------------------------------------
        # 4. FILTRADO TEMPORAL Y VALIDACIÓN
        # --------------------------------------------------------------------
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
            id_tipo_detectado = TIPO_NINGUNO

        confirmado = frames_detectado >= FRAMES_CONFIRMACION
        if confirmado:
            err_x, err_y = int(err_x_filtrado), int(err_y_filtrado)
            esta_centrado = (abs(err_x) <= tolerancia_x)
            tipo_enviar = id_tipo_detectado
        else:
            err_x, err_y = 0, 0
            esta_centrado = False
            tipo_enviar = TIPO_NINGUNO

        # --------------------------------------------------------------------
        # 5. TRANSMISIÓN UART A ESP32: errX, errY, centrado, detectado, id_tipo
        # --------------------------------------------------------------------
        uart.enviar_error_camara(err_x, err_y, esta_centrado, confirmado, tipo_enviar)

        # Visualización local opcional para desarrollo
        if MOSTRAR_VIDEO:
            cv.putText(frame, f"Modo: {modo_activo} | ErrX: {err_x} | Tipo: {tipo_enviar} | Centrado: {esta_centrado}",
                       (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
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
