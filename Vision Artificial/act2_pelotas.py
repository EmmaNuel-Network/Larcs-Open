import cv2 as cv
import numpy as np
import time

# Importaciones del módulo común y de comunicación
from vision_general import ESP32CamStream, TrackerObjetos, detectar_circulos
from conexion_esp import enviar_datos_a_esp32

# Configuración de conexión
IP_ESP32 = "192.168.8.125"  # Usa "192.168.4.1" si la ESP32 está en modo Punto de Acceso (SoftAP)
URL_STREAM = f"http://{IP_ESP32}:81/stream"

# --- RANGOS HSV POR COLOR ---
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

# --- CRITERIOS DE GEOMETRÍA Y TIEMPOS ---
AREA_MINIMA = 150
CIRCULARIDAD_MINIMA = 0.77
INTERVALO_ACTUALIZACION = 3.0

# --- INICIALIZACIÓN DE SERVICIOS COMUNES ---
stream = ESP32CamStream(URL_STREAM, IP_ESP32)
stream.start()

tracker = TrackerObjetos(
    tiempo_confirmacion=3.0,
    tiempo_maximo_perdida=1.0,
    dist_maxima=80
)

nombre_ventana = 'Deteccion de Pelotas'
cv.namedWindow(nombre_ventana, cv.WINDOW_NORMAL)
ultima_actualizacion = time.monotonic()

print("Servicio de detección de pelotas iniciado. Presiona ESC para salir.")

try:
    while cv.waitKey(1) != 27:
        has_frame, frame = stream.read()

        if not has_frame or frame is None:
            time.sleep(0.005)
            continue

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        hsv = cv.GaussianBlur(hsv, (5, 5), 0)

        # Generación de máscaras
        mascara_azul = cv.inRange(hsv, azul_lower, azul_upper)
        mascara_amarillo = cv.inRange(hsv, amarillo_lower, amarillo_upper)
        mascara_rojo = cv.bitwise_or(
            cv.inRange(hsv, rojo_lower_1, rojo_upper_1),
            cv.inRange(hsv, naranja_lower, naranja_upper)
        )
        mascara_negra = cv.inRange(hsv, black_lower, black_upper)

        # Detecciones utilizando la función importada de vision_common
        detecciones = []
        detecciones += detectar_circulos(mascara_azul, "Pelota azul", AREA_MINIMA, CIRCULARIDAD_MINIMA)
        detecciones += detectar_circulos(mascara_amarillo, "Pelota amarilla", AREA_MINIMA, CIRCULARIDAD_MINIMA)
        detecciones += detectar_circulos(mascara_rojo, "Pelota roja", AREA_MINIMA, CIRCULARIDAD_MINIMA)
        detecciones += detectar_circulos(mascara_negra, "Pelota negra", AREA_MINIMA, CIRCULARIDAD_MINIMA)

        ahora = time.monotonic()
        detecciones = tracker.actualizar(detecciones, ahora)

        # Impresión y envío UDP de coordenadas confirmadas
        if ahora - ultima_actualizacion >= INTERVALO_ACTUALIZACION:
            tracker.imprimir_confirmados()

            for s_id, data in tracker.coordenadas_xy.items():
                x, y = data["xy"]
                enviar_datos_a_esp32(piscina_id=s_id, x=x, y=y, angulo=0.0)

            ultima_actualizacion = ahora

        # Colores BGR para dibujar en la interfaz
        colores_dibujo = {
            "Pelota azul": (255, 0, 0),
            "Pelota amarilla": (0, 255, 255),
            "Pelota roja": (0, 0, 255),
            "Pelota negra": (0, 0, 0)
        }

        # Dibujar resultados sobre la imagen
        for d in detecciones:
            x, y = d["centro"]
            r = d["radio"]
            s_id = d["seguimiento_id"]
            tipo = d["tipo"]
            confirmada = d["confirmada"]

            color_bgr = colores_dibujo.get(tipo, (255, 255, 255))
            estado_str = "CONFIRMADA" if confirmada else "VERIFICANDO..."

            cv.circle(frame, (x, y), r, color_bgr, 3)
            cv.circle(frame, (x, y), 4, (255, 255, 255), -1)

            cv.putText(
                frame,
                f"{tipo} #{s_id} ({estado_str})",
                (x - r, y - r - 10),
                cv.FONT_HERSHEY_SIMPLEX,
                0.5,
                color_bgr,
                2
            )

        cv.putText(
            frame,
            f"Pelotas detectadas: {len(detecciones)}",
            (10, 25),
            cv.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        cv.imshow(nombre_ventana, frame)

finally:
    stream.stop()
    cv.destroyAllWindows()