import cv2 as cv
import numpy as np
import time

# Importaciones del archivo común y de conexión
from vision_general import ESP32CamStream, TrackerObjetos, detectar_rectangulos_orientados
from conexion_esp import enviar_datos_a_esp32

# IP de la ESP32 en modo Punto de Acceso (SoftAP)
IP_ESP32 = "192.168.4.1"
URL_STREAM = f"http://{IP_ESP32}:81/stream"

# Rango HSV para el azul predeterminado
azul_lower = np.array([106, 38, 40])
azul_upper = np.array([125, 176, 200])

# Criterios para la piscina (720 mm x 200 mm -> Proporción ~3.6)
AREA_MINIMA = 1200
ASPECT_RATIO_RANGE = (2.0, 5.0)
SOLIDEZ_MINIMA = 0.65

INTERVALO_ACTUALIZACION = 3.0

# Inicialización de servicios comunes
stream = ESP32CamStream(URL_STREAM, IP_ESP32)
stream.start()
tracker = TrackerObjetos(tiempo_confirmacion=3.0)

nombre_ventana = 'Deteccion de Piscina (Caja Azul)'
cv.namedWindow(nombre_ventana, cv.WINDOW_NORMAL)
ultima_actualizacion = time.monotonic()

print("Buscando piscina azul... Presiona ESC para salir.")

try:
    while cv.waitKey(1) != 27:
        has_frame, frame = stream.read()

        if not has_frame:
            pantalla_espera = np.zeros((480, 640, 3), dtype=np.uint8)
            cv.putText(
                pantalla_espera,
                "Reconectando a ESP32-CAM...",
                (120, 240),
                cv.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 165, 255),
                2
            )
            cv.imshow(nombre_ventana, pantalla_espera)
            continue

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        hsv = cv.GaussianBlur(hsv, (5, 5), 0)

        mascara_azul = cv.inRange(hsv, azul_lower, azul_upper)

        # Detección usando la función del archivo común
        detecciones = detectar_rectangulos_orientados(
            mascara_azul, "Piscina Azul", AREA_MINIMA, ASPECT_RATIO_RANGE, SOLIDEZ_MINIMA
        )

        ahora = time.monotonic()
        detecciones = tracker.actualizar(detecciones, ahora)

        # Envío UDP e impresión periódica
        if ahora - ultima_actualizacion >= INTERVALO_ACTUALIZACION:
            tracker.imprimir_confirmados()

            for s_id, data in tracker.coordenadas_xy.items():
                x, y = data["xy"]
                ang = data["angulo"]
                enviar_datos_a_esp32(piscina_id=s_id, x=x, y=y, angulo=ang)

            ultima_actualizacion = ahora

        # Dibujo de resultados
        for d in detecciones:
            box = d["box"]
            cx, cy = d["centro"]
            s_id = d["seguimiento_id"]
            confirmada = d["confirmada"]

            color_bgr = (0, 255, 0) if confirmada else (0, 255, 255)
            estado_str = "CONFIRMADA" if confirmada else "VERIFICANDO..."

            cv.polylines(frame, [box], True, color_bgr, 3)
            cv.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            cv.putText(
                frame,
                f"Piscina #{s_id} ({estado_str})",
                (cx - 60, cy - 15),
                cv.FONT_HERSHEY_SIMPLEX,
                0.6,
                color_bgr,
                2
            )

        cv.putText(
            frame,
            f"Piscinas detectadas: {len(detecciones)}",
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