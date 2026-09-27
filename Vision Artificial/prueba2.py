import cv2 as cv
import numpy as np
import requests
import time

IP_ESP32 = "192.168.8.125"
url = f"http://{IP_ESP32}:81/stream"

requests.get(f"http://{IP_ESP32}/control?var=led_intensity&val=255")
cap = cv.VideoCapture(url)

if not cap.isOpened():
    print("No se pudo conectar con la ESP32-CAM")
    exit()

print("Conectado a la ESP32-CAM")
print("Presiona ESC para salir")

azul_lower = np.array([106, 38,  40])
azul_upper = np.array([125, 176, 200])

amarillo_lower = np.array([10, 35, 134])
amarillo_upper = np.array([33, 108, 270])

rojo_lower_1 = np.array([167,   90,  55])
rojo_upper_1 = np.array([179,  194, 200])
                             #175   #163
naranja_lower = np.array([171, 96,  70])
naranja_upper = np.array([179, 255, 255])

black_lower = np.array([120, 50, 15])
black_upper = np.array([180, 130, 100])

AREA_MINIMA = 150           # descarta ruido muy pequeño (no depende de la distancia real)
CIRCULARIDAD_MINIMA = 0.77  # 1.0 = círculo perfecto; baja esto si pierdes pelotas lejanas
KERNEL = np.ones((5, 5), np.uint8)


TIEMPO_CONFIRMACION = 3.0
INTERVALO_ACTUALIZACION = 3.0
DISTANCIA_MAXIMA_SEGUIMIENTO = 80
TIEMPO_MAXIMO_PERDIDA = 1.0

seguimientos = {}
siguiente_id = 1
coordenadas_xy = {}


def actualizar_seguimientos(detecciones, ahora):
    """Asocia detecciones con pelotas existentes y confirma las persistentes."""
    global siguiente_id

    asociaciones = {}
    ids_disponibles = set(seguimientos)

    # Una detección solo puede asociarse a la pelota más cercana del mismo color.
    for indice, deteccion in enumerate(detecciones):
        x, y = deteccion["centro"]
        candidatos = []
        for seguimiento_id in ids_disponibles:
            seguimiento = seguimientos[seguimiento_id]
            if seguimiento["color"] != deteccion["color"]:
                continue
            distancia = np.hypot(
                x - seguimiento["centro"][0],
                y - seguimiento["centro"][1]
            )
            if distancia <= DISTANCIA_MAXIMA_SEGUIMIENTO:
                candidatos.append((distancia, seguimiento_id))

        if candidatos:
            _, seguimiento_id = min(candidatos)
            asociaciones[indice] = seguimiento_id
            ids_disponibles.remove(seguimiento_id)

    for indice, deteccion in enumerate(detecciones):
        seguimiento_id = asociaciones.get(indice)
        if seguimiento_id is None:
            seguimiento_id = siguiente_id
            siguiente_id += 1
            seguimientos[seguimiento_id] = {
                "color": deteccion["color"],
                "centro": deteccion["centro"],
                "inicio": ahora,
                "ultima_vez": ahora,
                "confirmada": False
            }
        else:
            seguimiento = seguimientos[seguimiento_id]
            seguimiento["centro"] = deteccion["centro"]
            seguimiento["ultima_vez"] = ahora

        seguimiento = seguimientos[seguimiento_id]
        if ahora - seguimiento["inicio"] >= TIEMPO_CONFIRMACION:
            seguimiento["confirmada"] = True
            coordenadas_xy[seguimiento_id] = {
                "color": seguimiento["color"],
                "xy": seguimiento["centro"],
                "actualizada": ahora
            }
        deteccion["seguimiento_id"] = seguimiento_id
        deteccion["confirmada"] = seguimiento["confirmada"]

    # Elimina pelotas que dejaron de verse para no reutilizar identificaciones antiguas.
    ids_expirados = [
        seguimiento_id
        for seguimiento_id, seguimiento in seguimientos.items()
        if ahora - seguimiento["ultima_vez"] > TIEMPO_MAXIMO_PERDIDA
    ]
    for seguimiento_id in ids_expirados:
        seguimientos.pop(seguimiento_id)
        coordenadas_xy.pop(seguimiento_id, None)

    return detecciones


def imprimir_coordenadas():
    """Imprime la última posición de cada pelota ya confirmada."""
    print(f"\nCoordenadas ({time.strftime('%H:%M:%S')}):")
    if not coordenadas_xy:
        print("  No hay pelotas confirmadas.")
        return

    for seguimiento_id in sorted(coordenadas_xy):
        pelota = coordenadas_xy[seguimiento_id]
        print(
            f"  Pelota {seguimiento_id} ({pelota['color']}): "
            f"x={pelota['xy'][0]}, y={pelota['xy'][1]}"
        )

def detectar_en_mascara(mascara, nombre_color):
    detecciones = []

    mascara = cv.morphologyEx(mascara, cv.MORPH_OPEN, KERNEL)
    mascara = cv.morphologyEx(mascara, cv.MORPH_CLOSE, KERNEL)

    contornos, _ = cv.findContours(
        mascara,
        cv.RETR_EXTERNAL,
        cv.CHAIN_APPROX_SIMPLE
    )

    for contorno in contornos:
        area = cv.contourArea(contorno)
        if area < AREA_MINIMA:
            continue

        perimetro = cv.arcLength(contorno, True)
        if perimetro == 0:
            continue

        circularidad = 4 * np.pi * area / (perimetro ** 2)
        if circularidad < CIRCULARIDAD_MINIMA:
            continue

        (x, y), radio = cv.minEnclosingCircle(contorno)

        detecciones.append({
            "color": nombre_color,
            "centro": (int(x), int(y)),
            "radio": int(radio),
            "area": area,
            "circularidad": circularidad
        })

    return detecciones

nombre_ventana = 'Deteccion de objetos'
cv.namedWindow(nombre_ventana, cv.WINDOW_NORMAL)
ultima_actualizacion = time.monotonic()

while cv.waitKey(1) != 27:

    has_frame, frame = cap.read()
    if not has_frame:
        print("No se pudo recibir imagen")
        break

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # Reduce un poco el ruido de la imagen antes de umbralizar
    hsv = cv.GaussianBlur(hsv, (5, 5), 0)

    # MÁSCARAS POR COLOR
    mascara_azul = cv.inRange(hsv, azul_lower, azul_upper)
    mascara_amarillo = cv.inRange(hsv, amarillo_lower, amarillo_upper)
    #mascara_rojo = cv.inRange(hsv, rojo_lower_1, rojo_upper_1)
    mascara_rojo = cv.bitwise_or(
        cv.inRange(hsv, rojo_lower_1, rojo_upper_1),
        cv.inRange(hsv, naranja_lower, naranja_upper))
    #mascara_naranja = cv.inRange(hsv, naranja_lower, naranja_upper)
    mascara_negra = cv.inRange(hsv, black_lower, black_upper)
    # Naranja cuenta como rojo
    #mascara_rojo = cv.bitwise_or(mascara_rojo, mascara_naranja)

    detecciones = []
    detecciones += detectar_en_mascara(mascara_azul, "Pelota azul")
    detecciones += detectar_en_mascara(mascara_amarillo, "Pelota amarilla")
    detecciones += detectar_en_mascara(mascara_rojo, "Pelota roja")
    detecciones += detectar_en_mascara(mascara_negra, "Pelota negra")

    ahora = time.monotonic()
    detecciones = actualizar_seguimientos(detecciones, ahora)
    if ahora - ultima_actualizacion >= INTERVALO_ACTUALIZACION:
        imprimir_coordenadas()
        ultima_actualizacion = ahora

    colores_dibujo = {
        "Pelota azul": (255, 0, 0),
        "Pelota amarilla": (0, 255, 255),
        "Pelota roja": (0, 0, 255),
        "Pelota negra": (0, 0, 0)
    }

    for d in detecciones:
        x, y = d["centro"]
        r = d["radio"]
        color_bgr = colores_dibujo[d["color"]]

        cv.circle(frame, (x, y), r, color_bgr, 3)
        cv.circle(frame, (x, y), 4, (255, 255, 255), -1)

        cv.putText(
            frame,
            f"{d['color']} #{d['seguimiento_id']}",
            (x - r, y - r - 10),
            cv.FONT_HERSHEY_SIMPLEX,
            0.6,
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

cap.release()
cv.destroyAllWindows()