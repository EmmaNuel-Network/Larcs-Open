# ============================================================================
# ARCHIVO: vision_general2.py
# DESCRIPCIÓN: Funciones de procesamiento de imágenes, filtrado y detección
# ============================================================================
import cv2 as cv
import numpy as np

KERNEL = np.ones((5, 5), np.uint8)

def detectar_circulos(mascara, nombre_tipo, area_min=150, area_max=10000, circularidad_min=0.75):
    """Detecta objetos esféricos (pelotas) asegurando alta circularidad."""
    detecciones = []
    mascara_limpia = cv.morphologyEx(mascara, cv.MORPH_OPEN, KERNEL)
    mascara_limpia = cv.morphologyEx(mascara_limpia, cv.MORPH_CLOSE, KERNEL)

    contornos, _ = cv.findContours(mascara_limpia, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    for contorno in contornos:
        area = cv.contourArea(contorno)
        if area < area_min or area > area_max:
            continue

        perimetro = cv.arcLength(contorno, True)
        if perimetro == 0:
            continue

        circularidad = 4 * np.pi * area / (perimetro ** 2)
        if circularidad >= circularidad_min:
            (x, y), radio = cv.minEnclosingCircle(contorno)
            detecciones.append({
                "tipo": nombre_tipo,
                "centro": (int(x), int(y)),
                "radio": int(radio),
                "area": area,
                "circularidad": circularidad
            })
    return detecciones

def detectar_follaje_arbol(mascara_verde, area_min_follaje=15000):
    """
    Detecta la masa verde continua del árbol.
    Diferencia el árbol completo de pelotas verdes aisladas.
    """
    mascara_limpia = cv.morphologyEx(mascara_verde, cv.MORPH_OPEN, KERNEL)
    mascara_limpia = cv.morphologyEx(mascara_limpia, cv.MORPH_CLOSE, KERNEL)

    contornos, _ = cv.findContours(mascara_limpia, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contornos:
        return None

    c_max = max(contornos, key=cv.contourArea)
    area = cv.contourArea(c_max)

    if area >= area_min_follaje:
        M = cv.moments(c_max)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return {
                "tipo": "Arbol Follaje",
                "centro": (cx, cy),
                "area": area,
                "contorno": c_max
            }
    return None

def detectar_rectangulos_orientados(mascara, nombre_tipo, area_min, aspect_ratio_range, solidez_min=0.65):
    """Detecta rectángulos con orientación libre (piscinas y contenedores)."""
    detecciones = []
    mascara_limpia = cv.morphologyEx(mascara, cv.MORPH_OPEN, KERNEL)
    mascara_limpia = cv.morphologyEx(mascara_limpia, cv.MORPH_CLOSE, KERNEL)

    contornos, _ = cv.findContours(mascara_limpia, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    aspect_min, aspect_max = aspect_ratio_range

    for contorno in contornos:
        area = cv.contourArea(contorno)
        if area < area_min:
            continue

        rect = cv.minAreaRect(contorno)
        (cx, cy), (w, h), angulo = rect
        if w == 0 or h == 0:
            continue

        largo = max(w, h)
        ancho = min(w, h)
        aspect_ratio = largo / ancho
        area_rect = w * h
        solidez = area / area_rect if area_rect > 0 else 0

        if aspect_min <= aspect_ratio <= aspect_max and solidez >= solidez_min:
            box = cv.boxPoints(rect)
            detecciones.append({
                "tipo": nombre_tipo,
                "centro": (int(cx), int(cy)),
                "angulo": angulo,
                "box": np.int32(box),
                "area": area
            })
    return detecciones