# ============================================================================
# ARCHIVO: calibrar_vision.py
# DESCRIPCIÓN: Calibrador HSV optimizado a 320x240 para VNC fluido.
#              Filtros morfológicos y de preprocesamiento sincronizados con vision_general2.py.
# ============================================================================
import cv2 as cv
import numpy as np

from parametros import leer_int

def nada(x):
    pass

cv.namedWindow("Calibracion HSV", cv.WINDOW_NORMAL)
cv.resizeWindow("Calibracion HSV", 420, 360)

cv.createTrackbar("H Min", "Calibracion HSV", 0, 179, nada)
cv.createTrackbar("H Max", "Calibracion HSV", 179, 179, nada)
cv.createTrackbar("S Min", "Calibracion HSV", 0, 255, nada)
cv.createTrackbar("S Max", "Calibracion HSV", 255, 255, nada)
cv.createTrackbar("V Min", "Calibracion HSV", 0, 255, nada)
cv.createTrackbar("V Max", "Calibracion HSV", 255, 255, nada)

cv.createTrackbar("Kernel Morf", "Calibracion HSV", 3, 9, nada)
cv.createTrackbar("Area Minima", "Calibracion HSV", 40, 2500, nada)  # Área calibrada para 320x240

# Exposición y balance de blancos fijos (definidos en .env)
EXPOSICION = leer_int("EXPOSICION")
TEMPERATURA_BLANCOS = leer_int("TEMPERATURA_BLANCOS")

cap = cv.VideoCapture(0)
if not cap.isOpened():
    raise SystemExit("[CALIBRADOR] Error: no se pudo abrir la cámara.")

# Resolución ligera para calibración fluida vía TightVNC sobre SSH
cap.set(cv.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv.CAP_PROP_AUTO_EXPOSURE, 1)  # 1 = manual en V4L2
cap.set(cv.CAP_PROP_EXPOSURE, EXPOSICION)
cap.set(cv.CAP_PROP_AUTO_WB, 0)
cap.set(cv.CAP_PROP_WB_TEMPERATURE, TEMPERATURA_BLANCOS)

print("=" * 60)
print("[CALIBRADOR HSV OPTIMIZADO - 320x240 (VNC)]")
print(" Filtros morfológicos: OPEN + CLOSE sincronizados con vision_general2.py")
print(" Verde: Cumple circularidad de esfera (>= 0.75)")
print(" Naranja: Contorno detectado pero no pasa filtro esférico")
print(" Presiona 's' para imprimir los rangos calibrados en consola.")
print(" Presiona 'q' para salir.")
print("=" * 60)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    h_min = cv.getTrackbarPos("H Min", "Calibracion HSV")
    h_max = cv.getTrackbarPos("H Max", "Calibracion HSV")
    s_min = cv.getTrackbarPos("S Min", "Calibracion HSV")
    s_max = cv.getTrackbarPos("S Max", "Calibracion HSV")
    v_min = cv.getTrackbarPos("V Min", "Calibracion HSV")
    v_max = cv.getTrackbarPos("V Max", "Calibracion HSV")
    k_size = cv.getTrackbarPos("Kernel Morf", "Calibracion HSV")
    area_min = cv.getTrackbarPos("Area Minima", "Calibracion HSV")

    if k_size % 2 == 0:
        k_size += 1
    k_size = max(1, k_size)

    # Preprocesamiento idéntico al código de producción (3x3 en 320x240 equivale a 5x5 en 640x480)
    blur = cv.GaussianBlur(frame, (3, 3), 0)
    hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mascara_cruda = cv.inRange(hsv, lower, upper)

    # Filtros morfológicos: OPEN + CLOSE idéntico a vision_general2.py
    kernel = np.ones((k_size, k_size), np.uint8)
    mascara_limpia = cv.morphologyEx(mascara_cruda, cv.MORPH_OPEN, kernel)
    mascara_limpia = cv.morphologyEx(mascara_limpia, cv.MORPH_CLOSE, kernel)

    contornos, _ = cv.findContours(mascara_limpia, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    frame_dibujado = frame.copy()

    for contorno in contornos:
        area = cv.contourArea(contorno)
        if area >= area_min:
            perimetro = cv.arcLength(contorno, True)
            circularidad = (4 * np.pi * area / (perimetro ** 2)) if perimetro > 0 else 0
            (x, y), radio = cv.minEnclosingCircle(contorno)
            relleno = area / (np.pi * radio ** 2) if radio > 0 else 0

            # Indicador visual de validez como esfera (reglas de vision_general2.py)
            es_esfera = (circularidad >= 0.75 and relleno >= 0.8)
            color_borde = (0, 255, 0) if es_esfera else (0, 165, 255)

            cv.circle(frame_dibujado, (int(x), int(y)), int(radio), color_borde, 2)
            cv.putText(frame_dibujado, f"A:{int(area)} C:{circularidad:.2f}",
                       (int(x) - 35, int(y) - int(radio) - 5),
                       cv.FONT_HERSHEY_SIMPLEX, 0.4, color_borde, 1)

    cv.imshow("Mascara HSV (Filtrada)", mascara_limpia)
    cv.imshow("Deteccion en Vivo", frame_dibujado)

    key = cv.waitKey(30) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):
        print("\n--- VALORES RANGO HSV CALIBRADOS (copiar en .env cambiando COLOR) ---")
        print(f"COLOR_LOWER={h_min},{s_min},{v_min}")
        print(f"COLOR_UPPER={h_max},{s_max},{v_max}")
        print(f"Area Minima equivalente en 640x480 (x4) = {area_min * 4}\n")

cap.release()
cv.destroyAllWindows()
