# ============================================================================
# ARCHIVO: calibrar_vision.py
# DESCRIPCIÓN: Calibrador HSV optimizado a 320x240 para VNC fluido
# ============================================================================
import cv2 as cv
import numpy as np

def nada(x):
    pass

cv.namedWindow("Calibracion HSV", cv.WINDOW_NORMAL)
cv.resizeWindow("Calibracion HSV", 400, 320)

cv.createTrackbar("H Min", "Calibracion HSV", 0, 179, nada)
cv.createTrackbar("H Max", "Calibracion HSV", 179, 179, nada)
cv.createTrackbar("S Min", "Calibracion HSV", 0, 255, nada)
cv.createTrackbar("S Max", "Calibracion HSV", 255, 255, nada)
cv.createTrackbar("V Min", "Calibracion HSV", 0, 255, nada)
cv.createTrackbar("V Max", "Calibracion HSV", 255, 255, nada)

cv.createTrackbar("Kernel Morf", "Calibracion HSV", 3, 9, nada)
cv.createTrackbar("Area Minima", "Calibracion HSV", 40, 1250, nada) # Área calibrada para 320x240

cap = cv.VideoCapture(0)
# Resolución ligera para calibración fluida vía VNC
cap.set(cv.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

print("=" * 60)
print("[CALIBRADOR HSV OPTIMIZADO - 320x240]")
print(" Ajusta los valores para aislar el objetivo.")
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

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mascara_cruda = cv.inRange(hsv, lower, upper)

    kernel = np.ones((k_size, k_size), np.uint8)
    mascara_limpia = cv.morphologyEx(mascara_cruda, cv.MORPH_OPEN, kernel)

    contornos, _ = cv.findContours(mascara_limpia, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    frame_dibujado = frame.copy()

    for contorno in contornos:
        if cv.contourArea(contorno) >= area_min:
            (x, y), radio = cv.minEnclosingCircle(contorno)
            cv.circle(frame_dibujado, (int(x), int(y)), int(radio), (0, 255, 0), 2)

    cv.imshow("Mascara HSV (Filtrada)", mascara_limpia)
    cv.imshow("Deteccion en Vivo", frame_dibujado)

    key = cv.waitKey(30) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):
        print("\n--- VALORES RANGO HSV CALIBRADOS ---")
        print(f"lower = np.array([{h_min}, {s_min}, {v_min}])")
        print(f"upper = np.array([{h_max}, {s_max}, {v_max}])")
        print(f"Area Minima en 640x480 (x4) = {area_min * 4}\n")

cap.release()
cv.destroyAllWindows()