# ============================================================================
# ARCHIVO: calibrar_vision.py
# DESCRIPCIÓN: Calibrador HSV interactivo en vivo para la Cámara USB HD 1080P
# ============================================================================
import cv2 as cv
import numpy as np

def nada(x):
    pass

cv.namedWindow("Calibracion HSV", cv.WINDOW_NORMAL)
cv.resizeWindow("Calibracion HSV", 500, 350)

cv.createTrackbar("H Min", "Calibracion HSV", 0, 179, nada)
cv.createTrackbar("H Max", "Calibracion HSV", 179, 179, nada)
cv.createTrackbar("S Min", "Calibracion HSV", 0, 255, nada)
cv.createTrackbar("S Max", "Calibracion HSV", 255, 255, nada)
cv.createTrackbar("V Min", "Calibracion HSV", 0, 255, nada)
cv.createTrackbar("V Max", "Calibracion HSV", 255, 255, nada)

cv.createTrackbar("Kernel Morf", "Calibracion HSV", 5, 15, nada)
cv.createTrackbar("Area Minima", "Calibracion HSV", 150, 5000, nada)

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

print("=" * 60)
print("[CALIBRADOR HSV INTERACTIVO]")
print(" Configura las barras para aislar el color deseado.")
print(" Presiona 's' para imprimir los rangos en consola.")
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
    if k_size < 1:
        k_size = 1

    blur = cv.GaussianBlur(frame, (5, 5), 0)
    hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mascara_cruda = cv.inRange(hsv, lower, upper)

    kernel = np.ones((k_size, k_size), np.uint8)
    mascara_limpia = cv.morphologyEx(mascara_cruda, cv.MORPH_OPEN, kernel)
    mascara_limpia = cv.morphologyEx(mascara_limpia, cv.MORPH_CLOSE, kernel)

    contornos, _ = cv.findContours(mascara_limpia, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    frame_dibujado = frame.copy()

    for contorno in contornos:
        area = cv.contourArea(contorno)
        if area >= area_min:
            (x, y), radio = cv.minEnclosingCircle(contorno)
            cv.circle(frame_dibujado, (int(x), int(y)), int(radio), (0, 255, 0), 2)

    mascara_3ch = cv.cvtColor(mascara_limpia, cv.COLOR_GRAY2BGR)
    vista_combinada = np.hstack((frame_dibujado, mascara_3ch))

    cv.imshow("Camara USB | Mascara Filtrada", vista_combinada)

    key = cv.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):
        print("\n--- VALORES RANGO HSV CALIBRADOS ---")
        print(f"lower = np.array([{h_min}, {s_min}, {v_min}])")
        print(f"upper = np.array([{h_max}, {s_max}, {v_max}])\n")

cap.release()
cv.destroyAllWindows()