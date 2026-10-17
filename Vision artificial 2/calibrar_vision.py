import cv2 as cv
import numpy as np

def nada(x):
    pass

# --- CREACIÓN DE VENTANA Y BARRAS DESLIZANTES ---
cv.namedWindow("Calibracion HSV", cv.WINDOW_NORMAL)
cv.resizeWindow("Calibracion HSV", 500, 350)

# Rangos HSV (Hue: 0-179, Saturation: 0-255, Value: 0-255)
cv.createTrackbar("H Min", "Calibracion HSV", 0, 179, nada)
cv.createTrackbar("H Max", "Calibracion HSV", 179, 179, nada)
cv.createTrackbar("S Min", "Calibracion HSV", 0, 255, nada)
cv.createTrackbar("S Max", "Calibracion HSV", 255, 255, nada)
cv.createTrackbar("V Min", "Calibracion HSV", 0, 255, nada)
cv.createTrackbar("V Max", "Calibracion HSV", 255, 255, nada)

# Filtros de Limpieza
cv.createTrackbar("Kernel Morf", "Calibracion HSV", 5, 15, nada)
cv.createTrackbar("Area Minima", "Calibracion HSV", 150, 5000, nada)

# --- INICIALIZACIÓN DE CÁMARA USB ---
cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

print("=" * 60)
print("[CALIBRADOR HSV INTERACTIVO]")
print(" Utiliza las barras deslizantes para aislar cada color.")
print(" Presiona 's' para imprimir los valores calibrados en consola.")
print(" Presiona 'q' para salir.")
print("=" * 60)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Error: No se puede obtener imagen de la camara /dev/video0")
        break

    # 1. Lectura de las barras deslizantes
    h_min = cv.getTrackbarPos("H Min", "Calibracion HSV")
    h_max = cv.getTrackbarPos("H Max", "Calibracion HSV")
    s_min = cv.getTrackbarPos("S Min", "Calibracion HSV")
    s_max = cv.getTrackbarPos("S Max", "Calibracion HSV")
    v_min = cv.getTrackbarPos("V Min", "Calibracion HSV")
    v_max = cv.getTrackbarPos("V Max", "Calibracion HSV")
    
    k_size = cv.getTrackbarPos("Kernel Morf", "Calibracion HSV")
    area_min = cv.getTrackbarPos("Area Minima", "Calibracion HSV")

    # El kernel morfológico debe ser impar y mínimo 1
    if k_size % 2 == 0:
        k_size += 1
    if k_size < 1:
        k_size = 1

    # 2. Preprocesamiento e imagen HSV
    blur = cv.GaussianBlur(frame, (5, 5), 0)
    hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

    # 3. Mascara de Color
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mascara_cruda = cv.inRange(hsv, lower, upper)

    # 4. Operaciones Morfológicas (Limpieza de Ruido)
    kernel = np.ones((k_size, k_size), np.uint8)
    # Apertura (MORPH_OPEN): elimina pequeños puntos de ruido aislado
    mascara_limpia = cv.morphologyEx(mascara_cruda, cv.MORPH_OPEN, kernel)
    # Cierre (MORPH_CLOSE): rellena pequeños huecos internos del objeto
    mascara_limpia = cv.morphologyEx(mascara_limpia, cv.MORPH_CLOSE, kernel)

    # 5. Detección de Objetos e Identificación de Área
    contornos, _ = cv.findContours(mascara_limpia, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    frame_dibujado = frame.copy()

    for contorno in contornos:
        area = cv.contourArea(contorno)
        if area >= area_min:
            (x, y), radio = cv.minEnclosingCircle(contorno)
            cv.circle(frame_dibujado, (int(x), int(y)), int(radio), (0, 255, 0), 2)
            cv.circle(frame_dibujado, (int(x), int(y)), 3, (0, 0, 255), -1)
            cv.putText(frame_dibujado, f"Area: {int(area)}", (int(x) - 30, int(y) - 10),
                       cv.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

    # Convertir máscara a 3 canales para mostrarla junto a la imagen original
    mascara_3ch = cv.cvtColor(mascara_limpia, cv.COLOR_GRAY2BGR)
    vista_combinada = np.hstack((frame_dibujado, mascara_3ch))

    cv.imshow("Camara USB (Izquierda) | Mascara Filtrada (Derecha)", vista_combinada)

    key = cv.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):
        print("\n--- VALORES RANGO HSV DETECTADOS ---")
        print(f"lower = np.array([{h_min}, {s_min}, {v_min}])")
        print(f"upper = np.array([{h_max}, {s_max}, {v_max}])")
        print(f"Kernel = {k_size}, Area Minima = {area_min}\n")

cap.release()
cv.destroyAllWindows()