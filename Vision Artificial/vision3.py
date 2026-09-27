import cv2
import requests

IP_ESP32 = "192.168.8.125"
url = f"http://{IP_ESP32}:81/stream"
requests.get(f"http://{IP_ESP32}/control?var=led_intensity&val=255")
cap = cv2.VideoCapture(url)

if not cap.isOpened():
    print("No se pudo conectar con la ESP32-CAM")
    exit()

print("Conectado a la ESP32-CAM")
print("Presiona ESC para salir")

while True:

    ret, frame = cap.read()
    if not ret:
        print("No se pudo recibir el frame")
        break

    cv2.imshow("ESP32-CAM", frame)
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()