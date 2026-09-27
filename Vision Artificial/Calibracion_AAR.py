import cv2 as cv
import numpy as np

# CONFIGURACIÓN: pon aquí las rutas de tus 3 fotos

rutas_imagenes = {
    "azul": "Visión Artificial\Pruebas\arbol4.png",
    "rojo-naranja": "Visión Artificial\Pruebas\arbol4.png",
    "amarillo": "Visión Artificial\Pruebas\arbol4.png",
    "negra": "Visión Artificial\Pruebas\negra3.png"
}

# ============================================================
# FUNCIÓN: seleccionar ROI manualmente y calcular rango HSV
# ============================================================

def calibrar_desde_imagen(nombre_color, ruta):

    frame = cv.imread(ruta)

    if frame is None:
        print(f"No se pudo leer la imagen: {ruta}")
        return None

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    print(f"\n=== Calibrando color: {nombre_color} ({ruta}) ===")
    print("Dibuja un rectángulo SOLO sobre la pelota (evita el fondo)")
    print("Presiona ENTER o ESPACIO para confirmar, o 'c' para cancelar")

    roi = cv.selectROI(f"Selecciona pelota {nombre_color}", frame, showCrosshair=True)
    cv.destroyWindow(f"Selecciona pelota {nombre_color}")

    x, y, w, h = roi

    if w == 0 or h == 0:
        print(f"No se seleccionó ninguna región para {nombre_color}")
        return None

    region_hsv = hsv[y:y + h, x:x + w].reshape(-1, 3)

    H = region_hsv[:, 0]
    S = region_hsv[:, 1]
    V = region_hsv[:, 2]

    H_promedio, S_promedio, V_promedio = H.mean(), S.mean(), V.mean()
    H_min, H_max = int(H.min()), int(H.max())
    S_min, S_max = int(S.min()), int(S.max())
    V_min, V_max = int(V.min()), int(V.max())

    print(f"H -> promedio: {H_promedio:.1f}  rango: {H_min}-{H_max}")
    print(f"S -> promedio: {S_promedio:.1f}  rango: {S_min}-{S_max}")
    print(f"V -> promedio: {V_promedio:.1f}  rango: {V_min}-{V_max}")

    lower = np.array([H_min, S_min, V_min])
    upper = np.array([H_max, S_max, V_max])

    # Validación visual: máscara resultante con ese rango sobre TODA la imagen
    mascara = cv.inRange(hsv, lower, upper)

    cv.imshow(f"{nombre_color} - original", frame)
    cv.imshow(f"{nombre_color} - mascara resultante", mascara)
    print("Revisa la máscara. Presiona cualquier tecla para continuar con el siguiente color...")
    cv.waitKey(0)
    cv.destroyAllWindows()

    return lower, upper


# ============================================================
# EJECUTAR PARA CADA FOTO
# ============================================================

resultados = {}

for nombre_color, ruta in rutas_imagenes.items():
    resultado = calibrar_desde_imagen(nombre_color, ruta)
    if resultado is not None:
        resultados[nombre_color] = resultado


# ============================================================
# RESUMEN FINAL (listo para copiar a tu script de detección)
# ============================================================

print("\n\n================ RESUMEN DE CALIBRACIÓN ================")
for nombre_color, (lower, upper) in resultados.items():
    print(f"{nombre_color}:")
    print(f"    lower = np.array({lower.tolist()})")
    print(f"    upper = np.array({upper.tolist()})")