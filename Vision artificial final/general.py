# ============================================================================
# ARCHIVO: prueba_identificador_arbol.py (VERSIÓN OPTIMIZADA Y ANTI-REFLEJOS)
# DESCRIPCIÓN: Identificación rápida de pelotas cosechables (Maduras y
#              Sobremaduras) en Raspberry Pi 4.
#              - Ignora las pelotas verdes (ahorra CPU y evita penalización).
#              - Recorta (ROI) solo la zona del árbol para multiplicar los FPS.
#              - Normaliza la iluminación con CLAHE y corrige huecos por brillo
#                usando envolvente convexa (Convex Hull).
# ============================================================================
import cv2 as cv
import numpy as np
import time

from parametros import leer_float, leer_hsv, leer_int

# --- 1. CONFIGURACIÓN DE CÁMARA USB ---
ANCHO_FRAME = 640
ALTO_FRAME = 480
EXPOSICION = leer_int("EXPOSICION")
TEMPERATURA_BLANCOS = leer_int("TEMPERATURA_BLANCOS")

# --- 2. RANGOS HSV INICIALES (Desde .env, más tolerantes a la luz) ---
verde_arbol_lower = leer_hsv("VERDE_ARBOL_LOWER")
verde_arbol_upper = leer_hsv("VERDE_ARBOL_UPPER")

rojo_lower1 = leer_hsv("ROJO_LOWER1")
rojo_upper1 = leer_hsv("ROJO_UPPER1")
rojo_lower2 = leer_hsv("ROJO_LOWER2")
rojo_upper2 = leer_hsv("ROJO_UPPER2")

amarillo_lower = leer_hsv("AMARILLO_LOWER")
amarillo_upper = leer_hsv("AMARILLO_UPPER")

azul_lower = leer_hsv("AZUL_LOWER")
azul_upper = leer_hsv("AZUL_UPPER")

negro_lower = leer_hsv("NEGRO_LOWER")
negro_upper = leer_hsv("NEGRO_UPPER")

# Rango Naranja (Maduro)
naranja_lower = np.array([7, 85, 70])
naranja_upper = np.array([18, 255, 255])

# Relajar ligeramente S y V mínimos para que las sombras no borren las pelotas
for rng in (rojo_lower1, rojo_lower2, amarillo_lower, azul_lower):
    rng[1] = max(55, int(rng[1] * 0.75))  # Permite menor saturación si hay brillo
    rng[2] = max(45, int(rng[2] * 0.75))  # Permite menor brillo si hay sombra

ARBOL_AREA_MIN = leer_int("ARBOL_AREA_MIN")
PELOTA_AREA_MIN = max(250, int(leer_int("PELOTA_AREA_MIN") * 0.65))
PELOTA_AREA_MAX = leer_int("PELOTA_AREA_MAX")
# Con Convex Hull podemos exigir buena circularidad sin que el reflejo la rompa
CIRCULARIDAD_MIN = 0.70
RELLENO_MIN = 0.72

KERNEL_3 = cv.getStructuringElement(cv.MORPH_ELLIPSE, (3, 3))
KERNEL_5 = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))

# Ecualizador adaptativo de luz (CLAHE) en el canal V (Brillo)
CLAHE = cv.createCLAHE(clipLimit=2.0, tileGridSize=(6, 6))

COLORES_BGR = {
    "ROJO":     (0, 0, 255),
    "NARANJA":  (0, 140, 255),
    "AMARILLO": (0, 255, 255),
    "AZUL":     (255, 120, 0),
    "NEGRO":    (180, 180, 180),
}

def preprocesar_luz_hsv(frame_bgr):
    """
    Convierte a HSV y estabiliza el canal V (Brillo) con CLAHE para reducir
    el impacto de sombras y cambios de luz en el salón sin alterar el Tono (H).
    """
    blur = cv.GaussianBlur(frame_bgr, (5, 5), 0)
    hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)
    h, s, v = cv.split(hsv)
    v_eq = CLAHE.apply(v)
    return cv.merge((h, s, v_eq))

def detectar_roi_arbol(hsv_completo):
    """
    Localiza el árbol usando una imagen reducida a la mitad (320x240)
    para gastar 4 veces menos CPU en la Raspberry Pi.
    """
    hsv_small = cv.resize(hsv_completo, (320, 240), interpolation=cv.INTER_NEAREST)
    mascara_v = cv.inRange(hsv_small, verde_arbol_lower, verde_arbol_upper)
    mascara_v = cv.morphologyEx(mascara_v, cv.MORPH_OPEN, KERNEL_3)

    contornos, _ = cv.findContours(mascara_v, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contornos:
        return None

    c_max = max(contornos, key=cv.contourArea)
    if cv.contourArea(c_max) * 4 < ARBOL_AREA_MIN:
        return None

    x, y, w, h = cv.boundingRect(c_max)
    # Escalar de vuelta a 640x480 con un pequeño margen de 12 px
    x1 = max(0, x * 2 - 12)
    y1 = max(0, y * 2 - 12)
    x2 = min(ANCHO_FRAME, (x + w) * 2 + 12)
    y2 = min(ALTO_FRAME, (y + h) * 2 + 12)
    return (x1, y1, x2 - x1, y2 - y1)

def detectar_pelotas_robusto(mascara, nombre_color, categoria, offset_xy):
    """
    Usa Convex Hull (envolvente convexa) sobre cada contorno antes de medir
    circularidad y relleno. Así, si la pelota tiene un brillo blanco o una
    sombra que la parte o le hace una muesca, se reconstruye la esfera.
    """
    ox, oy = offset_xy
    # Cerrar primero los huecos internos causados por el reflejo de la luz
    limpia = cv.morphologyEx(mascara, cv.MORPH_CLOSE, KERNEL_5)
    limpia = cv.morphologyEx(limpia, cv.MORPH_OPEN, KERNEL_3)

    contornos, _ = cv.findContours(limpia, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    pelotas = []

    for c in contornos:
        # Envolvente convexa: rellena muescas causadas por reflejos de luz en el PLA
        hull = cv.convexHull(c)
        area = cv.contourArea(hull)
        if area < PELOTA_AREA_MIN or area > PELOTA_AREA_MAX:
            continue

        perimetro = cv.arcLength(hull, True)
        if perimetro == 0:
            continue

        circularidad = 4 * np.pi * area / (perimetro ** 2)
        if circularidad < CIRCULARIDAD_MIN:
            continue

        (x, y), radio = cv.minEnclosingCircle(hull)
        relleno = area / (np.pi * (radio ** 2)) if radio > 0 else 0
        if relleno < RELLENO_MIN:
            continue

        pelotas.append({
            "tipo": nombre_color,
            "categoria": categoria,
            "centro": (int(x) + ox, int(y) + oy),
            "radio": int(radio),
            "area": area
        })
    return pelotas

def filtrar_duplicados(detecciones, dist_min=20):
    ordenadas = sorted(detecciones, key=lambda d: d["area"], reverse=True)
    unicas = []
    for d in ordenadas:
        cx, cy = d["centro"]
        if not any(np.hypot(cx - u["centro"][0], cy - u["centro"][1]) < dist_min for u in unicas):
            unicas.append(d)
    return unicas

# Klik del mouse para inspeccionar y auto-adaptar rangos si falta una pelota
ultimo_clic_hsv = None
ultimo_clic_xy = None

def mouse_callback(event, x, y, flags, param):
    global ultimo_clic_xy
    if event == cv.EVENT_LBUTTONDOWN:
        ultimo_clic_xy = (x, y)

def main():
    global ultimo_clic_hsv, ultimo_clic_xy
    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        raise SystemExit("[ERROR] No se pudo abrir la cámara USB.")

    cap.set(cv.CAP_PROP_FRAME_WIDTH, ANCHO_FRAME)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, ALTO_FRAME)
    cap.set(cv.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.set(cv.CAP_PROP_EXPOSURE, EXPOSICION)
    cap.set(cv.CAP_PROP_AUTO_WB, 0)
    cap.set(cv.CAP_PROP_WB_TEMPERATURE, TEMPERATURA_BLANCOS)

    cv.namedWindow("Identificador Optimizado LARC 2025")
    cv.setMouseCallback("Identificador Optimizado LARC 2025", mouse_callback)

    print("=" * 70)
    print("[IDENTIFICADOR OPTIMIZADO ANTI-REFLEJOS - SIN PELOTAS VERDES]")
    print(" - Haz CLIC IZQUIERDO sobre una pelota que no se detecte para ver su HSV.")
    print(" - Presiona '1'=Rojo, '2'=Naranja, '3'=Amarillo, '4'=Azul, '5'=Negro")
    print("   después de hacer clic en una pelota para ampliar ese color al instante.")
    print(" - Presiona 'q' para salir.")
    print("=" * 70)

    prev_time = time.time()

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret or frame is None:
            time.sleep(0.005)
            continue

        # 1. Preprocesar imagen con estabilización de luz CLAHE
        hsv_full = preprocesar_luz_hsv(frame)

        # 2. Detectar ROI del árbol a baja resolución (muy rápido)
        roi_rect = detectar_roi_arbol(hsv_full)
        if roi_rect is not None:
            rx, ry, rw, rh = roi_rect
            hsv_roi = hsv_full[ry:ry+rh, rx:rx+rw]
            # Crear máscara de lo que ES follaje verde para excluirlo de las pelotas
            mascara_follaje = cv.inRange(hsv_roi, verde_arbol_lower, verde_arbol_upper)
            no_verde = cv.bitwise_not(mascara_follaje)
        else:
            rx, ry, rw, rh = 0, 0, ANCHO_FRAME, ALTO_FRAME
            hsv_roi = hsv_full
            no_verde = None

        # 3. Máscaras solo en el recorte (ROI) del árbol y excluyendo el verde
        m_rojo = cv.bitwise_or(
            cv.inRange(hsv_roi, rojo_lower1, rojo_upper1),
            cv.inRange(hsv_roi, rojo_lower2, rojo_upper2)
        )
        m_naranja  = cv.inRange(hsv_roi, naranja_lower, naranja_upper)
        m_amarillo = cv.inRange(hsv_roi, amarillo_lower, amarillo_upper)
        m_azul     = cv.inRange(hsv_roi, azul_lower, azul_upper)
        m_negro    = cv.inRange(hsv_roi, negro_lower, negro_upper)

        if no_verde is not None:
            m_negro = cv.bitwise_and(m_negro, no_verde)

        # 4. Detección rápida con corrección de reflejos (Convex Hull)
        candidatas = []
        for mascara_c, nombre_c, cat_c in [
            (m_rojo,     "ROJO",     "MADURO"),
            (m_naranja,  "NARANJA",  "MADURO"),
            (m_amarillo, "AMARILLO", "MADURO"),
            (m_azul,     "AZUL",     "SOBREMADURO"),
            (m_negro,    "NEGRO",    "SOBREMADURO"),
        ]:
            candidatas.extend(detectar_pelotas_robusto(mascara_c, nombre_c, cat_c, (rx, ry)))

        pelotas = filtrar_duplicados(candidatas, dist_min=20)

        # 5. Clasificar por fila (Alta, Media, Baja) y contar
        lim_1 = int(ry + rh * 0.37)
        lim_2 = int(ry + rh * 0.67)

        if roi_rect is not None:
            cv.rectangle(frame, (rx, ry), (rx + rw, ry + rh), (0, 200, 0), 2)
            cv.line(frame, (rx, lim_1), (rx + rw, lim_1), (0, 140, 0), 1)
            cv.line(frame, (rx, lim_2), (rx + rw, lim_2), (0, 140, 0), 1)

        conteo_filas = {
            "ALTA":  {"MADURO": 0, "SOBREMADURO": 0},
            "MEDIA": {"MADURO": 0, "SOBREMADURO": 0},
            "BAJA":  {"MADURO": 0, "SOBREMADURO": 0},
        }
        tot_maduros = 0
        tot_sobremaduros = 0

        for p in pelotas:
            cx, cy = p["centro"]
            r = p["radio"]
            col = p["tipo"]
            cat = p["categoria"]

            fila = "ALTA" if cy < lim_1 else ("MEDIA" if cy < lim_2 else "BAJA")
            conteo_filas[fila][cat] += 1

            if cat == "MADURO":
                tot_maduros += 1
            else:
                tot_sobremaduros += 1

            bgr = COLORES_BGR[col]
            cv.circle(frame, (cx, cy), r, bgr, 2)
            cv.circle(frame, (cx, cy), 3, (255, 255, 255), -1)
            cv.putText(frame, f"{col[:3]}-{fila[0]}", (cx - 22, cy - r - 4),
                       cv.FONT_HERSHEY_SIMPLEX, 0.42, bgr, 2)

        # 6. Lectura de píxel por clic para calibrar en 1 segundo bajo cualquier luz
        if ultimo_clic_xy is not None:
            px, py = ultimo_clic_xy
            # Promedio en una ventanita de 5x5 alrededor del clic para evitar ruido
            parche = hsv_full[max(0, py-2):min(ALTO_FRAME, py+3), max(0, px-2):min(ANCHO_FRAME, px+3)]
            ultimo_clic_hsv = np.median(parche.reshape(-1, 3), axis=0).astype(int)
            cv.circle(frame, (px, py), 5, (255, 0, 255), -1)
            h_c, s_c, v_c = ultimo_clic_hsv
            cv.putText(frame, f"Clic HSV: H={h_c} S={s_c} V={v_c} (Pulsa 1:Rojo 2:Nar 3:Ama 4:Azu 5:Neg)",
                       (10, ALTO_FRAME - 10), cv.FONT_HERSHEY_SIMPLEX, 0.42, (255, 100, 255), 1)

        # 7. HUD ligero con FPS y conteo de cosechables
        curr_time = time.time()
        fps = int(1.0 / max(0.001, curr_time - prev_time))
        prev_time = curr_time

        cv.rectangle(frame, (0, 0), (640, 75), (15, 15, 15), -1)
        cv.putText(frame, f"FPS: {fps} | COSECHABLES: {tot_maduros + tot_sobremaduros} (Maduros: {tot_maduros} | Sobremaduros: {tot_sobremaduros})",
                   (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1)
        cv.putText(frame, f"ALTA -> M:{conteo_filas['ALTA']['MADURO']} S:{conteo_filas['ALTA']['SOBREMADURO']} | MEDIA -> M:{conteo_filas['MEDIA']['MADURO']} S:{conteo_filas['MEDIA']['SOBREMADURO']} | BAJA -> M:{conteo_filas['BAJA']['MADURO']} S:{conteo_filas['BAJA']['SOBREMADURO']}",
                   (10, 45), cv.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1)
        cv.putText(frame, "Verdes ignoradas. Clic en pelota + tecla (1-5) para auto-ajustar a esta luz.",
                   (10, 66), cv.FONT_HERSHEY_SIMPLEX, 0.40, (180, 255, 180), 1)

        cv.imshow("Identificador Optimizado LARC 2025", frame)

        key = cv.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        # Auto-ajuste instantáneo con las teclas 1 a 5 tras hacer clic en una pelota
        elif ultimo_clic_hsv is not None and key in [ord('1'), ord('2'), ord('3'), ord('4'), ord('5')]:
            hc, sc, vc = ultimo_clic_hsv
            if key == ord('1'):   # Rojo
                if hc < 90:
                    rojo_lower1[:] = [0, max(40, sc - 35), max(40, vc - 45)]
                    rojo_upper1[:] = [max(rojo_upper1[0], hc + 4), 255, 255]
                else:
                    rojo_lower2[:] = [min(rojo_lower2[0], hc - 4), max(40, sc - 35), max(40, vc - 45)]
                    rojo_upper2[:] = [179, 255, 255]
                print(f"[AUTO-LUZ] Rojo actualizado -> L1:{rojo_lower1} L2:{rojo_lower2}")
            elif key == ord('2'): # Naranja
                naranja_lower[:] = [max(5, hc - 4), max(50, sc - 35), max(50, vc - 45)]
                naranja_upper[:] = [min(22, hc + 4), 255, 255]
                print(f"[AUTO-LUZ] Naranja actualizado -> {naranja_lower} a {naranja_upper}")
            elif key == ord('3'): # Amarillo
                amarillo_lower[:] = [max(14, hc - 5), max(50, sc - 35), max(50, vc - 45)]
                amarillo_upper[:] = [min(38, hc + 5), 255, 255]
                print(f"[AUTO-LUZ] Amarillo actualizado -> {amarillo_lower} a {amarillo_upper}")
            elif key == ord('4'): # Azul
                azul_lower[:] = [max(90, hc - 8), max(45, sc - 35), max(35, vc - 45)]
                azul_upper[:] = [min(138, hc + 8), 255, 255]
                print(f"[AUTO-LUZ] Azul actualizado -> {azul_lower} a {azul_upper}")
            elif key == ord('5'): # Negro (se ajusta el techo máximo de brillo V)
                negro_upper[:] = [180, 255, min(95, vc + 20)]
                print(f"[AUTO-LUZ] Negro actualizado -> Upper: {negro_upper}")

    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
