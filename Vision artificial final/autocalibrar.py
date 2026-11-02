# ============================================================================
# ARCHIVO: autocalibrar.py
# DESCRIPCIÓN: Auto calibración de los rangos HSV de las pelotas.
#              1. Indica abajo cuántas pelotas de cada color ve la cámara ahora.
#              2. El script prueba muchas combinaciones de saturación (S) y brillo (V)
#                 y se queda con la que cuenta exactamente esas pelotas.
#              3. Si lo confirmas, guarda los valores en .env (con copia en .env.bak).
#              El tono (H) no se modifica: depende del color de la pelota, no de la luz.
# ============================================================================
import shutil
import cv2 as cv

from vision_general2 import detectar_circulos
from parametros import RUTA_ENV, leer_float, leer_hsv, leer_int

# --- PELOTAS QUE LA CÁMARA ESTÁ VIENDO AHORA (0 = no calibrar ese color) ---
PELOTAS_AMARILLAS = 2
PELOTAS_ROJAS = 0
PELOTAS_NARANJAS = 0
PELOTAS_AZULES = 0
PELOTAS_NEGRAS = 0

# --- CONFIGURACIÓN DE LA BÚSQUEDA ---
FRAMES_A_CAPTURAR = 10  # El conteo debe ser correcto en todos (más = más fiable)
PASO = 10               # Salto entre valores de S y V probados (menor = más fino y más lento)

# --- CONFIGURACIÓN DE CÁMARA (la misma que usa el robot) ---
ANCHO_FRAME = 640
ALTO_FRAME = 480
EXPOSICION = leer_int("EXPOSICION")
TEMPERATURA_BLANCOS = leer_int("TEMPERATURA_BLANCOS")

# --- PARÁMETROS DE FORMA (los mismos que usa el robot) ---
PELOTA_AREA_MIN = leer_int("PELOTA_AREA_MIN")
PELOTA_AREA_MAX = leer_int("PELOTA_AREA_MAX")
PELOTA_CIRCULARIDAD_MIN = leer_float("PELOTA_CIRCULARIDAD_MIN")
PELOTA_RELLENO_MIN = leer_float("PELOTA_RELLENO_MIN")

# Por color: pelotas esperadas, rangos (lower, upper) en .env y qué extremo se ajusta.
# Los colores se aíslan subiendo S y V mínimos; el negro, bajando S y V máximos.
COLORES = {
    "AMARILLO": (PELOTAS_AMARILLAS, [("AMARILLO_LOWER", "AMARILLO_UPPER")], "MINIMOS"),
    "ROJO": (PELOTAS_ROJAS, [("ROJO_LOWER1", "ROJO_UPPER1"), ("ROJO_LOWER2", "ROJO_UPPER2")], "MINIMOS"),
    "AZUL": (PELOTAS_AZULES, [("AZUL_LOWER", "AZUL_UPPER")], "MINIMOS"),
    "NEGRO": (PELOTAS_NEGRAS, [("NEGRO_LOWER", "NEGRO_UPPER")], "MAXIMOS"),
}

def capturar_frames():
    """Captura varios frames con la configuración del robot y los devuelve en HSV."""
    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        raise SystemExit("[AUTOCALIBRAR] Error: no se pudo abrir la cámara.")
    cap.set(cv.CAP_PROP_FRAME_WIDTH, ANCHO_FRAME)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, ALTO_FRAME)
    cap.set(cv.CAP_PROP_AUTO_EXPOSURE, 1)  # 1 = manual en V4L2
    cap.set(cv.CAP_PROP_EXPOSURE, EXPOSICION)
    cap.set(cv.CAP_PROP_AUTO_WB, 0)
    cap.set(cv.CAP_PROP_WB_TEMPERATURE, TEMPERATURA_BLANCOS)

    for _ in range(15):  # Descartar los primeros frames mientras la cámara se estabiliza
        cap.read()

    frames_hsv = []
    for _ in range(FRAMES_A_CAPTURAR):
        ret, frame = cap.read()
        if ret and frame is not None:
            blur = cv.GaussianBlur(frame, (5, 5), 0)  # Mismo preprocesamiento que el robot
            frames_hsv.append(cv.cvtColor(blur, cv.COLOR_BGR2HSV))
    cap.release()

    if not frames_hsv:
        raise SystemExit("[AUTOCALIBRAR] Error: la cámara no entregó ningún frame.")
    return frames_hsv

def contar_pelotas(hsv, rangos):
    """Cuenta las pelotas que se detectan con una lista de rangos (lower, upper)."""
    mascara = cv.inRange(hsv, rangos[0][0], rangos[0][1])
    for lower, upper in rangos[1:]:
        mascara = cv.bitwise_or(mascara, cv.inRange(hsv, lower, upper))

    pelotas = detectar_circulos(
        mascara, "Pelota", area_min=PELOTA_AREA_MIN, area_max=PELOTA_AREA_MAX,
        circularidad_min=PELOTA_CIRCULARIDAD_MIN, relleno_min=PELOTA_RELLENO_MIN
    )
    return len(pelotas)

def ajustar_rangos(rangos, s, v, extremo):
    """Devuelve una copia de los rangos con S y V cambiados en el extremo indicado."""
    nuevos = []
    for lower, upper in rangos:
        lower, upper = lower.copy(), upper.copy()
        if extremo == "MINIMOS":
            lower[1], lower[2] = s, v
        else:
            upper[1], upper[2] = s, v
        nuevos.append((lower, upper))
    return nuevos

def elegir_mas_robusta(validas, fallidas):
    """
    De las combinaciones (S, V) que cuentan bien, elige la más alejada de las que fallan.
    Así un pequeño cambio de luz no la saca de la zona que funciona.
    Salirse de 0-255 también cuenta como fallo, para no elegir extremos como S=0 o V=0
    (aceptarían también blancos, grises y negros que no estaban en la escena).
    """
    def distancia_al_fallo_mas_cercano(combinacion):
        s, v = combinacion
        distancias = [(s - fs) ** 2 + (v - fv) ** 2 for fs, fv in fallidas]
        distancias += [(s + PASO) ** 2, (v + PASO) ** 2, (255 + PASO - s) ** 2, (255 + PASO - v) ** 2]
        return min(distancias)

    return max(validas, key=distancia_al_fallo_mas_cercano)

def calibrar_color(frames_hsv, esperadas, rangos, extremo):
    """Prueba todas las combinaciones de S y V. Devuelve los rangos más robustos o None."""
    validas, fallidas = [], []
    for s in range(0, 256, PASO):
        for v in range(0, 256, PASO):
            candidatos = ajustar_rangos(rangos, s, v, extremo)
            if all(contar_pelotas(hsv, candidatos) == esperadas for hsv in frames_hsv):
                validas.append((s, v))
            else:
                fallidas.append((s, v))

    if not validas:
        return None
    s, v = elegir_mas_robusta(validas, fallidas)
    return ajustar_rangos(rangos, s, v, extremo)

def a_texto(hsv):
    """Convierte un rango HSV al formato de .env: 'H,S,V'."""
    return ",".join(str(int(x)) for x in hsv)

def guardar_en_env(nuevos_valores):
    """Reescribe en .env solo las claves indicadas, conservando comentarios y orden."""
    with open(RUTA_ENV, encoding="utf-8") as archivo:
        lineas = archivo.readlines()

    with open(RUTA_ENV, "w", encoding="utf-8") as archivo:
        for linea in lineas:
            clave = linea.split("=", 1)[0].strip()
            if clave in nuevos_valores and not linea.lstrip().startswith("#"):
                linea = f"{clave}={nuevos_valores[clave]}\n"
            archivo.write(linea)

if __name__ == "__main__":
    print("=" * 65)
    print("[AUTO CALIBRACIÓN HSV DE PELOTAS]")
    print(" Deja quietas delante de la cámara las pelotas indicadas en el script.")
    print("=" * 65)

    frames_hsv = capturar_frames()
    print(f"[AUTOCALIBRAR] {len(frames_hsv)} frames capturados.")

    nuevos_valores = {}
    for nombre, (esperadas, claves, extremo) in COLORES.items():
        if esperadas == 0:
            continue
        print(f"\n[AUTOCALIBRAR] Calibrando {nombre} ({esperadas} pelotas)...")

        rangos = [(leer_hsv(k_lower), leer_hsv(k_upper)) for k_lower, k_upper in claves]
        resultado = calibrar_color(frames_hsv, esperadas, rangos, extremo)

        if resultado is None:
            print(f"  No hay ningún valor que cuente {esperadas} pelotas en todos los frames.")
            print("  Revisa el número de pelotas, la luz o el tono (H) con calibrar_vision.py")
            continue

        for (k_lower, k_upper), (lower, upper) in zip(claves, resultado):
            nuevos_valores[k_lower] = a_texto(lower)
            nuevos_valores[k_upper] = a_texto(upper)
            print(f"  {k_lower}: {a_texto(leer_hsv(k_lower))} -> {a_texto(lower)}")
            print(f"  {k_upper}: {a_texto(leer_hsv(k_upper))} -> {a_texto(upper)}")

    if not nuevos_valores:
        raise SystemExit("\n[AUTOCALIBRAR] No hay valores nuevos que guardar.")

    if input("\n¿Guardar estos valores en .env? (s/n): ").strip().lower() == "s":
        shutil.copy(RUTA_ENV, RUTA_ENV + ".bak")
        guardar_en_env(nuevos_valores)
        print("[AUTOCALIBRAR] Guardado. Copia de los valores anteriores en .env.bak")
        print("[AUTOCALIBRAR] Verifica el resultado con todo.py")
    else:
        print("[AUTOCALIBRAR] No se guardó nada.")
