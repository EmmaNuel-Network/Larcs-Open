import cv2 as cv
import numpy as np
import requests
import time
import threading

KERNEL = np.ones((5, 5), np.uint8)


class ESP32CamStream:
    """Maneja el flujo de video en segundo plano con reconexión automática."""
    def __init__(self, url, ip_esp32):
        self.url = url
        self.ip_esp32 = ip_esp32
        self.cap = None
        self.frame = None
        self.ret = False
        self.running = False
        self.lock = threading.Lock()
        self.thread = None
        self.ultima_lectura = time.monotonic()

    def _encender_led(self):
        try:
            requests.get(f"http://{self.ip_esp32}/control?var=led_intensity&val=255", timeout=2)
        except Exception:
            pass

    def conectar(self):
        self._encender_led()
        if self.cap is not None:
            self.cap.release()
        self.cap = cv.VideoCapture(self.url)
        return self.cap.isOpened()

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._actualizar_frame, daemon=True)
        self.thread.start()

    def _actualizar_frame(self):
        while self.running:
            if self.cap is not None and self.cap.isOpened():
                has_frame, frame = self.cap.read()
                if has_frame:
                    with self.lock:
                        self.frame = frame
                        self.ret = True
                        self.ultima_lectura = time.monotonic()
                else:
                    with self.lock:
                        self.ret = False
            else:
                with self.lock:
                    self.ret = False

            if time.monotonic() - self.ultima_lectura > 2.0:
                self._reconectar()

            time.sleep(0.005)

    def _reconectar(self):
        with self.lock:
            self.ret = False
        if self.cap is not None:
            self.cap.release()

        while self.running:
            if self.conectar():
                has_frame, frame = self.cap.read()
                if has_frame:
                    with self.lock:
                        self.frame = frame
                        self.ret = True
                        self.ultima_lectura = time.monotonic()
                    break
            time.sleep(2.0)

    def read(self):
        with self.lock:
            if self.ret and self.frame is not None:
                return True, self.frame.copy()
            return False, None

    def stop(self):
        self.running = False
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.cap is not None:
            self.cap.release()


class TrackerObjetos:
    """Clase para seguimiento temporal y confirmación de objetos detectados."""
    def __init__(self, tiempo_confirmacion=3.0, tiempo_maximo_perdida=1.0, dist_maxima=120):
        self.tiempo_confirmacion = tiempo_confirmacion
        self.tiempo_max_perdida = tiempo_maximo_perdida
        self.dist_maxima = dist_maxima
        self.seguimientos = {}
        self.coordenadas_xy = {}
        self.siguiente_id = 1

    def actualizar(self, detecciones, ahora):
        asociaciones = {}
        ids_disponibles = set(self.seguimientos)

        for indice, deteccion in enumerate(detecciones):
            x, y = deteccion["centro"]
            candidatos = []
            for s_id in ids_disponibles:
                seguimiento = self.seguimientos[s_id]
                # Asociar solo si es del mismo color/tipo
                if seguimiento["tipo"] != deteccion["tipo"]:
                    continue

                distancia = np.hypot(x - seguimiento["centro"][0], y - seguimiento["centro"][1])
                if distancia <= self.dist_maxima:
                    candidatos.append((distancia, s_id))

            if candidatos:
                _, s_id = min(candidatos)
                asociaciones[indice] = s_id
                ids_disponibles.remove(s_id)

        for indice, deteccion in enumerate(detecciones):
            s_id = asociaciones.get(indice)
            if s_id is None:
                s_id = self.siguiente_id
                self.siguiente_id += 1
                self.seguimientos[s_id] = {
                    "tipo": deteccion["tipo"],
                    "centro": deteccion["centro"],
                    "inicio": ahora,
                    "ultima_vez": ahora,
                    "confirmada": False
                }
            else:
                seguimiento = self.seguimientos[s_id]
                seguimiento["centro"] = deteccion["centro"]
                seguimiento["ultima_vez"] = ahora

            seguimiento = self.seguimientos[s_id]
            if ahora - seguimiento["inicio"] >= self.tiempo_confirmacion:
                seguimiento["confirmada"] = True
                self.coordenadas_xy[s_id] = {
                    "tipo": seguimiento["tipo"],
                    "xy": seguimiento["centro"],
                    "angulo": deteccion["angulo"],
                    "dimensiones_px": deteccion["dimensiones_px"],
                    "actualizada": ahora
                }
            deteccion["seguimiento_id"] = s_id
            deteccion["confirmada"] = seguimiento["confirmada"]

        ids_expirados = [
            s_id for s_id, s in self.seguimientos.items()
            if ahora - s["ultima_vez"] > self.tiempo_max_perdida
        ]
        for s_id in ids_expirados:
            self.seguimientos.pop(s_id)
            self.coordenadas_xy.pop(s_id, None)

        return detecciones

    def imprimir_confirmados(self):
        print(f"\n--- Objetos Confirmados ({time.strftime('%H:%M:%S')}) ---")
        if not self.coordenadas_xy:
            print("  No hay objetos confirmados actualmente.")
            return

        for s_id, data in sorted(self.coordenadas_xy.items()):
            x, y = data["xy"]
            ang = data["angulo"]
            w_px, h_px = data["dimensiones_px"]
            print(
                f"  [{data['tipo']}] #{s_id}: Centro (X:{x}, Y:{y}) | "
                f"Ángulo: {ang:.1f}° | Dim px: ({w_px}x{h_px})"
            )


def detectar_rectangulos_orientados(mascara, nombre_tipo, area_min, aspect_ratio_range, solidez_min):
    """Detecta rectángulos con cualquier ángulo de rotación usando minAreaRect."""
    detecciones = []

    mascara = cv.morphologyEx(mascara, cv.MORPH_OPEN, KERNEL)
    mascara = cv.morphologyEx(mascara, cv.MORPH_CLOSE, KERNEL)

    contornos, _ = cv.findContours(mascara, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
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
            box = np.int32(box)

            detecciones.append({
                "tipo": nombre_tipo,
                "centro": (int(cx), int(cy)),
                "angulo": angulo,
                "dimensiones_px": (int(largo), int(ancho)),
                "box": box,
                "area": area,
                "aspect_ratio": aspect_ratio
            })

    return detecciones

def detectar_circulos(mascara, nombre_tipo, area_min, circularidad_min):
    """Detecta círculos (pelotas) basándose en área y circularidad."""
    detecciones = []

    mascara = cv.morphologyEx(mascara, cv.MORPH_OPEN, KERNEL)
    mascara = cv.morphologyEx(mascara, cv.MORPH_CLOSE, KERNEL)

    contornos, _ = cv.findContours(mascara, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    for contorno in contornos:
        area = cv.contourArea(contorno)
        if area < area_min:
            continue

        perimetro = cv.arcLength(contorno, True)
        if perimetro == 0:
            continue

        circularidad = 4 * np.pi * area / (perimetro ** 2)
        if circularidad < circularidad_min:
            continue

        (x, y), radio = cv.minEnclosingCircle(contorno)

        detecciones.append({
            "tipo": nombre_tipo,
            "centro": (int(x), int(y)),
            "radio": int(radio),
            "area": area,
            "circularidad": circularidad,
            "angulo": 0.0,
            "dimensiones_px": (int(radio * 2), int(radio * 2))
        })

    return detecciones