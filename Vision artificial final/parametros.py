# ============================================================================
# ARCHIVO: parametros.py
# DESCRIPCIÓN: Lectura de parámetros de detección desde .env / variables de entorno
# ============================================================================
import os
import numpy as np

RUTA_ENV = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")

def _cargar_env(ruta):
    """Carga las líneas CLAVE=VALOR del archivo sin pisar variables de entorno ya definidas."""
    if not os.path.exists(ruta):
        return
    with open(ruta, encoding="utf-8") as archivo:
        for linea in archivo:
            linea = linea.strip()
            if not linea or linea.startswith("#") or "=" not in linea:
                continue
            clave, valor = linea.split("=", 1)
            os.environ.setdefault(clave.strip(), valor.strip())

def _leer(clave):
    valor = os.environ.get(clave)
    if valor is None:
        raise SystemExit(f"[PARAMETROS] Falta la variable '{clave}' en {RUTA_ENV}")
    return valor

def leer_int(clave):
    return int(_leer(clave))

def leer_float(clave):
    return float(_leer(clave))

def leer_hsv(clave):
    """Lee un rango HSV con formato 'H,S,V' y lo devuelve como np.array."""
    return np.array([int(v) for v in _leer(clave).split(",")])

_cargar_env(RUTA_ENV)
