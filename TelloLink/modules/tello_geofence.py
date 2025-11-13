from __future__ import annotations
import math
import threading
import time
from typing import List, Tuple, Optional, Dict, Any

# Configuración general ---
_DEFAULT_MAX_X_CM = 150.0
_DEFAULT_MAX_Y_CM = 150.0
_DEFAULT_MAX_Z_CM = 120.0
_DEFAULT_POLL_S = 0.10
_HARD_LAND_DELAY = 0.2
_MODE_SOFT_ABORT = "soft"
_MODE_HARD_LAND = "hard"


#Funciones geométricas
def _point_in_poly(x: float, y: float, poly: List[Tuple[float, float]], eps=1e-6) -> bool:

    n = len(poly)
    if n < 3:
        return False

    # Borde explícito
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if _point_on_segment(x, y, x1, y1, x2, y2, eps):
            return True

    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)):
            denom = (yj - yi) if (yj - yi) != 0 else 1e-9
            x_inter = (xj - xi) * (y - yi) / denom + xi
            if x < x_inter:
                inside = not inside
        j = i
    return inside


def _point_on_segment(px, py, x1, y1, x2, y2, eps=1e-6):
    """Comprueba si (px,py) está en el segmento [(x1,y1),(x2,y2)]"""
    cross = abs((px - x1) * (y2 - y1) - (py - y1) * (x2 - x1))
    if cross > eps:
        return False
    dot = (px - x1) * (px - x2) + (py - y1) * (py - y2)
    if dot > eps:
        return False
    return True


def _point_in_circle(x, y, cx, cy, r_cm):
    dx, dy = x - cx, y - cy
    return (dx * dx + dy * dy) <= (r_cm * r_cm + 1e-6)


# --- API pública ---

def set_geofence(self,
                 max_x_cm=_DEFAULT_MAX_X_CM,
                 max_y_cm=_DEFAULT_MAX_Y_CM,
                 max_z_cm=_DEFAULT_MAX_Z_CM,
                 z_min_cm=0.0,
                 mode=_MODE_SOFT_ABORT,
                 poll_interval_s=_DEFAULT_POLL_S):

    # Normaliza límites (0 o negativo = sin inclusión)
    lim: Dict[str, float] = {}
    if max_x_cm and max_x_cm > 0:
        lim["max_x"] = float(max_x_cm)
    if max_y_cm and max_y_cm > 0:
        lim["max_y"] = float(max_y_cm)
    if max_z_cm and max_z_cm > 0:
        lim["max_z"] = float(max_z_cm)

    #  Soporte para z_min
    lim["zmin"] = float(z_min_cm) if z_min_cm is not None else 0.0

    self._gf_limits = lim if lim else None
    self._gf_mode = mode if mode in (_MODE_SOFT_ABORT, _MODE_HARD_LAND) else _MODE_SOFT_ABORT
    self._gf_poll_s = max(0.05, float(poll_interval_s))

    # Si no había centro definido, por defecto (0,0)
    if not hasattr(self, "_gf_center"):
        self._gf_center = (0.0, 0.0)
    self._gf_enabled = True

    # Asegura contenedores (SIEMPRE como listas de dicts)
    if not hasattr(self, "_gf_excl_polys"):
        self._gf_excl_polys = []
    if not hasattr(self, "_gf_excl_circles"):
        self._gf_excl_circles = []

    # Histéresis / rachas
    self._gf_violation_streak = 0

    # (Re)inicia el monitor con seguridad
    _stop_geofence_monitor(self)
    _start_geofence_monitor(self)

    span_txt = f"ancho={lim.get('max_x', 0):.0f}x{lim.get('max_y', 0):.0f} cm" if lim else "SIN inclusión (solo exclusiones)"
    maxz_txt = f"z_max={lim.get('max_z', 0):.0f} cm" if lim else "z_max=∞"
    zmin_txt = f"z_min={lim.get('zmin', 0):.0f} cm"
    print(f"[geofence] Activado: {span_txt}, {zmin_txt}, {maxz_txt}, modo={self._gf_mode}")


def disable_geofence(self):
    self._gf_enabled = False
    _stop_geofence_monitor(self)
    print("[geofence] Desactivado.")


def recenter_geofence(self):
    """Reancla el centro del cubo al punto actual."""
    pose = getattr(self, "pose", None)
    if pose:
        self._gf_center = (float(getattr(pose, "x_cm", 0.0) or 0.0),
                           float(getattr(pose, "y_cm", 0.0) or 0.0))
        print(f"[geofence] Recentrado en {self._gf_center}")
    else:
        print("[geofence] No se pudo recentrar (pose desconocida).")


def add_exclusion_circle(self, cx, cy, r_cm, z_min_cm=None, z_max_cm=None):

    if not hasattr(self, "_gf_excl_circles"):
        self._gf_excl_circles = []

    cx = float(cx)
    cy = float(cy)
    r = abs(float(r_cm))

    item = {
        "cx": cx,
        "cy": cy,
        "r": r,
        "zmin": float(z_min_cm) if z_min_cm is not None else None,
        "zmax": float(z_max_cm) if z_max_cm is not None else None
    }

    self._gf_excl_circles.append(item)
    _ensure_gf_monitor(self)

    z_range = f"z∈[{item['zmin']},{item['zmax']}]" if item['zmin'] is not None and item[
        'zmax'] is not None else "z=todas"
    print(f"[geofence] Círculo añadido: centro=({cx:.1f},{cy:.1f}), r={r:.1f}cm, {z_range}")

    return item


def add_exclusion_poly(self, points, z_min_cm=None, z_max_cm=None):

    if not hasattr(self, "_gf_excl_polys"):
        self._gf_excl_polys = []

    poly = [(float(x), float(y)) for (x, y) in points]

    item = {
        "poly": poly,
        "zmin": float(z_min_cm) if z_min_cm is not None else None,
        "zmax": float(z_max_cm) if z_max_cm is not None else None
    }

    self._gf_excl_polys.append(item)
    _ensure_gf_monitor(self)

    z_range = f"z∈[{item['zmin']},{item['zmax']}]" if item['zmin'] is not None and item[
        'zmax'] is not None else "z=todas"
    print(f"[geofence] Polígono añadido: {len(poly)} vértices, {z_range}")

    return item


def clear_exclusions(self):
    self._gf_excl_polys = []
    self._gf_excl_circles = []
    print("[geofence] Exclusiones eliminadas.")


def _start_geofence_monitor(self, force=False):
    """Arranca el hilo del monitor si no está ya en marcha."""
    if getattr(self, "_gf_monitoring", False) and not force:
        return
    self._gf_monitoring = True
    t = threading.Thread(target=_gf_monitor_loop, args=(self,), daemon=True)
    self._gf_thread = t
    t.start()
    print("[geofence] Monitor iniciado.")


def _stop_geofence_monitor(self):
    """Detiene el hilo del monitor si está activo."""
    self._gf_monitoring = False
    t = getattr(self, "_gf_thread", None)
    if t and isinstance(t, threading.Thread):
        try:
            t.join(timeout=1.0)
        except Exception:
            pass
    self._gf_thread = None
    print("[geofence] Monitor detenido.")


def _ensure_gf_monitor(self):
    """Garantiza que el monitor esté corriendo cuando _gf_enabled es True."""
    if getattr(self, "_gf_enabled", False) and not getattr(self, "_gf_monitoring", False):
        _start_geofence_monitor(self)


# --- Funciones internas ---

def _gf_monitor_loop(self):
    """Bucle de supervisión del geofence (inclusión + exclusiones)."""
    self._gf_violation_streak = 0
    while getattr(self, "_gf_monitoring", False) and getattr(self, "_gf_enabled", False):
        try:
            st = getattr(self, "state", "")
            if st not in ("flying", "landing", "hovering", "takingoff"):
                time.sleep(self._gf_poll_s)
                continue

            pose = getattr(self, "pose", None)
            if pose is None:
                time.sleep(self._gf_poll_s)
                continue

            x = float(getattr(pose, "x_cm", 0.0) or 0.0)
            y = float(getattr(pose, "y_cm", 0.0) or 0.0)
            z = float(getattr(pose, "z_cm", getattr(self, "height_cm", 0.0)) or 0.0)

            #  Validación completa
            violated = (not _inside_inclusion(self, x, y, z)) or _inside_any_exclusion(self, x, y, z)

            if violated:
                self._gf_violation_streak += 1
            else:
                self._gf_violation_streak = 0

            # Dispara acción si hay 2 lecturas consecutivas de violación
            if self._gf_violation_streak >= 2:
                _handle_violation(self)
                # Si es HARD, dejamos que _handle_violation pare el monitor o aterrizaje
                if getattr(self, "_gf_mode", _MODE_SOFT_ABORT) == _MODE_HARD_LAND:
                    # En HARD, no seguimos tras ordenar el aterrizaje
                    break

        except Exception as e:
            print(f"[geofence] Error monitor: {e}")

        time.sleep(self._gf_poll_s)

    # Sale del bucle
    self._gf_monitoring = False


def _inside_inclusion(self, x, y, z):
    """
    Devuelve True si (x,y,z) está dentro de la inclusión; si no hay inclusión, devuelve True.

     CORREGIDO: max_x y max_y son anchos TOTALES, se dividen por 2 para obtener semiejes.
    """
    lim = getattr(self, "_gf_limits", None)
    if not lim:
        return True  # SOLO exclusiones

    cx, cy = getattr(self, "_gf_center", (0.0, 0.0))
    max_x = float(lim.get("max_x", 0.0) or 0.0)
    max_y = float(lim.get("max_y", 0.0) or 0.0)
    max_z = float(lim.get("max_z", 0.0) or 0.0)
    zmin = float(lim.get("zmin", 0.0) or 0.0)

    #  FIX CRÍTICO: max_x/max_y son ANCHOS TOTALES del rectángulo, no semiejes
    # Dividimos por 2 para obtener la distancia máxima desde el centro
    half_x = max_x / 2.0
    half_y = max_y / 2.0

    # Si algún eje no está definido o es 0 => sin límite en ese eje
    in_x = True if max_x <= 0 else (abs(x - cx) <= half_x)
    in_y = True if max_y <= 0 else (abs(y - cy) <= half_y)
    in_z = True if max_z <= 0 else (zmin <= z <= max_z)

    return in_x and in_y and in_z


def _inside_any_exclusion(self, x, y, z):
    """
    Verifica si (x,y,z) está dentro de alguna zona de exclusión.

     CORREGIDO: maneja exclusivamente dicts y valida rangos Z correctamente.
    """
    # Polígonos: deben ser dicts {poly, zmin, zmax}
    for entry in list(getattr(self, "_gf_excl_polys", [])):
        if not isinstance(entry, dict):
            continue  # Ignora formatos antiguos/corruptos

        poly = entry.get("poly", [])
        zmin = entry.get("zmin")
        zmax = entry.get("zmax")

        if _point_in_poly(x, y, poly):
            #  Validación de altura Z
            z_ok = (zmin is None or z >= zmin) and (zmax is None or z <= zmax)
            if z_ok:
                print(f"[geofence]  VIOLACIÓN POLY @ ({x:.1f},{y:.1f},{z:.1f})")
                return True

    # Círculos: deben ser dicts {cx, cy, r, zmin, zmax}
    for entry in list(getattr(self, "_gf_excl_circles", [])):
        if not isinstance(entry, dict):
            continue  # Ignora formatos antiguos/corruptos

        cx_c = entry.get("cx")
        cy_c = entry.get("cy")
        r = entry.get("r")
        zmin = entry.get("zmin")
        zmax = entry.get("zmax")

        if _point_in_circle(x, y, cx_c, cy_c, r):
            #  Validación de altura Z
            z_ok = (zmin is None or z >= zmin) and (zmax is None or z <= zmax)
            if z_ok:
                print(f"[geofence]  VIOLACIÓN CIRCLE @ ({x:.1f},{y:.1f},{z:.1f})")
                return True

    return False


def _handle_violation(self):
    """Maneja una violación del geofence."""
    mode = getattr(self, "_gf_mode", _MODE_SOFT_ABORT)

    # Evita reentradas
    if getattr(self, "_gf_last_report", None) != mode:
        print(f"[geofence]  Violación detectada (modo={mode}).")
        self._gf_last_report = mode

    # Señales de abortar tareas de alto nivel
    setattr(self, "_goto_abort", True)
    setattr(self, "_mission_abort", True)

    if mode == _MODE_HARD_LAND:
        # No dispares múltiples lands
        if getattr(self, "_gf_landing_initiated", False):
            return
        self._gf_landing_initiated = True

        st = getattr(self, "state", "")
        # Si ya no estamos volando, no intentes aterrizar
        if st not in ("flying", "hovering", "takingoff"):
            return

        print("[geofence]  Aterrizando de emergencia…")
        # Dejamos de monitorear para no entrar en bucles
        self._gf_monitoring = False

        def do_land():
            try:
                time.sleep(_HARD_LAND_DELAY)
                self.Land(blocking=True)
            except Exception as e:
                print(f"[geofence] Error Land: {e}")
            finally:
                self._gf_landing_initiated = False

        threading.Thread(target=do_land, daemon=True).start()