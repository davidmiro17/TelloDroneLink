# TelloLink/modules/tello_land.py
# Estilo robusto “por pasos”, análogo a tello_takeOff_robusto:
# - Evita dobles aterrizajes con un flag interno.
# - No manda 'land' si ya estás en el suelo (≤20 cm) o no estás en vuelo.
# - Acepta respuestas no-estándar del SDK (no aborta por '44', etc.).
# - Espera a que realmente baje (por altura), con timeout razonable.
# - Normaliza self.state al finalizar.

import threading
import time


def _read_height_cm_runtime(self) -> float:
    """
    Lectura robusta de altura para aterrizaje:
    1) Intenta SDK (get_height).
    2) Si falla, cae a self.height_cm.
    """
    try:
        if hasattr(self, "_tello") and self._tello:
            h = self._tello.get_height()
            if h is not None:
                return float(max(0, int(h)))
    except Exception:
        pass
    try:
        return float(max(0, int(getattr(self, "height_cm", 0) or 0)))
    except Exception:
        return 0.0


# --- API público: Land ---
def Land(self, blocking=True, callback=None, params=None):
    """
    Aterriza el dron. Si blocking=True, espera a terminar.
    Devuelve True si se inicia (o ya estaba) el aterrizaje.
    """
    # 0) Antirreentradas: evita lanzar 2 lands a la vez
    if getattr(self, "_landing_in_progress", False):
        print("[land] Ya hay un aterrizaje en curso; ignoro la petición duplicada.")
        return True

    setattr(self, "_landing_in_progress", True)

    # 1) Comprobaciones rápidas de estado
    st = getattr(self, "state", "")
    if st != "flying":
        # Igual que en takeOff robusto: no fuerces comandos si el estado no aplica.
        print("[land] Estado actual no es 'flying'; no mando 'land'. Normalizo a 'connected'.")
        try:
            self.state = "connected"
            # Por estética de la UI, fija z=0 si tienes pose
            if hasattr(self, "pose") and self.pose:
                try:
                    self.pose.z_cm = 0.0
                except Exception:
                    pass
        finally:
            setattr(self, "_landing_in_progress", False)
        return True

    # 2) Modo bloqueante o no-bloqueante
    if blocking:
        try:
            _land(self, callback=callback, params=params)
        finally:
            setattr(self, "_landing_in_progress", False)
        return True
    else:
        def _runner():
            try:
                _land(self, callback=callback, params=params)
            finally:
                setattr(self, "_landing_in_progress", False)
        threading.Thread(target=_runner, daemon=True).start()
        return True


# --- Implementación real del aterrizaje ---
def _land(self, callback=None, params=None):
    """
    Aterrizaje robusto:
      Paso 1) Marca estado 'landing'
      Paso 2) Si ya está en el suelo (≤20 cm), no manda 'land' y normaliza.
      Paso 3) Envía 'land' (acepta respuesta no 'ok' sin abortar)
      Paso 4) Espera descenso real por altura (timeout)
      Paso 5) Normaliza estado y callback
    """
    try:
        # Paso 1) Estado intermedio
        try:
            self.state = "landing"
        except Exception:
            pass

        # Paso 2) Si ya está a ras de suelo, no fuerces 'land'
        h0 = _read_height_cm_runtime(self)
        if h0 <= 20.0:
            print("[land] Altura inicial ≤ 20 cm. Ya está en el suelo; no mando 'land'.")
            _normalize_after_land(self)
            _do_callback(callback, params)
            return

        # Paso 3) Enviar 'land' (sin tratar respuesta rara como error duro)
        try:
            resp = self._send("land")
            print(f"[land] Respuesta SDK: {resp!r}")   # puede ser 'ok', 'error ...', '44', etc.
        except Exception as e:
            # No abortamos; pasamos a monitorizar altura como en takeOff robusto
            print(f"[land] Aviso al enviar 'land': {e}")

        # Paso 4) Espera a que realmente baje (por altura) con timeout
        TIMEOUT_S = 15.0      # ventana cómoda para que baje
        LOW_CM    = 15.0      # umbral “en suelo”
        t0 = time.time()
        while time.time() - t0 < TIMEOUT_S:
            h = _read_height_cm_runtime(self)
            if h <= LOW_CM:
                break
            time.sleep(0.2)

        # Paso 5) Normaliza estado final y limpieza
        _normalize_after_land(self)
        print("[land] Completado.")

        # Callback
        _do_callback(callback, params)

    finally:
        # Limpia flag de antirreentradas pase lo que pase
        try:
            setattr(self, "_landing_in_progress", False)
        except Exception:
            pass


# ----- Helpers internos -----

def _normalize_after_land(self):
    """Deja el estado en 'connected' y, si existe pose, Z=0."""
    try:
        self.state = "connected"
    except Exception:
        pass
    try:
        if hasattr(self, "pose") and self.pose:
            self.pose.z_cm = 0.0
    except Exception:
        pass


def _do_callback(cb, params):
    """Llama al callback si existe, tolerando firmas con/sin parámetro."""
    if not cb:
        return
    try:
        cb(params)
    except TypeError:
        try:
            cb()
        except Exception:
            pass