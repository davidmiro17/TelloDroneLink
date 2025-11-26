import threading
import time

#Función en la que intentamos leer la altura del backend del Tello, y si falla usamos el valor guardado en self.height_cm
def _read_height_cm_runtime(self) -> float:

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


#Función pública del aterrizaje
def Land(self, blocking=True, callback=None, params=None):

    #Evitamos lanzar 2 lands a la vez
    if getattr(self, "_landing_in_progress", False):
        print("[land] Ya hay un aterrizaje en curso; ignoro la petición duplicada.")
        return True

    setattr(self, "_landing_in_progress", True)

    #Comprobaciones rápidas del estado
    st = getattr(self, "state", "")
    if st != "flying":

        print("[land] Estado actual no es 'flying'; no mando 'land'. Normalizo el estado a 'connected'.")
        try:
            self.state = "connected"

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


#Función que contiene toda la lógica del aterrizaje completo
def _land(self, callback=None, params=None):

    try:
        #Estado intermedio
        try:
            self.state = "landing"
        except Exception:
            pass

        #Si ya está a ras de suelo, no se pone como 'land'
        h0 = _read_height_cm_runtime(self)
        if h0 <= 20.0:
            print("[land] Altura inicial ≤ 20 cm. Ya está en el suelo; no mando 'land'.")
            _normalize_after_land(self)
            _do_callback(callback, params)
            return

        # Enviar 'land'
        try:
            resp = self._send("land")
            print(f"[land] Respuesta SDK: {resp!r}")
        except Exception as e:

            print(f"[land] Aviso al enviar 'land': {e}")

        # Esperamos a que realmente baje (por altura) con timeout
        TIMEOUT_S = 15.0      # ventana  para que baje
        LOW_CM    = 15.0      # umbral “en suelo”
        t0 = time.time()
        while time.time() - t0 < TIMEOUT_S:
            h = _read_height_cm_runtime(self)
            if h <= LOW_CM:
                break
            time.sleep(0.2)

        #Normalizamos el estado al final
        _normalize_after_land(self)
        print("[land] Completado.")


        _do_callback(callback, params)

    finally:

        try:
            setattr(self, "_landing_in_progress", False)
        except Exception:
            pass



#Función para limpiar el estado despues de aterrizar
def _normalize_after_land(self):

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

    if not cb:
        return
    try:
        cb(params)
    except TypeError:
        try:
            cb()
        except Exception:
            pass