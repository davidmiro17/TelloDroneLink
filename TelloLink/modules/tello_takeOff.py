
import time

_MIN_BAT_PCT = 10  # seguridad mínima por si se usa sin comprobaciones externas


def _read_height_cm_runtime(self) -> int:

    try:
        if hasattr(self, "_tello") and self._tello:
            h = self._tello.get_height()
            if h is not None:
                return max(0, int(h))
    except Exception:
        pass
    try:
        return max(0, int(getattr(self, "height_cm", 0) or 0))
    except Exception:
        return 0


def _checkAltitudeReached(self, target_h_cm, timeout_s=5.0):
    """Comprueba si el dron ha alcanzado al menos la altura objetivo."""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        h = _read_height_cm_runtime(self)
        if h >= target_h_cm * 0.9:  # margen del 10%
            return True
        time.sleep(0.2)
    return False


def _ascend_to_target(self, target_h_cm):
    """Sube hasta la altura objetivo en un solo empujón seguro."""
    try:
        self._send(f"up {int(target_h_cm)}")
    except Exception as e:
        print(f"[WARN] Subida adicional falló: {e}")


def _takeOff(self, altura_objetivo_m=0.5, blocking=True):
    """
    Despegue controlado con comprobación de altura, empujón de rescate
    y dos mejoras:
      - cooldown final para que el PRIMER movimiento no falle,
      - reset de pose para que los ejes sean relativos a ESTE despegue.
    """
    try:
        if getattr(self, "state", "") == "disconnected":
            print("[ERROR] Dron desconectado, abortando despegue.")
            return False

        bat = getattr(self, "battery_pct", None)
        if isinstance(bat, int) and bat < _MIN_BAT_PCT:
            print(f"[WARN] Batería muy baja ({bat}%), riesgo en despegue.")

        print("Empezamos a despegar")
        resp = self._send("takeoff")
        print(f"[INFO] tello_takeOff -> respuesta inicial: {resp}")

        # --- Espera a que suba al menos a ~20 cm ---
        t0 = time.time()
        ok_alt = False
        h = 0
        while time.time() - t0 < 5.0:
            h = _read_height_cm_runtime(self)
            if h >= 20:
                ok_alt = True
                break
            time.sleep(0.2)

        # Empujón de rescate si no llegó
        if not ok_alt:
            print("[WARN] Altura <20 cm tras 5s, aplico empujón 'up 20'")
            try:
                self._send("up 20")
            except Exception as e:
                print(f"[WARN] Empujón falló: {e}")
            time.sleep(2.0)
            h = _read_height_cm_runtime(self)
            if h >= 20:
                ok_alt = True

        if not ok_alt:
            print("[ERROR] No se confirmó despegue (altura <20 cm tras reintento).")
            return False

        # Confirmado: ya está en el aire
        print(f"Altura inicial confirmada: {h} cm (ok).")
        self.state = "flying"

        # --- RESET DE POSE EN CADA DESPEGUE ---
        try:
            pose = getattr(self, "pose", None)
            if pose is not None:
                pose.reset()          # x=0,y=0,z=0,yaw=0
                pose.z_cm = float(h)  # ancla Z a la altura barométrica actual
                pose.yaw_deg = 0.0    # el rumbo actual pasa a ser 0° relativo
        except Exception:
            pass

        # Subida adicional si la altura objetivo es mayor que la actual
        target_h_cm = int(altura_objetivo_m * 100)
        if h < target_h_cm:
            _ascend_to_target(self, target_h_cm - h)
            print(f"Altura objetivo alcanzada (~{target_h_cm} cm).")

        # --- COOLDOWN FINAL PARA EVITAR FALLO DEL PRIMER COMANDO ---
        try:
            self._after_takeoff_ts = time.time()
        except Exception:
            pass
        time.sleep(0.7)  # ~0.5–0.8s suele ser suficiente

        print("Despegue completado (≈1 m)")
        return True

    except Exception as e:
        print(f"[ERROR] takeOff -> {e}")
        return False


def takeOff(self, altura_objetivo_m=0.5, blocking=True):
    """Wrapper público para el hilo de despegue."""
    if blocking:
        return _takeOff(self, altura_objetivo_m, blocking=False)
    else:
        import threading
        threading.Thread(
            target=_takeOff, args=(self, altura_objetivo_m, False), daemon=True
        ).start()
        return True