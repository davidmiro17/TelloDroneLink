import threading
import time

# Intentamos importar la PoseVirtual (opcional: si no existe no se rompe)
try:
    from TelloLink.modules.tello_pose import PoseVirtual
except Exception:
    PoseVirtual = None


def _telemetry_loop(self, period_s: float):

    if not hasattr(self, "_pose_takeoff_synced"):
        self._pose_takeoff_synced = False

    while not getattr(self, "_telemetry_stop", True):

        # Si no hay conexión, esperamos y reintentamos
        if getattr(self, "_tello", None) is None or getattr(self, "state", "disconnected") == "disconnected":
            time.sleep(period_s)
            continue

        # Valores locales para sincronización de pose
        height_val = None
        yaw_val = None

        # Altura (cm)
        try:
            h = self._tello.get_height()
            if h is not None:
                self.height_cm = max(0, int(h))
                height_val = self.height_cm
        except Exception:
            pass

        #Yaw (grados)
        try:
            # Primero intentamos get_yaw()
            gy = getattr(self._tello, "get_yaw", None)
            if callable(gy):
                y = gy()
            else:
                # Alternativa: leer del estado  si existe
                y = None
                gc = getattr(self._tello, "get_current_state", None)
                if callable(gc):
                    try:
                        st = gc()
                        # djitellopy suele mostrar 'yaw' en grados
                        if isinstance(st, dict):
                            y = st.get("yaw", None)
                    except Exception:
                        y = None

            if y is not None:

                self.yaw_deg = float(y)
                yaw_val = self.yaw_deg
        except Exception:
            pass

        # Batería (%)
        try:
            b = self._tello.get_battery()
            if b is not None:
                self.battery_pct = max(0, int(b))
        except Exception:
            pass

        #Temperatura (°C)
        try:
            t = self._tello.get_temperature()
            if t is not None:
                self.temp_c = float(t)
        except Exception:
            pass

        # WiFi (0..100 aprox)
        try:
            w = self._tello.get_wifi()
            if w is not None:
                self.wifi = int(w)
        except Exception:
            pass

        #  Tiempo de vuelo (s)
        try:
            ft = self._tello.get_flight_time()
            if ft is not None:
                self.flight_time_s = max(0, int(ft))
        except Exception:
            pass

        #  Sincronización de PoseVirtual (z/yaw)
        try:
            # Si aún no existe pose, la creamos
            if not hasattr(self, "pose") or self.pose is None:
                if PoseVirtual is not None:
                    self.pose = PoseVirtual()

            if hasattr(self, "pose") and self.pose is not None:
                # Altura (z)
                self.pose.set_from_telemetry(height_cm=height_val)

                # Yaw absoluto -> relativo
                if yaw_val is not None:
                    try:
                        self.pose.set_heading_from_absolute_yaw(yaw_val)
                    except Exception:
                        pass

                # Al pasar a estado 'flying' por primera vez, fijamos referencia de yaw del vuelo
                try:
                    if getattr(self, "state", "") == "flying":
                        if yaw_val is not None and not getattr(self, "_pose_takeoff_synced", False):
                            self.pose.set_takeoff_reference(yaw_val)
                            self._pose_takeoff_synced = True
                    else:
                        # cuando no estamos volando, reseteamos la marca para el siguiente vuelo
                        self._pose_takeoff_synced = False
                except Exception:
                    pass
        except Exception:

            pass


        self.telemetry_ts = time.time()

        time.sleep(period_s)


def startTelemetry(self, freq_hz: int = 5):

    if freq_hz <= 0:
        freq_hz = 5

    if getattr(self, "_telemetry_thread", None) and self._telemetry_thread.is_alive():
        return False

    # Inicialización de atributos (si no existen)
    self.height_cm = getattr(self, "height_cm", 0)
    self.battery_pct = getattr(self, "battery_pct", None)
    self.temp_c = getattr(self, "temp_c", None)
    self.wifi = getattr(self, "wifi", None)
    self.flight_time_s = getattr(self, "flight_time_s", 0)
    self.telemetry_ts = time.time()

    # Creamos aquí la pose
    if not hasattr(self, "pose") or self.pose is None:
        if PoseVirtual is not None:
            self.pose = PoseVirtual()

    self._telemetry_stop = False
    period_s = 1.0 / float(freq_hz)

    th = threading.Thread(target=_telemetry_loop, args=(self, period_s), daemon=True)
    self._telemetry_thread = th
    th.start()
    return True


def stopTelemetry(self):

    self._telemetry_stop = True
    th = getattr(self, "_telemetry_thread", None)
    if th and th.is_alive():
        th.join(timeout=2.0)
    self._telemetry_thread = None
    return True