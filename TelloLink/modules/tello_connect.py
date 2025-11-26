import threading
import time
from djitellopy import Tello


def _connect(self, freq=5, callback=None, params=None):
    try:
        # Crea el objeto Tello y conecta
        self._tello = Tello()

        #  Aumentar timeout para comandos lentos
        self._tello.RESPONSE_TIMEOUT = 15

        self._tello.connect()

        # Limpiamos el stream
        try:
            self._tello.streamoff()
        except Exception:
            pass

        # Estado del Tello pasa a ser conectado
        self.state = "connected"

        # POSE: asegurar objeto y sincronizar Z (altura) con barómetro
        try:
            if not hasattr(self, "pose") or self.pose is None:
                self.pose = self.PoseVirtual()
            self.pose.set_from_telemetry(height_cm=getattr(self, "height_cm", None))
        except Exception:
            pass

        return True

    except Exception as Err:
        print("Error conectando a Tello:", Err)
        self._tello = None
        self.state = "disconnected"
        return False


def connect(self, freq=5, blocking=True, callback=None, params=None):
    if self.state != "disconnected":
        return False

    if blocking:
        return _connect(self, freq=freq, callback=callback, params=params)
    else:
        t = threading.Thread(
            target=_connect, args=(self,), kwargs=dict(freq=freq, callback=callback, params=params), daemon=True
        )
        t.start()
        return True


def disconnect(self):
    # Paramos la telemetría si está activa
    try:
        self.stopTelemetry()
    except Exception:
        pass

    # Cerramos la conexión con el dron
    try:
        if self._tello:
            try:
                self._tello.streamoff()
            except Exception:
                pass
            self._tello.end()
    finally:
        self._tello = None
        self.state = "disconnected"

    # Pequeña pausa de seguridad
    time.sleep(0.2)
    return True


def _require_connected(self):
    if not hasattr(self, "_tello") or self._tello is None:
        raise RuntimeError("No hay backend '_tello' inicializado. ¿Llamaste connect()?")


def _send(self, cmd: str) -> str:
    _require_connected(self)

    # djitellopy expone distintos nombres según la versión, con esto nos aseguramos de que funcione con versiones más antiguas
    if hasattr(self._tello, "send_read_command"):
        resp = self._tello.send_read_command(cmd)
        return str(resp)

    if hasattr(self._tello, "send_command_with_return"):
        resp = self._tello.send_command_with_return(cmd)
        return str(resp)

    if hasattr(self._tello, "send_control_command"):
        res = self._tello.send_control_command(cmd)
        if isinstance(res, bool):
            return "ok" if res else "error"
        return str(res)

    raise RuntimeError("El backend Tello no soporta envío textual en la versión actual.")