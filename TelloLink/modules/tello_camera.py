import time
import os
from datetime import datetime

def stream_on(self):
    self._require_connected() #Comprueba que el dron está conectado

    try:
        self._tello.streamoff() #Por si de una sesión anterior había una sesión de stream colgada, se cierra el stream
        time.sleep(0.2)
    except Exception:
        pass


    self._tello.streamon() #Ahora si que activa el vídeo con streamon()
    time.sleep(0.2)


    try:
        self._frame_reader = self._tello.get_frame_read() #Se crea un objeto frame_reader que, en segundo plane captura contínuamente frames de la cámara del Tello
    except Exception as e:                                #self._frame_reader.frame contendrá el último fotograma disponible.
        try: self._tello.streamoff()    #Si falla, se intenta apagar el stream para que quede "limpio"
        except Exception: pass
        raise RuntimeError(f"No se pudo obtener frame reader: {e}") #Si no se consigue, se lanza error para avisar al usuario

    return True


def stream_off(self):
    try:
        if getattr(self, "_frame_reader", None) is not None: #Si existe un frame_reader activo, se elimina
            self._frame_reader = None
        self._tello.streamoff() #Se manda al Tello la orden de parar el streaming de vídeo
        time.sleep(0.1)
    except Exception: #Si algo falla (no había  stream activo) se ignora
        pass
    return True


def get_frame(self):
    fr = getattr(self, "_frame_reader", None) #Busca si existe un objeto frame_reader
    if fr is None: #Si no existe, devuelve None
        return None
    frame = fr.frame #Obtiene el último frame (imagen) disponible del dron

    if frame is None: #Si el frame no está listo, devuelve None
        return None
    return frame #Si está listo, se devuelve el frame como objeto


def snapshot(self, path: str | None = None):  #Función para capturar imagen
    frame = get_frame(self)  #Se intenta obtener la última imagen del Tello
    if frame is None:        #Si no hay imagen, espera y lo intenta de nuevo
        time.sleep(0.2)
        frame = get_frame(self)
        if frame is None:
            raise RuntimeError("No hay frame disponible (¿stream_on activo?)")

    if path is None:  #Si no le pasamos la ruta para guardar la foto
        out_dir = os.path.join(".", "snapshots")
        os.makedirs(out_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(out_dir, f"tello_{ts}.jpg")

    try:
        import cv2
    except Exception as e:
        raise RuntimeError("Falta OpenCV (pip install opencv-python)") from e

    #Conversión de color para que no salga azul
    if getattr(self, "FRAME_FORMAT", "RGB") == "RGB":
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    ok = cv2.imwrite(path, frame)
    if not ok:
        raise RuntimeError(f"No se pudo escribir el snapshot en: {path}")

    return path