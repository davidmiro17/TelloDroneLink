import threading, time

_LOOP_SLEEP = 0.001  # baja carga CPU

def _need_cv2(): #Función para hacer el import de OpenCV, librería necesaria para trabajar con vídeo e imagen en Pyhton
    try:
        import cv2
    except Exception as e:
        raise RuntimeError("Falta OpenCV (pip install opencv-python)") from e #Si falla, se lanza este mensaje al usuario, el "from e" encadena el error original al nuevo error, para saber que pasó en el error original y no perder su pista.

def _convert_for_cv2(self, frame):
    import cv2
    fmt = getattr(self, "FRAME_FORMAT", "RGB").upper() #lee el formato del frame (RGB/BGR) con por defecto "RGB" y se normaliza a mayúsculas
    return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if fmt == "RGB" else frame #Convierte a BGR solo si el frame está en RGB (para que OpenCV lo muestre con colores correctos)

def _ensure_stream(self):   #Función para preparar el stream si no está ya activo

    if getattr(self, "_frame_reader", None) is not None: #Se intenta obtener el atributo _frame_reader, si existe (no es None) se devuelve return y no pasa nada.
        return

    if hasattr(self, "stream_on"): #Comprueba si el objeto tiene el metodo stream_on
        self.stream_on()  #Si lo tiene, activa el stream
        return

    self._tello.streamoff(); time.sleep(0.2) #Se reinicia el stream de vídeo, y crea el stream reader.
    self._tello.streamon();  time.sleep(0.3)
    self._frame_reader = self._tello.get_frame_read()

def _video_loop(self, window_name, resize):  #Esta función consiste en el bucle que mantiene la ventana de vídeo en la pantalla
    import cv2
    while getattr(self, "_video_run", False): #Mientras el vídeo este corriendo
        fr = getattr(self, "_frame_reader", None) #Busca si hay un frame reader
        frame = None if fr is None else fr.frame #Si existe toma el último frame de su buffer, si no existe o no hay frame aún, = None
        if frame is None: #Si áun no hay frame (aún no ha llegado vídeo)
            time.sleep(_LOOP_SLEEP) #Hace una pausa muy corta y vuelve al bucle
            continue

        bgr = _convert_for_cv2(self, frame) #Convierte el frame RGB a BGR, para que no se vea azul
        if resize: #Si se ha pedido un tamaño concreto de ventana, se redimensiona, si por algún motivo falla, se ignora y sigue
            w, h = resize
            try:
                bgr = cv2.resize(bgr, (w, h))
            except:
                pass

        try:
            cv2.imshow(window_name, bgr) #Se muestra el vídeo en una ventana
            if cv2.waitKey(1) & 0xFF == ord('q'): #Si después de 1 ms, se pulsa q, se desactiva _video_run y sale del bucle (se cierra la ventana)
                self._video_run = False
                break
        except: #Si algo falla previamente en el imshow, se ignora el error
            pass
        time.sleep(_LOOP_SLEEP)

    #Al salir del bucle de vídeo, se intentan cerrar primero esa ventana, y si falla, se cierran todas
    try:
        import cv2
        cv2.destroyWindow(window_name)
    except:
        try:
            import cv2
            cv2.destroyAllWindows()
        except:
            pass

def start_video(self, resize=None, window_name="Tello FPV"):
    _need_cv2()
    self._require_connected()
    if getattr(self, "_video_run", False): #Si ya hay un vídeo corriendo, no arranca otro
        return True
    _ensure_stream(self) #Se asegura de que el stream de camara esté preparado
    self._video_run = True
    self._video_thread = threading.Thread(
        target=_video_loop, args=(self, window_name, resize), daemon=True
    )
    self._video_thread.start()
    return True

def stop_video(self):

    self._video_run = False
    th = getattr(self, "_video_thread", None)
    if th and th.is_alive():
        try:
            th.join(timeout=2.0)
        except:
            pass
    self._video_thread = None
    return True





#Variante de la función bloqueante
def show_video_blocking(self, resize=None, window_name="Tello FPV"):

    _need_cv2()
    self._require_connected()
    _ensure_stream(self)

    import cv2

    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    except Exception:
        pass

    try:
        while True:
            fr = getattr(self, "_frame_reader", None)
            frame = None if fr is None else fr.frame
            if frame is None:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            bgr = _convert_for_cv2(self, frame)
            if resize:
                w, h = resize
                try:
                    bgr = cv2.resize(bgr, (w, h))
                except Exception:
                    pass

            cv2.imshow(window_name, bgr)
            # En macOS, waitKey en el hilo principal es imprescindible para refrescar la ventana
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        try:
            cv2.destroyWindow(window_name)
        except Exception:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass