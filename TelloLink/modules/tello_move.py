import time

#Valores máximos y mínimos del SDK de Tello
MIN_STEP = 20       # cm (límite inferior de movimiento del Tello)
MAX_STEP = 500      # cm (límite superior de movimiento del Tello)
MIN_SPEED = 10      # cm/s
MAX_SPEED = 100     # cm/s
COOLDOWN_S = 0.4    # pequeña pausa entre comandos por seguridad



def _resp_is_ok(resp):
    s = str(resp).strip().lower()
    # Acepta 'ok' y también respuestas no vacías que NO contengan 'error'
    return s == "ok" or (s and "error" not in s)


def _ensure_techo(self):
    # Techo de seguridad por defecto: 1.5 m si no está definido
    if not hasattr(self, "TECHO_M") or self.TECHO_M is None: #Si el atributo TECHO_ no existe o está definido como none, le popnemos 1,5 m.
        self.TECHO_M = 2.5

def _distancia_acotada(dist_cm: int) -> int:
    # Acota la distancia a los límites del SDK
    try:
        d = int(dist_cm)
    except Exception: #Si lo que se le pasa no se puede convertir en un entero, se pone el valor mínimo seguro
        d = MIN_STEP
    return max(MIN_STEP, min(MAX_STEP, d)) #Si d es mas pequeño que el mínimo se sube a 20 y si es mayor que el maximo se baja a 500

def _move(self, verb, dist_cm):
    #Envia un movimiento horizontal simple ("forward", "back", "left", "right" son los tipos de "verb" (opciones de movimiento))
    self._require_connected()
    d = _distancia_acotada(dist_cm)
    resp = self._send(f"{verb} {d}") #Se envia el tipo de verb y su distancia, por ejemplo forward y 50)
    if not _resp_is_ok(resp):   #Si el dron no responde con un "ok", lanzamos ek error
        raise RuntimeError(f"{verb} {d} -> {resp}")
    #POSE (añadido mínimo): actualizar pose tras OK ---
    try:
        pose = getattr(self, "pose", None)
        if pose is not None:
            pose.update_move(verb, d)
    except Exception:
        pass
    time.sleep(COOLDOWN_S)
    return True

#Atajos de los movimientos horizontales
def forward(self, dist_cm: int):
    return _move(self, "forward", dist_cm)

def back(self, dist_cm: int):
    return _move(self, "back", dist_cm)

def left(self, dist_cm: int):
    return _move(self, "left", dist_cm)

def right(self, dist_cm: int):
    return _move(self, "right", dist_cm)


#Movimiento hacia arriba
def up(self, dist_cm: int):

    self._require_connected()
    _ensure_techo(self)

    d = _distancia_acotada(dist_cm)

    # Si tenemos altura actual, calculamos si pasamos del techo y recortamos
    curr_h = getattr(self, "height_cm", None) # Lee la altura actual en cm si existe, o None si aún no hay datos de la misma de telemetría
    if isinstance(curr_h, int): #Comprueba si curr_h es entero
        max_cm = int(self.TECHO_M * 100) #Convertimos el techo de seguridad de metros a centímetros
        objetivo = curr_h + d #Se calcula hasta donde llegaría el dron al subir d cm desde la altura actual del dron
        if objetivo > max_cm:  #Si la distancia objetivo es mas alta que el techo
            d = max(0, max_cm - curr_h) #Se recorta d para que solo suba hasta el techo
            if d < MIN_STEP: #Si lo que falta es menos que 20 cm, no se manda nada (mínimo permitido)
                print(f"[INFO] up recortado a 0 (ya en techo ≈ {self.TECHO_M} m)")
                return True  # nada que subir
    if d == 0:
        return True

    resp = self._send(f"up {d}") #Se manda el comando al Tello
    if not _resp_is_ok(resp): #Si no devuelve "ok"
        raise RuntimeError(f"up {d} -> {resp}") #Lanza error
    # --- POSE (añadido mínimo): actualizar pose tras OK ---
    try:
        pose = getattr(self, "pose", None)
        if pose is not None:
            pose.update_move("up", d)
    except Exception:
        pass
    time.sleep(COOLDOWN_S)
    return True

def down(self, dist_cm: int): #Función para bajar

    self._require_connected()
    d = _distancia_acotada(dist_cm)

    curr_h = getattr(self, "height_cm", None) # Lee la altura actual en cm si existe, o None si aún no hay datos de la misma de telemetría
    if isinstance(curr_h, int):
        if d > curr_h: #Si la altura que se desea bajar es mayor que la altura actual (imposible)
            d = max(MIN_STEP, curr_h)  #Va a bajar  la altura actual o lo que pueda (MIN_STEP)
    resp = self._send(f"down {d}") #Se manda el comando al Tello
    if not _resp_is_ok(resp): #Si no devuelve "ok"
        raise RuntimeError(f"down {d} -> {resp}") #Lanza error
    # --- POSE (añadido mínimo): actualizar pose tras OK ---
    try:
        pose = getattr(self, "pose", None)
        if pose is not None:
            pose.update_move("down", d)
    except Exception:
        pass
    time.sleep(COOLDOWN_S)
    return True


def set_speed(self, speed_cm_s: int): #En vez de trabajar siempre con la velocidad por defecto, fijamos la velocidad.

    self._require_connected()
    try:
        v = int(speed_cm_s)
    except Exception: #Si se pasa un no entero, se pone el valor por defecto seguro
        v = 20
    v = max(MIN_SPEED, min(MAX_SPEED, v)) #Si supera por abajo o por arriba los límites, el valor se va a ajustar a estos
    resp = self._send(f"speed {v}")
    if str(resp).lower() != "ok":
        raise RuntimeError(f"speed {v} -> {resp}")

    time.sleep(COOLDOWN_S)
    return True


def rc(self, vx: int, vy: int, vz: int, yaw: int):
    # Limitar valores al rango válido del SDK Tello
    vx = max(-100, min(100, int(vx)))
    vy = max(-100, min(100, int(vy)))
    vz = max(-100, min(100, int(vz)))
    yaw = max(-100, min(100, int(yaw)))

    try:

        self._tello.send_rc_control(vx, vy, vz, yaw)
        return True
    except Exception as e:
        print(f"[rc] Error enviando comando: {e}")
        return False