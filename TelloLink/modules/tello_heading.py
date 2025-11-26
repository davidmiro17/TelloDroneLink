import time

MIN_DEG = 1
STEP_MAX_DEG = 360      # El SDK solo acepta 1..360 por comando
COOLDOWN_S = 0.4        #pequeña pausa entre comandos por seguridad


def _magnitud_grados(deg):   #Esta función convierte cualquier ángulo en un ángulo positivo válido
    if deg is None:
        return 0  #Si no le pasamos nada, por seguridad, devuelve 0
    try:
        d = int(round(float(deg))) #Convertimos cualquier valor numérico en un entero redondeado en grados
    except Exception: #Si la conversión falla, se devuelve 0
        return 0
    d = abs(d) #El valor se queda en valor absoluto, el signo (horario o antihorario) se gestiona fuera (con cw o ccw)
    return d


def rotate(self, deg):
    self._require_connected() #Se confirma que el dron esté conectado

    if not deg: #Si no hay deg
        return True  #Se devuelove true, pero no hace nada
    verb = "cw" if deg > 0 else "ccw" #sentido horario si es positivo y antihorario si es negativo

    total = _magnitud_grados(deg) #Aquí normalizamos la magnitud total del giro, y se guarda en total
    if total < MIN_DEG:  #Si el ángulo es menor que el mínimo permitido
        return True  # nada que hacer

    restante = total   #Guardamos en restante la variable total
    while restante > 0: #Mientras quede algo por girar
        paso = restante if restante <= STEP_MAX_DEG else STEP_MAX_DEG #En la primera iteración, si hay que girar menos de 360 grados se gira lo que se haya pedido, si es superior, se gira 360 grados.
        resp = self._send(f"{verb} {paso}") #Se envía el comando al dron de los grados y el sentido horario de giro
        if str(resp).lower() != "ok": #Si el tello no devuelve ok
            raise RuntimeError(f"{verb} {paso} -> {resp}") #Se lanza error y el mensaje de este

        # >>> POSE: actualizar yaw si existe pose y el SDK aceptó el giro
        try:
            if hasattr(self, "pose") and self.pose is not None:
                # deg>0 = cw; deg<0 = ccw. 'paso' es magnitud positiva.
                delta = float(paso) if verb == "cw" else -float(paso)
                self.pose.update_yaw(delta)
        except Exception:
            pass
        # <<< POSE

        restante = restante - paso #Lo que queda pendiente por girar se actualiza para volver al bucle, y girar finalmente
        time.sleep(COOLDOWN_S)

    return True


def cw(self, deg):    #Estas dos funciones son útiles para los tests,  en vez de darles el angulo en negativo, le damos el angulo deseado y si lo queremos en horario o antihorario
    return rotate(self, abs(deg))

def ccw(self, deg):
    return rotate(self, -abs(deg))