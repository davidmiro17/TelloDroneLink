from __future__ import annotations
import math
import threading
import time
from typing import Optional, Callable, Any
from TelloLink.modules.tello_move import MIN_STEP

#Parámetros ajustables
_MIN_BAT_PCT   = 20        #Batería mínima para realizar la operación
_STEP_XY_CM    = 25.0      #Paso horizontal
_STEP_Z_CM     = 20.0      #Paso vertical
_TOL_XY_CM     = 8         #Tolerancia horizontal
_TOL_Z_CM      = 8         #Tolerancia vertical
_SLEEP_S       = 0.10      #Pausa entre comandos
_MAX_RETRY_CMD = 2         #Reintentos de cada paso si falla

#Función que decide el tamaño de cada paso a realizar según lo que le queda por recorrer.
def _adaptive_step(rest: float, base: float, min_step: float = MIN_STEP) -> float:
    rest = abs(rest)  # Distancia restante en valor absoluto
    if rest <= min_step: #Si la distancia restante es mas pequeña que el mínimo paso permitido en Tello
        return rest #Devuelve lo que falta
    #Escala el paso según la distancia que queda: lejos = paso base, cerca = paso reducido. Si rest / (3.0 * base) es mayor que 1, aún le falta mucho y coge 1 que es el paso base
    scale = max(0.35, min(1.0, rest / (3.0 * base)))
    # Paso final, siempre >= min_step. Si el paso que hemos escalado es menor que el mínimo de Tello, daría error, por eso eligiriamos el mínimo de tello
    return max(min_step, base * scale)

#Función que hace girar al dron hasta el ángulo deseado
def _rotate_to_yaw(self, target_yaw_deg: float) -> bool:
    curr = float(getattr(self, "yaw_deg", 0.0) or 0.0) #Busca la orientación actual del dron, si no existe, le pone un 0
    delta = (float(target_yaw_deg) - curr) % 360.0 #Calcula cuanto tiene que girar para llegar al ángulo objetivo
    if delta == 0: #Si ya se está en la orientación, no hace nada
        return True
    if delta > 180.0: #Si lo que hay que girar es mayor que 180
        amt = int(round(360.0 - delta)) #Calcula el ángulo equivalente en sentido antihorario
        resp = self.ccw(amt) #Gira
    else: #Si en cambio es menor
        amt = int(round(delta))  #Calcula el ángulo en sentido horario
        resp = self.cw(amt) #Gira
    return bool(str(resp).lower() == "ok" or resp is True)



#Esta función envía un mmovimiento al dron, espera la confirmación y actualiza la pose virtual si tuvo éxito
def _send_and_update(self, cmd: str, dist_cm: float) -> bool:
    dist_i = int(round(dist_cm))
    if dist_i <= 0: #Si la distancia del paso a realizar es 0, devuelve True
        return True
    for _ in range(_MAX_RETRY_CMD + 1): #Hacemos un bucle con el numero de "vueltas" (intentos) que son el inicial + el número de reintentos
        resp = getattr(self, cmd)(dist_i) #Ejecuta el movimiento
        ok = bool(str(resp).lower() == "ok" or resp is True) #Comprueba si el dron confirmó el movimiento
        if ok: #Si se ejecutó correctamente
            try:
                if hasattr(self, "pose") and self.pose: #Comprueba que haya el atributo pose exista
                    self.pose.update_move(cmd, dist_i) #Si existe, actualiza las coordenadas
            except Exception:
                pass
            return True
        time.sleep(0.05)
    return False


def _goto_rel_worker(self,
                     dx_cm: float, dy_cm: float, dz_cm: float = 0.0,
                     yaw_deg: Optional[float] = None,
                     speed_cm_s: Optional[float] = None,
                     callback: Optional[Callable[..., Any]] = None,
                     params: Any = None) -> None:
    #Chequeos básicos previos
    if not hasattr(self, "pose") or self.pose is None: #Si la pose no existe, aborta
        print("[goto] No hay PoseVirtual; abortando.")
        return
    if getattr(self, "state", "") == "disconnected": #Si está desconectado, aborta
        print("[goto] Dron desconectado; abortando.")
        return
    bat = getattr(self, "battery_pct", None) #Si la batería es inferior al umbral definido anteriormente (20%), aborta
    if isinstance(bat, int) and bat < _MIN_BAT_PCT:
        print(f"[goto] Batería baja ({bat}%), abortando.")
        return

    #Si el dron no está volando, hace un despegue seguro a 0,5 metros
    if getattr(self, "state", "") != "flying":
        ok = self.takeOff(0.5, blocking=True)
        if not ok:
            print("[goto] No se pudo despegar.") #Si falla el despegue, aborta
            return
        time.sleep(0.4)

    #Si la velocidad no se ha pasado en la función, intenta fijar la velocidad del SDK de Tello
    if speed_cm_s is not None:
        try:
            self.set_speed(int(speed_cm_s))
        except Exception:
            pass

    #Llama a _rotate_to_yaw para orientar el dron al ángulo deseado, si el giro falla, aborta
    if yaw_deg is not None:
        if not _rotate_to_yaw(self, float(yaw_deg)):
            print("[goto] Error en giro inicial.")
            return
        time.sleep(0.05)

    # Objetivo de cada coordenada (suma la posición actual del dron con el desplazamiento deseado que se le manda al dron)
    x_goal = self.pose.x_cm + float(dx_cm)
    y_goal = self.pose.y_cm + float(dy_cm)
    z_goal = self.pose.z_cm + float(dz_cm)

    #Esta función calcula lo que falta para llegar al objetivo en cada eje y en el plano. Finalmente se calcula la distancia directa al objetivo (hipotenusa)
    def remaining():
        rx = x_goal - self.pose.x_cm
        ry = y_goal - self.pose.y_cm
        rz = z_goal - self.pose.z_cm
        rxy = math.hypot(rx, ry)
        return rx, ry, rz, rxy

    # Si ya estamos dentro de tolerancia, nada que hacer
    _, _, rz0, rxy0 = remaining()
    if (abs(rz0) <= _TOL_Z_CM) and (rxy0 <= _TOL_XY_CM):
        if callback:
            try: callback(params)
            except TypeError:
                try: callback()
                except Exception: pass
        return

    # Bucle hasta llegar al objetivo, o que haya algún error debido a motivos de seguridad
    while True:
        # posibilidad de aborto externo
        if getattr(self, "_goto_abort", False): #Busca si el dron tiene el atributo _goto_abort, si no existe vale False, si existe aborta
            print("[goto] Abortado por solicitud externa.")
            return
        #Verificación de seguridad por batería baja
        bat = getattr(self, "battery_pct", None) #Obtiene el valor actual de la batería
        if isinstance(bat, int) and bat < _MIN_BAT_PCT: #Si el nivel está por debajo del mínim, aborta
            print(f"[goto] Abortado por batería ({bat}%).")
            return

        rx, ry, rz, rxy = remaining() #Actualiza la distancia que falta en tiempo real

        #Si consideramos que ya ha llegado (dentro de las tolerancias) se hace el break, y sale del bucle
        if (abs(rz) <= _TOL_Z_CM) and (rxy <= _TOL_XY_CM):
            break

        #Corregir primero la altura
        if abs(rz) > _TOL_Z_CM: #si aún hay que subir o bajar
            stepz = _adaptive_step(rz, _STEP_Z_CM, min_step=20.0) #realiza el paso
            cmd = "up" if rz > 0 else "down"
            if not _send_and_update(self, cmd, stepz): #se envía el paso al dron y actualiza la pose, si falla se muestra el mensaje
                print("[goto] Micro-paso Z fallido.")
            time.sleep(_SLEEP_S)
            continue

        #Este bloque convierte lo que falta en el mapa a cuanto falta avanzar/retroceder/izquierda/derecha según hacia donde mira el dron (yaw)
        yaw = math.radians(getattr(self.pose, "yaw_deg", 0.0) or 0.0) #lee el yaw actual y lo convierte a radianes
        fx, fy = math.cos(yaw), math.sin(yaw)  # eje forward (mundo)
        rx_, ry_ = math.sin(yaw), -math.cos(yaw)  # eje right = forward rotado -90° (CW)

        f_comp = rx * fx + ry * fy     #componente frontal
        r_comp = rx * rx_ + ry * ry_   #componente horizontal

        moved = False

        if abs(f_comp) > (_TOL_XY_CM * 0.4): #Si lo que falta por avanzar o retroceder es mayor que el 40% de la tolerancia
            stepx = _adaptive_step(f_comp, _STEP_XY_CM, min_step=15.0) #Se realiza el paso con su función
            cmd = "forward" if f_comp > 0 else "back" #decide si va hacia delante o detrás
            if not _send_and_update(self, cmd, stepx): #se manda el comando al dron y se actualiza la pose
                print("[goto] Micro-paso forward/back fallido.")
            moved = True
        #Se realiza lo mismo pero para derecha o izquierda
        if abs(r_comp) > (_TOL_XY_CM * 0.4):
            stepy = _adaptive_step(r_comp, _STEP_XY_CM, min_step=15.0)
            cmd = "right" if r_comp > 0 else "left"
            if not _send_and_update(self, cmd, stepy):
                print("[goto] Micro-paso right/left fallido.")
            moved = True

        # si no hay nada significativo que mover, evita bucle vacío
        if not moved and rxy <= _TOL_XY_CM:
            break

        time.sleep(_SLEEP_S)

    print("[goto] Objetivo alcanzado.")
    if callback:
        try: callback(params)
        except TypeError:
            try: callback()
            except Exception: pass

#Funcion pública
def goto_rel(self,
             dx_cm: float, dy_cm: float, dz_cm: float = 0.0,
             yaw_deg: Optional[float] = None,
             speed_cm_s: Optional[float] = None,
             blocking: bool = True,
             callback: Optional[Callable[..., Any]] = None,
             params: Any = None) -> None:
    setattr(self, "_goto_abort", False)

    t = threading.Thread(
        target=_goto_rel_worker,
        args=(self, dx_cm, dy_cm, dz_cm, yaw_deg, speed_cm_s, callback, params),
        daemon=True
    )
    t.start()
    if blocking:
        t.join()

def abort_goto(self) -> None:
    setattr(self, "_goto_abort", True)