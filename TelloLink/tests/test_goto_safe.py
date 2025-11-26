from TelloLink.Tello import TelloDron
import time

def main():
    print("Test seguro goto")

    dron = TelloDron()
    dron.connect()
    time.sleep(1)

    # --- Seguridad ---
    print("\n[INFO] Iniciando telemetría...")
    dron.startTelemetry()
    time.sleep(1)

    bat = getattr(dron, "battery_pct", None)
    if bat is not None and isinstance(bat, int) and bat < 30:
        print(f"[ABORT] Batería baja ({bat}%). Carga antes de volar.")
        return

    #Por seguridad, establecemos un "volumen cúbico de seguridad", es decir máximo 200 cm en Z (altura) y 200 en X e Y (horizontal)
    MAX_RADIUS_CM = 200
    MAX_HEIGHT_CM = 200
    def soft_guard():
        try:
            if hasattr(dron, "pose") and dron.pose: #si existe pose y no es None
                if (abs(dron.pose.x_cm) > MAX_RADIUS_CM or #Si las coordenadas actuales del dron sobrepasan los límites establecidos
                    abs(dron.pose.y_cm) > MAX_RADIUS_CM or
                    dron.pose.z_cm > MAX_HEIGHT_CM):
                    print("Soft-box excedida → abortando goto.")  #Abortamos el test por razones de seguridad
                    dron.abort_goto()
                    return True
        except Exception:
            pass
        return False
    #Intentamos despegar, si el resultado no es ok, se aborta el test
    print("\nDespegando a 0.5 m")
    ok = dron.takeOff(0.5, blocking=True)
    if not ok:
        print("[ERROR] No se pudo despegar. Abortando test.")
        dron.stopTelemetry()
        dron.disconnect()
        return

    time.sleep(1.0)

    #Secuencia de pasos cortos seguros para el test
    sequence = [
        ("Forward 20 cm",  20,  0, 0),
        ("Back 20 cm",    -20,  0, 0),
        ("Right 20 cm",     0, 20, 0),
        ("Left 20 cm",      0,-20, 0),
        ("Up 20 cm",        0,  0, 20),
        ("Down 20 cm",      0,  0,-20)
    ]

    for desc, dx, dy, dz in sequence:
        if soft_guard():
            break
        print(f"\n--> {desc}")
        start_t = time.time()
        dron.goto_rel(dx_cm=dx, dy_cm=dy, dz_cm=dz, blocking=True)

        print(f"[OK] {desc} completado o timeout alcanzado.")

    #Finalización
    print("\n[INFO] Aterrizando...")
    try:
        dron.Land(blocking=True)
    except Exception as e:
        print(f"[WARN] Error en aterrizaje: {e}")

    dron.stopTelemetry()
    dron.disconnect()
    print("\n Test de goto_rel finalizado con seguridad.")

if __name__ == "__main__":
    main()