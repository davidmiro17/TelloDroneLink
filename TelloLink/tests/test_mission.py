import time
import threading
from TelloLink.Tello import TelloDron


MIN_BAT_PCT   = 30          # aborta si batería < 30%
MAX_RADIUS_CM = 120         # |x| y |y| no deben superar 1.2 m desde el origen
MAX_HEIGHT_CM = 120         # z no debe superar 1.2 m

def main():
    print(" Tello Mission Test ")
    dron = TelloDron()
    try:
        print("[INFO] Conectando...")
        dron.connect()
        time.sleep(1.0)

        # Telemetría (recomendado)
        dron.startTelemetry(freq_hz=5)
        time.sleep(0.5)

        # Batería mínima
        bat = getattr(dron, "battery_pct", None)
        if bat is not None and isinstance(bat, int) and bat < MIN_BAT_PCT:
            print(f"[ABORT] Batería baja ({bat}%). Carga antes de volar.")
            return

        # Soft-box: aborta misión si se sale del área
        def soft_guard():
            try:
                if hasattr(dron, "pose") and dron.pose:
                    if (abs(dron.pose.x_cm) > MAX_RADIUS_CM or
                        abs(dron.pose.y_cm) > MAX_RADIUS_CM or
                        dron.pose.z_cm > MAX_HEIGHT_CM):
                        print("[SAFE] Soft-box excedida → abortando misión.")
                        dron.abort_mission()
                        return True
            except Exception:
                pass
            return False

        # Waypoints RELATIVOS (pequeños).
        waypoints = [
            {"dx": 20, "dy":  0, "dz":  0, "delay": 0.5},  # adelante 20 cm
            {"dx":  0, "dy": 20, "dz":  0, "delay": 0.5},  # derecha  20 cm
            {"dx": -20, "dy":  0, "dz":  0, "delay": 0.5}, # atrás     20 cm
            {"dx":  0, "dy": -20, "dz":  0, "delay": 0.5}, # izquierda 20 cm
            {"dx":  0, "dy":   0, "dz": 20, "delay": 0.5}, # subir     20 cm
            {"dx":  0, "dy":   0, "dz":-20, "delay": 0.0}, # bajar     20 cm
        ]

        finished = threading.Event()

        def on_wp(idx, wp):
            print(f"[WP] Iniciando WP{idx}: {wp}")

        def on_finish():
            print("[INFO] Misión finalizada (o abortada).")
            finished.set()

        print("[INFO] Lanzando misión…")
        # Ejecuta la misión NO bloqueante
        dron.run_mission(
            waypoints=waypoints,
            do_land=True,         # aterriza al terminar
            blocking=False,       # no bloqueante
            on_wp=on_wp,
            on_finish=on_finish
        )


        t0 = time.time()
        HARD_TIMEOUT_S = 60
        while not finished.is_set():
            if soft_guard():
                break
            if time.time() - t0 > HARD_TIMEOUT_S:
                print("[SAFE] Timeout duro alcanzado → abortando misión.")
                dron.abort_mission()
                break
            time.sleep(0.05)


        finished.wait(timeout=10.0)

    except Exception as e:
        print(f"[ERROR] Test falló: {e}")

    finally:
        # Cierre limpio
        try:
            dron.stopTelemetry()
        except Exception:
            pass
        try:
            if getattr(dron, "state", "") == "flying":
                print("[SAFE] Aterrizando por seguridad…")
                dron.Land(blocking=True)
        except Exception:
            pass
        try:
            dron.disconnect()
        except Exception:
            pass
        print("=== Test terminado ===")

if __name__ == "__main__":
    main()