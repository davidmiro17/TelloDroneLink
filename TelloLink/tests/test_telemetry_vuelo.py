from TelloLink.Tello import TelloDron
import time

def main():
    print(" Test de telemetría con vuelo corto y seguro (indoor) ")
    dron = TelloDron()
    dron.connect()
    time.sleep(1.5)  # pequeña pausa

    # Telemetría
    dron.startTelemetry(freq_hz=5)
    time.sleep(1.0)  # primeras lecturas

    # Lectura inicial (con N/A donde falte)
    wifi = dron.wifi if dron.wifi is not None else "N/A"
    bat  = dron.battery_pct if dron.battery_pct is not None else "N/A"
    temp = dron.temp_c if dron.temp_c is not None else "N/A"
    yaw  = getattr(dron, "yaw_deg", "N/A")
    pose_str = str(dron.pose) if hasattr(dron, "pose") and dron.pose is not None else "N/A"
    print(f"Inicial -> Altura={dron.height_cm} cm | Battery={bat}% | Temp={temp}°C | Wifi={wifi} | FlightTime={dron.flight_time_s}s | Yaw={yaw}° | Pose={pose_str}")

    #Batería mínima (20%)
    if isinstance(dron.battery_pct, int) and dron.battery_pct < 20:
        print("Batería baja (<20%). Abortando prueba por seguridad.")
        dron.stopTelemetry()
        dron.disconnect()
        return

    target_m = 1.0
    print(f"\n--> Despegando a {target_m} m (indoor, seguro)")
    took_off = False

    try:
        ok = dron.takeOff(target_m, blocking=True)
        if not ok:
            print("No se pudo iniciar el despegue (estado no válido).")
            return
        took_off = True

        # Observa 5 s en aire
        print("\n--> Telemetría en aire (5 s):")
        t0 = time.time()
        while time.time() - t0 < 5.0:
            wifi = dron.wifi if dron.wifi is not None else "N/A"
            flight_time = dron.flight_time_s if dron.flight_time_s is not None else "N/A"
            temp = dron.temp_c if dron.temp_c is not None else "N/A"
            yaw  = getattr(dron, "yaw_deg", "N/A")
            pose_str = str(dron.pose) if hasattr(dron, "pose") and dron.pose is not None else "N/A"

            print(
                f"Altura={dron.height_cm} cm | "
                f"Battery={dron.battery_pct}% | "
                f"Temp={temp}°C | "
                f"Wifi={wifi} | "
                f"FlightTime={flight_time}s | "
                f"Yaw={yaw}° | "
                f"Pose={pose_str}"
            )
            time.sleep(0.5)

        print("\n--> Aterrizando...")
        dron.Land(blocking=True)

    finally:
        # Si algo falló, intenta aterrizar igualmente
        if took_off and getattr(dron, "state", "") != "connected":
            try:
                dron.Land(blocking=True)
            except Exception:
                pass
        # Cierre limpio
        try:
            dron.stopTelemetry()
        except Exception:
            pass
        try:
            dron.disconnect()
        except Exception:
            pass

    print("\n=== Test completado ===")

if __name__ == "__main__":
    main()