from TelloLink.Tello import TelloDron
import time

def main():
    print(" Test de telemetría (básico, sin vuelo) ")
    dron = TelloDron()
    dron.connect()

    print("\n--> Iniciando telemetría durante 10s...")
    dron.startTelemetry(freq_hz=5)

    for i in range(10):
        wifi = dron.wifi if dron.wifi is not None else "N/A"
        flight_time = dron.flight_time_s if dron.flight_time_s is not None else "N/A"
        temp = dron.temp_c if dron.temp_c is not None else "N/A"
        yaw = getattr(dron, "yaw_deg", "N/A")

        pose_str = "N/A"
        if hasattr(dron, "pose") and dron.pose is not None:
            pose_str = str(dron.pose)  # usa __repr__: x,y,z,yaw

        print(
            f"[{i+1}] Altura={dron.height_cm} cm | "
            f"Battery={dron.battery_pct}% | "
            f"Temp={temp}°C | "
            f"Wifi={wifi} | "
            f"FlightTime={flight_time}s | "
            f"Yaw={yaw}° | "
            f"Pose={pose_str}"
        )
        time.sleep(1)

    print("\n--> Parando telemetría")
    dron.stopTelemetry()
    dron.disconnect()
    print("=== Test completado ===")

if __name__ == "__main__":
    main()