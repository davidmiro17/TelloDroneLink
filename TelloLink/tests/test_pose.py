from TelloLink.modules.tello_pose import PoseVirtual
import time
import math

def main():
    print("Test completo del módulo tello_pose.py")

    # Crear la pose inicial
    pose = PoseVirtual()
    print("\n[1] Pose inicial (tras creación):")
    print(pose)

    # Reset
    pose.reset()
    print("\n[2] Tras reset():")
    print(pose)

    # Simular movimientos
    print("\n[3] Simulación de movimientos básicos (forward, right, up):")
    pose.update_move("forward", 100)
    pose.update_move("right", 50)
    pose.update_move("up", 30)
    print(pose)

    # Simular giro
    print("\n[4] Giro horario de 90°:")
    pose.update_yaw(90)
    print(pose)

    # Movimiento tras giro (debería afectar eje X/Y distinto)
    print("\n[5] Movimiento forward tras yaw=90°:")
    pose.update_move("forward", 100)
    print(pose)

    # Movimiento contrario
    print("\n[6] Movimiento back (50 cm):")
    pose.update_move("back", 50)
    print(pose)

    # Simulación de telemetría (altitud real + yaw real)
    print("\n[7] Actualización desde telemetría real (height=120, yaw=275):")
    pose.set_from_telemetry(height_cm=120, yaw_deg=275)
    print(pose)

    # Calcular distancia a otro punto
    print("\n[8] Cálculo de distancia a otra pose:")
    destino = PoseVirtual(x_cm=150, y_cm=150, z_cm=120)
    dist = pose.distance_to(destino)
    print(f"Pose actual: {pose}")
    print(f"Destino:     {destino}")
    print(f"Distancia:   {dist:.1f} cm")

    # Mostrar snapshot/to_dict
    print("\n[9] Snapshot de la pose actual:")
    snap = pose.to_dict() if hasattr(pose, "to_dict") else pose.capture()
    print(snap)

    # Test de ángulos límite
    print("\n[10] Prueba de normalización de yaw (wrap):")
    pose.update_yaw(200)
    print(f"Yaw tras +200° -> {pose.yaw_deg:.1f}°")
    pose.update_yaw(-600)
    print(f"Yaw tras -600° -> {pose.yaw_deg:.1f}°")

    print("\nTest del módulo tello_pose completado correctamente.")

if __name__ == "__main__":
    main()