import sys
import os



from TelloLink import JoystickController
import time


def main():
    print("=" * 80)
    print("TEST DEL MÓDULO JOYSTICK (sin el dron)")
    print("=" * 80)
    print()

    # Crear controlador con la configuración de tu joystick
    print("Configuración:")
    print("  - Eje 0: Palanca IZQ horizontal → vx (left/right)")
    print("  - Eje 1: Palanca IZQ vertical → vy (forward/backward)")
    print("  - Eje 4: Palanca DER vertical → vz (up/down)")
    print("  - Eje 2: Palanca DER horizontal → yaw (rotación)")
    print("  - Deadzone: 0.1")
    print("  - Expo: 1.5")
    print()

    controller = JoystickController(
        axis_left_x=0,  # Palanca izq horizontal
        axis_left_y=1,  # Palanca izq vertical
        axis_right_x=2,  # Palanca der horizontal
        axis_right_y=4,  # Palanca der vertical
        invert_left_y=True,  # Invertir (arriba=-1 → +1)
        invert_right_y=True,  # Invertir (arriba=-1 → +1)
        deadzone=0.1,  # Zona muerta 10%
        expo=1.5  # Curva de respuesta
    )

    # Conectar
    print("Conectando al joystick...")
    if not controller.connect():
        print("\nNo se pudo conectar al joystick")
        print("   Verifica que esté conectado y encendido\n")
        return

    print()
    print("Joystick conectado correctamente")
    print()
    print("CONTROLES:")
    print("  Palanca IZQUIERDA:")
    print("    - Horizontal: vx (izquierda/derecha)")
    print("    - Vertical: vy (adelante/atrás)")
    print()
    print("  Palanca DERECHA:")
    print("    - Vertical: vz (arriba/abajo)")
    print("    - Horizontal: yaw (rotación)")
    print()
    print("LEYENDA:")
    print("  [L] = Izquierda   [R] = Derecha")
    print("  [F] = Forward     [B] = Backward")
    print("  [U] = Up          [D] = Down")
    print("  [↺] = Antihorario [↻] = Horario")
    print()
    print("Presiona Ctrl+C para salir")
    print()
    print("=" * 80)
    print("VALORES RC EN TIEMPO REAL:")
    print("=" * 80)

    try:
        while True:
            # Leer ejes y convertir a RC
            vx, vy, vz, yaw = controller.read_axes()

            # Indicadores visuales de dirección
            vx_dir = "L" if vx < -10 else "R" if vx > 10 else "-"
            vy_dir = "B" if vy < -10 else "F" if vy > 10 else "-"
            vz_dir = "D" if vz < -10 else "U" if vz > 10 else "-"
            yaw_dir = "↺" if yaw < -10 else "↻" if yaw > 10 else "-"

            # Formatear salida
            output = (f"vx:{vx:+4d} [{vx_dir}] | "
                      f"vy:{vy:+4d} [{vy_dir}] | "
                      f"vz:{vz:+4d} [{vz_dir}] | "
                      f"yaw:{yaw:+4d} [{yaw_dir}]")

            # Leer botones presionados (primeros 10)
            buttons = []
            num_buttons = controller.joystick.get_numbuttons()
            for i in range(min(10, num_buttons)):
                if controller.get_button(i):
                    buttons.append(f"B{i}")

            if buttons:
                output += f" | Botones: {','.join(buttons)}"

            # Mostrar (sobreescribe línea anterior)
            print(f"\r{output:<78}", end="", flush=True)

            time.sleep(0.05)  # 20 Hz (50ms)

    except KeyboardInterrupt:
        print("\n")
        print("=" * 80)
        print(" Test terminado")
        print("=" * 80)
        print()

    finally:
        # Desconectar joystick
        controller.disconnect()
        print("Joystick desconectado\n")


if __name__ == "__main__":
    main()