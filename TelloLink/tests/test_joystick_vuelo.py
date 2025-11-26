import sys
import time
from TelloLink import TelloDron, JoystickController


def main():
    print("=" * 80)
    print("VUELO MANUAL DEL DRON CON JOYSTICK")
    print("=" * 80)
    print()

    # Configuración de botones
    BUTTON_TAKEOFF = 2
    BUTTON_LAND = 3

    print("CONTROLES:")
    print("  Palanca IZQUIERDA:")
    print("    - Horizontal: Izquierda/Derecha (vx)")
    print("    - Vertical: Adelante/Atrás (vy)")
    print()
    print("  Palanca DERECHA:")
    print("    - Vertical: Arriba/Abajo (vz)")
    print("    - Horizontal: Rotación (yaw)")
    print()
    print(f"  Botón {BUTTON_TAKEOFF}: DESPEGAR")
    print(f"  Botón {BUTTON_LAND}: ATERRIZAR")
    print()
    print("=" * 80)
    print()

    # Inicializar dron
    print("Conectando al dron...")
    dron = TelloDron()
    if not dron.connect():
        print("ERROR: No se pudo conectar al dron")
        return
    print("✓ Dron conectado")
    print()

    # Inicializar joystick
    print("Conectando al joystick...")
    controller = JoystickController(
        axis_left_x=0,
        axis_left_y=1,
        axis_right_x=2,
        axis_right_y=4,
        invert_left_y=True,
        invert_right_y=True,
        deadzone=0.1,
        expo=1.5
    )

    if not controller.connect():
        print("ERROR: No se pudo conectar al joystick")
        dron.disconnect()
        return
    print("✓ Joystick conectado")
    print()

    print("=" * 80)
    print(f"PRESIONA BOTÓN {BUTTON_TAKEOFF} PARA DESPEGAR")
    print("=" * 80)
    print()

    flying = False
    last_takeoff_pressed = False
    last_land_pressed = False

    try:
        while True:
            # Leer estado de botones
            takeoff_pressed = controller.get_button(BUTTON_TAKEOFF)
            land_pressed = controller.get_button(BUTTON_LAND)

            # DESPEGAR
            if takeoff_pressed and not last_takeoff_pressed and not flying:
                print("\n" + "=" * 80)
                print("DESPEGANDO...")
                print("=" * 80)
                dron.takeOff()
                flying = True
                print("✓ En vuelo - Control manual activo")
                print(f"  (Presiona botón {BUTTON_LAND} para aterrizar)")
                print()

            # ATERRIZAR
            if land_pressed and not last_land_pressed and flying:
                print("\n" + "=" * 80)
                print("ATERRIZANDO...")
                print("=" * 80)
                dron.Land()
                flying = False
                print("✓ Aterrizaje completado")
                print(f"  (Presiona botón {BUTTON_TAKEOFF} para despegar de nuevo)")
                print()

            # Actualizar estado previo de botones
            last_takeoff_pressed = takeoff_pressed
            last_land_pressed = land_pressed

            # CONTROL MANUAL (solo si está volando)
            if flying:
                vx, vy, vz, yaw = controller.read_axes()
                dron.rc(vx, vy, vz, yaw)

                # Mostrar valores actuales
                vx_dir = "L" if vx < -10 else "R" if vx > 10 else "-"
                vy_dir = "B" if vy < -10 else "F" if vy > 10 else "-"
                vz_dir = "D" if vz < -10 else "U" if vz > 10 else "-"
                yaw_dir = "↺" if yaw < -10 else "↻" if yaw > 10 else "-"

                output = (f"RC → vx:{vx:+4d}[{vx_dir}] "
                          f"vy:{vy:+4d}[{vy_dir}] "
                          f"vz:{vz:+4d}[{vz_dir}] "
                          f"yaw:{yaw:+4d}[{yaw_dir}]")

                print(f"\r{output:<78}", end="", flush=True)

            time.sleep(0.05)  # 20 Hz

    except KeyboardInterrupt:
        print("\n\n" + "=" * 80)
        print("CTRL+C detectado - Finalizando...")
        print("=" * 80)

        # Si está volando, aterrizar
        if flying:
            print("Aterrizando dron...")
            dron.Land()
            print("✓ Aterrizaje de emergencia completado")

    finally:
        # Asegurar RC en 0
        if flying:
            dron.rc(0, 0, 0, 0)

        # Desconectar
        print("\nDesconectando...")
        controller.disconnect()
        dron.disconnect()
        print("✓ Todo desconectado")
        print()
        print("=" * 80)
        print("Vuelo finalizado")
        print("=" * 80)
        print()


if __name__ == "__main__":
    main()