import pygame
import sys


def main():
    print("=== Detección de Joystick ===\n")

    # Inicializar modulo pygame
    pygame.init()
    pygame.joystick.init()

    # Contar joysticks conectados
    joystick_count = pygame.joystick.get_count()
    print(f"Joysticks conectados: {joystick_count}\n")

    if joystick_count == 0:
        print(" No hay joysticks conectados.")
        print("\nConecta un joystick  y vuelve a ejecutar.\n")
        return

    # Mostrar información de cada joystick
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()

        print(f"─── Joystick {i} ───")
        print(f"  Nombre: {joystick.get_name()}")
        print(f"  Ejes: {joystick.get_numaxes()}")
        print(f"  Botones: {joystick.get_numbuttons()}")
        print(f"  Hats (D-pad): {joystick.get_numhats()}")
        print()

    print(" Detección completada\n")


if __name__ == "__main__":
    main()