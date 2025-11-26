import pygame
import sys
import time


def main():
    print("Ejes del Joystick\n")

    # Inicializar modulo pygame
    pygame.init()
    pygame.joystick.init()

    # Verificar joysticks
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No hay joysticks conectados.\n")
        return

    # Usar el primer joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"Usando: {joystick.get_name()}")
    print(f"Ejes disponibles: {joystick.get_numaxes()}")
    print(f"Botones disponibles: {joystick.get_numbuttons()}\n")

    print("INSTRUCCIONES:")
    print("- Mueve las palancas del joystick")
    print("- Observa qué eje cambia y en qué dirección")
    print("- Presiona Ctrl+C para salir\n")

    print("Valores en tiempo real:")
    print("-" * 80)

    try:
        while True:
            # Procesar eventos (necesario para actualizar valores)
            pygame.event.pump()

            # Leer todos los ejes
            axes_values = []
            for i in range(joystick.get_numaxes()):
                value = joystick.get_axis(i)
                axes_values.append(f"Eje {i}: {value:+.3f}")

            # Leer botones presionados
            buttons_pressed = []
            for i in range(joystick.get_numbuttons()):
                if joystick.get_button(i):
                    buttons_pressed.append(f"B{i}")

            # Mostrar en una línea (sobreescribe la anterior)
            output = " | ".join(axes_values)
            if buttons_pressed:
                output += f" | Botones: {', '.join(buttons_pressed)}"

            # Limpiar línea y mostrar
            print(f"\r{output:<78}", end="", flush=True)

            time.sleep(0.05)  # 20 Hz

    except KeyboardInterrupt:
        print("\n\n Exploración terminada\n")


if __name__ == "__main__":
    main()