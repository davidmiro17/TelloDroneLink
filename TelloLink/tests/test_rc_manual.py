from TelloLink.Tello import TelloDron
import time


def main():
    print("Test de control RC manual (sin joystick)\n")

    dron = TelloDron()

    try:
        # Conexión
        print("--> Conectando al Tello...")
        dron.connect()
        time.sleep(1)
        print("Conectado\n")

        # Telemetría
        print("--> Iniciando telemetría...")
        dron.startTelemetry(freq_hz=5)
        time.sleep(1)
        print(" Telemetría activa\n")

        # Configurar velocidad
        dron.set_speed(30)  # Velocidad baja para indoor seguro
        print(" Velocidad configurada: 30 cm/s\n")

        # Despegue
        print("--> Despegando a 1.0 m...")
        ok = dron.takeOff(1.0, blocking=True)
        if not ok:
            print(" No se pudo despegar")
            return
        print("Despegue exitoso\n")

        time.sleep(1)


        print("--> Test 1: Adelante suave (30%) durante 2s")
        for i in range(20):  # 20 * 0.1s = 2s
            dron.rc(0, 30, 0, 0)
            time.sleep(0.1)

        # Hover
        print("    Hover (detener movimiento)")
        dron.rc(0, 0, 0, 0)
        time.sleep(1)


        print("--> Test 2: Atrás suave (30%) durante 2s")
        for i in range(20):
            dron.rc(0, -30, 0, 0)
            time.sleep(0.1)

        # Hover
        print("    Hover (detener movimiento)")
        dron.rc(0, 0, 0, 0)
        time.sleep(1)

        # TEST 3: Izquierda
        print("--> Test 3: Izquierda (30%) durante 1.5s")
        for i in range(30):
            dron.rc(-30, 0, 0, 0)
            time.sleep(0.1)

        # Hover
        print("    Hover")
        dron.rc(0, 0, 0, 0)
        time.sleep(1)

        # Derecha (volver)
        print("--> Test 4: Derecha (30%) durante 1.5s")
        for i in range(30):
            dron.rc(30, 0, 0, 0)
            time.sleep(0.1)

        # Hover
        print("    Hover")
        dron.rc(0, 0, 0, 0)
        time.sleep(1)

        # Rotación horaria
        print("--> Test 5: Rotación horaria (30%) durante 2s")
        for i in range(20):
            dron.rc(0, 0, 0, 30)
            time.sleep(0.1)

        # Hover
        print("    Hover")
        dron.rc(0, 0, 0, 0)
        time.sleep(1)

        #  Rotación antihoraria (volver)
        print("--> Test 6: Rotación antihoraria (30%) durante 2s")
        for i in range(20):
            dron.rc(0, 0, 0, -30)
            time.sleep(0.1)

        # Hover final
        print("    Hover final")
        dron.rc(0, 0, 0, 0)
        time.sleep(1)

        # Mostrar estado final
        bat = dron.battery_pct if dron.battery_pct else "N/A"
        altura = dron.height_cm
        print(f"\nEstado final → Altura: {altura} cm | Batería: {bat}%\n")

        # Aterrizaje
        print("--> Aterrizando...")
        dron.Land(blocking=True)
        print(" Aterrizaje completado\n")

    except KeyboardInterrupt:
        print("\n️ Test interrumpido por usuario (Ctrl+C)")
        try:
            if dron.state == "flying":
                print("️ Aterrizaje de emergencia...")
                dron.Land(blocking=True)
        except Exception:
            pass

    except Exception as e:
        print(f"\n Error en el test: {e}")
        import traceback
        traceback.print_exc()
        try:
            if dron.state == "flying":
                print(" Aterrizaje de emergencia...")
                dron.Land(blocking=True)
        except Exception:
            pass

    finally:
        # Cleanup
        try:
            dron.stopTelemetry()
        except Exception:
            pass

        print("--> Desconectando...")
        try:
            dron.disconnect()
            print(" Desconectado")
        except Exception:
            pass

    print("\n=== Test completado ===")


if __name__ == "__main__":
    main()