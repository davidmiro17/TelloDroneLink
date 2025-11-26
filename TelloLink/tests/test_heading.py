from TelloLink.Tello import TelloDron
import time

def main():
    print("Test de rotaciones (heading)")
    dron = TelloDron()
    dron.connect()

    took_off = False #El dron no ha despegado
    try:

        print("\n--> Despegando a 1.0 m") # Intenta despegar
        ok = dron.takeOff(1.0, blocking=True)
        if not ok:
            print("No se pudo iniciar el despegue")
            return
        took_off = True

        time.sleep(1.0)

        print("\n--> Giro horario 90°")
        dron.cw(90)
        time.sleep(1.0)

        print("\n--> Giro antihorario 90°")
        dron.ccw(90)
        time.sleep(1.0)


        print("\n--> Giro horario 360° (una vuelta completa)")
        dron.cw(360)
        time.sleep(1.0)

    finally:
        if took_off:  #Si  ha llegado a despegar, ahora va a aterrizar
            print("\n--> Aterrizando...")
            try:
                dron.Land(blocking=True)
            except Exception as e:
                print("Error al aterrizar:", e)

        dron.disconnect()
        print("Test completado")

if __name__ == "__main__":
    main()