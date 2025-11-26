from TelloLink.Tello import TelloDron
import time

def main():
    print("Test de despegue y aterrizaje con lecturas de altura")

    dron = TelloDron()
    dron.connect()

    print("\n--> Despegando a 1.2 m ...")
    dron.takeOff(1.2, blocking=True)

    #Durante 5 segundos, cada 0,5 segundos se van a hacer lecturas de la altura
    print("\n--> Lecturas de altura en vuelo (5 s):")
    for _ in range(10):   # 10 lecturas cada 0.5s
        try:
            h = dron._tello.get_height()  # usa la liobreria djitellopy directamente
            print("Altura real:", h, "cm")
        except Exception:
            print("Altura real: N/A")
        time.sleep(0.5)

    print("\n--> Aterrizando...")
    dron.Land(blocking=True)

    print("\n=== Test completado ===")

if __name__ == "__main__":
    main()