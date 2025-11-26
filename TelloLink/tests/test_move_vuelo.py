from TelloLink.Tello import TelloDron
import time

def main():
    print("Test de vuelo indoor")
    dron = TelloDron()
    dron.connect()
    time.sleep(2)

    #Activamos la telemetría
    dron.startTelemetry(freq_hz=5)
    time.sleep(1)


    dron.TECHO_M = 2

    #Por seguridad, operamos con un mínimo de 20% de batería
    bat = dron.battery_pct if dron.battery_pct is not None else 100
    if bat < 20:
        print("Batería < 20%: abortando por seguridad.")
        dron.stopTelemetry(); dron.disconnect(); return

    #Ponemos la velocidad a 20 cm/s
    try:
        dron.set_speed(20)
    except Exception as e:
        print("Aviso: set_speed falló:", e)

    took_off = False
    try:

        print("\n--> Despegando a 1.0 m")
        ok = dron.takeOff(1.0, blocking=True)
        if not ok:
            print("No se pudo iniciar el despegue")
            return
        took_off = True


        time.sleep(1.0)
        wifi = dron.wifi if dron.wifi is not None else "N/A"
        temp = dron.temp_c if dron.temp_c is not None else "N/A"
        print(f"Altura={dron.height_cm} cm | Bat={dron.battery_pct}% | Temp={temp}°C | Wifi={wifi}")


        STEP = 20
        COOLDOWN = 0.6

        print("\n--> Movimientos horizontales muy cortos:")
        dron._move("forward", STEP);  time.sleep(COOLDOWN)
        dron._move("back",    STEP);  time.sleep(COOLDOWN)
        dron._move("left",    STEP);  time.sleep(COOLDOWN)
        dron._move("right",   STEP);  time.sleep(COOLDOWN)


        print("\n--> Subir/bajar 20 cm (respetando techo)")
        dron.up(STEP);    time.sleep(COOLDOWN)
        dron.down(STEP);  time.sleep(COOLDOWN)

        #Aterrizaje ---
        print("\n--> Aterrizando...")
        dron.Land(blocking=True)

    finally:
        #Aterriza aunque haya error
        if took_off and dron.state != "connected":
            try:
                dron.Land(blocking=True)
            except Exception:
                pass
        #Cierre de la telemetria limpio, para que no se quede "colgado"
        try: dron.stopTelemetry()
        except Exception: pass
        try: dron.disconnect()
        except Exception: pass

    print("\n Fin del test indoor ")

if __name__ == "__main__":
    main()