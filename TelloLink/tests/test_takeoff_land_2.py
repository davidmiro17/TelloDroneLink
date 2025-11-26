from TelloLink.Tello import TelloDron
import time


def main():
    print("Test de despegue y aterrizaje con telemetría \n")

    dron = TelloDron()

    try:
        # Conectar
        print("--> Conectando al Tello...")
        dron.connect()
        time.sleep(1)
        print(" Conectado")

        # Iniciar telemetría
        print("--> Iniciando telemetría...")
        dron.startTelemetry(freq_hz=5)
        time.sleep(1)  # Esperar a que arranque y haga las primeras lecturas
        print(" Telemetría activa\n")

        # Mostrar estado inicial
        bat_inicial = dron.battery_pct if dron.battery_pct else "N/A"
        temp_inicial = dron.temp_c if dron.temp_c else "N/A"
        wifi_inicial = dron.wifi if dron.wifi else "N/A"
        print(f"Estado inicial → Batería: {bat_inicial}% | Temp: {temp_inicial}°C | WiFi: {wifi_inicial}\n")

        # Despegar
        print("--> Despegando a 1.2 m...")
        ok = dron.takeOff(1.2, blocking=True)
        if not ok:
            print(" Error: No se pudo despegar")
            return
        print(" Despegue completado\n")

        # Lecturas de altura durante 5 segundos
        print("--> Lecturas de telemetría en vuelo (5 s):")
        altura_esperada = 120  #1.2 m

        for i in range(10):
            # Leer datos de telemetría
            h = dron.height_cm
            bat = dron.battery_pct if dron.battery_pct else "N/A"
            temp = dron.temp_c if dron.temp_c else "N/A"
            wifi = dron.wifi if dron.wifi else "N/A"

            # Calcular diferencia con altura esperada
            diff = abs(h - altura_esperada) if h else 0

            # Validar si está dentro de tolerancia (±20 cm)
            if diff < 20:
                status = "[OK]"
            else:
                status = "[NOT OK]"

            print(
                f"[{i + 1}/10] Altura: {h} cm {status} | Bat: {bat}% | Temp: {temp}°C | WiFi: {wifi} | Diff: {diff} cm")

            time.sleep(0.5)

        # Aterrizar
        print("\n--> Aterrizando...")
        dron.Land(blocking=True)
        print(" Aterrizaje completado")

        # Mostrar estado final
        print("\n--> Estado final:")
        bat_final = dron.battery_pct if dron.battery_pct else "N/A"
        temp_final = dron.temp_c if dron.temp_c else "N/A"
        tiempo_vuelo = dron.flight_time_s if dron.flight_time_s else "N/A"
        print(f"Batería: {bat_final}% | Temp: {temp_final}°C | Tiempo vuelo: {tiempo_vuelo}s\n")

        # Detener telemetría
        print("--> Deteniendo telemetría...")
        dron.stopTelemetry()
        print(" Telemetría detenida\n")

    except KeyboardInterrupt:
        print("\n Test interrumpido por usuario (Ctrl+C)")
        # Aterrizar de emergencia si está volando
        try:
            if dron.state == "flying":
                print("️ Aterrizaje de emergencia...")
                dron.Land(blocking=True)
        except Exception as e:
            print(f" Error en aterrizaje de emergencia: {e}")

    except Exception as e:
        print(f"\n Error en el test: {e}")
        import traceback
        traceback.print_exc()
        # Intentar aterrizar si está volando
        try:
            if dron.state == "flying":
                print("️ Aterrizaje de emergencia...")
                dron.Land(blocking=True)
        except Exception as e2:
            print(f" Error en aterrizaje de emergencia: {e2}")

    finally:

        try:
            print("\n--> Deteniendo telemetría...")
            dron.stopTelemetry()
        except Exception:
            pass

        print("--> Desconectando...")
        try:
            dron.disconnect()
            print(" Desconectado")
        except Exception as e:
            print(f" Error al desconectar: {e}")

    print("\n=== Test completado ===")


if __name__ == "__main__":
    main()