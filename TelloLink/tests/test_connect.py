from TelloLink.Tello import TelloDron


def main():
    print("Vamos a probar la conexión con el Tello...")

    # Creamos el objeto dron
    dron = TelloDron()

    # Intentamos conectar
    if dron.connect():
        print("Conexión establecida con el Tello")

        # Probamos un comando básico (preguntar batería)
        try:
            battery = dron._tello.get_battery()
            print(f"Nivel de batería: {battery}%")
        except Exception as Err:
            print("Error al pedir batería:", Err)

        # Desconectamos
        if dron.disconnect():
            print("Desconexión correcta")
        else:
            print("Fallo al desconectar")
    else:
        print("No se pudo conectar al Tello")

if __name__ == "__main__":
    main()