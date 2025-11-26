from TelloLink.Tello import TelloDron


def main():
    print("Probando desconexión del Tello...")

    dron = TelloDron()

    # Intentamos desconectar sin haber conectado antes. Como la función está diseñada para ello, al desconectar sin haber conectado antes no dará error.
    if dron.disconnect():
        print("Desconexión ejecutada (aunque no había conexión previa)")
        print("Estado actual:", dron.state)       # debería ser "disconnected"
        print("Objeto _tello:", dron._tello)      # debería ser None
    else:
        print("Algo raro al desconectar")

if __name__ == "__main__":
    main()