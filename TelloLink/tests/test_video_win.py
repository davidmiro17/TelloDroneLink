from TelloLink.Tello import TelloDron
import cv2
import time

UDP_URLS = [
    # URL estándar
    "udp://0.0.0.0:11111",
    # Variantes que a veces ayudan en Windows/FFmpeg:
    "udp://0.0.0.0:11111?overrun_nonfatal=1&fifo_size=50000000",
]

def open_udp_capture():
    """Intenta abrir el stream con varias URLs UDP hasta que una funcione."""
    for url in UDP_URLS:
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if cap.isOpened():
            return cap, url
        cap.release()
    return None, None

def main():
    print("Test de vídeo FPV (OpenCV, Windows)")
    dron = TelloDron()
    dron.connect()

    # Asegura estado de cámara en el Tello
    print("\n--> Preparando stream del Tello")
    try:
        dron._tello.streamoff()
        time.sleep(0.3)
    except Exception:
        pass

    try:
        dron._tello.streamon()
        time.sleep(0.8)  # pequeña espera para que empiece a mandar UDP
    except Exception as e:
        print(f"[ERROR] streamon falló: {e}")
        dron.disconnect()
        return

    print("\n--> Abriendo captura OpenCV en udp://0.0.0.0:11111")
    cap, used_url = open_udp_capture()
    if cap is None:
        print("[ERROR] OpenCV no pudo abrir el puerto UDP 11111.")
        print("• Verifica firewall de Windows (permitir Python/FFmpeg en UDP 11111).")
        print("• Asegúrate de estar conectado a la Wi-Fi del Tello.")
        try:
            dron._tello.streamoff()
        except Exception:
            pass
        dron.disconnect()
        return

    print(f"[OK] Captura abierta con: {used_url}")

    window_name = "Tello FPV (OpenCV)"
    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    except Exception:
        pass

    print("\n--> Mostrando vídeo durante 10 s (pulsa 'q' para salir antes)...")
    t0 = time.time()
    while time.time() - t0 < 10.0:
        ok, frame = cap.read()
        if not ok or frame is None:
            # En los primeros ms puede no llegar frame aún
            if time.time() - t0 < 2.0:
                time.sleep(0.02)
                continue
            print("[WARN] No se reciben frames. ¿Firewall bloqueando UDP 11111?")
            break

        # Mostrar frame
        try:
            cv2.imshow(window_name, frame)
        except Exception:
            pass

        # Salir con 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print("\n--> Cerrando vídeo y stream...")
    try:
        cap.release()
    except Exception:
        pass
    try:
        cv2.destroyWindow(window_name)
    except Exception:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    try:
        dron._tello.streamoff()
    except Exception:
        pass

    dron.disconnect()
    print(" Test completado")

if __name__ == "__main__":
    main()