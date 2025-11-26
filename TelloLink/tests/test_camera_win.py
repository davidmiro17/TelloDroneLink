# tests/test_camera_cv2.py
import time
import os
from datetime import datetime

import cv2
from djitellopy import Tello

URL = "udp://0.0.0.0:11111?listen=1&fifo_size=50000000&overrun_nonfatal=1"

def main():
    print(" Test de cámara")

    tello = Tello()
    tello.connect()

    # Asegura estado limpio del stream
    try:
        tello.streamoff()
        time.sleep(0.3)
    except Exception:
        pass

    # Activa el stream
    tello.streamon()
    time.sleep(0.5)

    print(f"[INFO] Abriendo VideoCapture con FFMPEG: {URL}")
    cap = cv2.VideoCapture(URL, cv2.CAP_FFMPEG)

    if not cap.isOpened():
        print("[ERROR] OpenCV no pudo abrir el puerto UDP 11111.")
        print("Posibles causas: firewall de Windows bloqueando UDP 11111 para tu python.exe, "
              "no estás conectado al Wi-Fi del Tello, o falta soporte FFMPEG en OpenCV.")
        # Limpieza
        try:
            tello.streamoff()
        except Exception:
            pass
        tello.end()
        return

    print("[INFO] Esperando primer frame...")
    frame = None
    t0 = time.time()
    while time.time() - t0 < 6.0:   # espera hasta 6 segundos
        ok, frm = cap.read()
        if ok and frm is not None:
            frame = frm
            break
        # pequeña pausa para no quemar CPU
        cv2.waitKey(1)

    if frame is None:
        print("[ERROR] No llegaron frames. Revisa firewall (UDP 11111 IN/OUT) y Wi-Fi TELLO-xxxx.")
    else:
        # Guardar snapshot
        out_dir = os.path.join(".", "snapshots")
        os.makedirs(out_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(out_dir, f"tello_cv2_{ts}.jpg")
        cv2.imwrite(path, frame)
        print(f"✅ Snapshot guardado en: {path}")

        # Mostrar 3 segundos de vídeo (opcional)
        print("[INFO] Mostrando ventana 3s (pulsa 'q' para cerrar antes)…")
        t_show = time.time()
        while time.time() - t_show < 3.0:
            ok, frm = cap.read()
            if not ok or frm is None:
                break
            cv2.imshow("Tello FPV (OpenCV)", frm)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # Limpieza
    cap.release()
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass

    try:
        tello.streamoff()
    except Exception:
        pass
    tello.end()
    print("=== Fin del test ===")

if __name__ == "__main__":
    main()

