# Silencia todos los popups de messagebox (info/warning/error)
def _silence_all_popups():
    try:
        import tkinter.messagebox as _mb
        def _noop(*args, **kwargs):  # no-op que no bloquea
            # Si quieres ver qué se silencia, descomenta:
            # print("[popup silenciado]:", args)
            return None

        _mb.showinfo = _noop
        _mb.showwarning = _noop
        _mb.showerror = _noop
    except Exception:
        pass


_silence_all_popups()  # activar antes de cargar nada más

import tkinter as tk
from tkinter import messagebox
import threading
import time
import sys

from TelloLink.Tello import TelloDron

BAT_MIN_SAFE = 15  # % mínimo recomendado para volar en pruebas
DEFAULT_STEP = 20  # cm (mover)
DEFAULT_ANGLE = 30  # grados (giro)
DEFAULT_SPEED = 20  # cm/s


class MiniRemoteApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Demo Tello Pose")
        self.root.geometry("520x540")
        self.root.resizable(True, True)

        self.dron = TelloDron()

        # Variables interfaz gráfica
        self.step_var = tk.IntVar(value=DEFAULT_STEP)
        self.speed_var = tk.IntVar(value=DEFAULT_SPEED)
        self.angle_var = tk.IntVar(value=DEFAULT_ANGLE)

        self.state_var = tk.StringVar(value="disconnected")
        self.bat_var = tk.StringVar(value="—")
        self.h_var = tk.StringVar(value="0 cm")
        self.wifi_var = tk.StringVar(value="—")

        self._telemetry_running = False

        # --- NUEVO: bandera debounce para aterrizaje desde la UI ---
        self._ui_landing = False

        # --- POSE: variables de texto X/Y/Z/Yaw (añadido) ---
        self.x_var = tk.StringVar(value="X: —")
        self.y_var = tk.StringVar(value="Y: —")
        self.z_var = tk.StringVar(value="Z: —")
        self.yaw_var = tk.StringVar(value="Yaw: —")

        self._build_ui()
        self._bind_keys()

    # Código de la interfaz gráfica
    def _build_ui(self):
        pad = dict(padx=6, pady=6)

        # Contenedor con los datos de telemetría del dron
        top = tk.Frame(self.root)
        top.pack(fill="x", **pad)

        tk.Label(top, text="Estado:").grid(row=0, column=0, sticky="w")
        tk.Label(top, textvariable=self.state_var, width=12).grid(row=0, column=1, sticky="w")

        tk.Label(top, text="Batería:").grid(row=0, column=2, sticky="e")
        tk.Label(top, textvariable=self.bat_var, width=6).grid(row=0, column=3, sticky="w")

        tk.Label(top, text="Altura:").grid(row=0, column=4, sticky="e")
        tk.Label(top, textvariable=self.h_var, width=7).grid(row=0, column=5, sticky="w")

        tk.Label(top, text="WiFi:").grid(row=0, column=6, sticky="e")
        tk.Label(top, textvariable=self.wifi_var, width=6).grid(row=0, column=7, sticky="w")

        # Conexión
        conn = tk.Frame(self.root, bd=1, relief="groove")
        conn.pack(fill="x", **pad)

        tk.Button(conn, text="Conectar", width=12, command=self.on_connect, bg="#ffb347").grid(row=0, column=0, **pad)
        tk.Button(conn, text="Desconectar", width=12, command=self.on_disconnect).grid(row=0, column=1, **pad)
        tk.Button(conn, text="Salir", width=12, command=self.on_exit, bg="#ff6961").grid(row=0, column=2, **pad)

        # Parámetros ajustables
        params = tk.Frame(self.root, bd=1, relief="groove")
        params.pack(fill="x", **pad)

        tk.Label(params, text="Paso (cm):").grid(row=0, column=0, sticky="e")
        tk.Entry(params, textvariable=self.step_var, width=6).grid(row=0, column=1, sticky="w")

        tk.Label(params, text="Velocidad (cm/s):").grid(row=0, column=2, sticky="e")
        tk.Entry(params, textvariable=self.speed_var, width=6).grid(row=0, column=3, sticky="w")
        tk.Button(params, text="Aplicar", command=self.apply_speed).grid(row=0, column=4, padx=10)

        tk.Label(params, text="Ángulo (°):").grid(row=0, column=5, sticky="e")
        tk.Entry(params, textvariable=self.angle_var, width=6).grid(row=0, column=6, sticky="w")

        # Vuelo
        flight = tk.Frame(self.root, bd=1, relief="groove")
        flight.pack(fill="x", **pad)

        tk.Button(flight, text="Despegar (Enter)", width=16, command=self.on_takeoff, bg="#90ee90").grid(row=0,
                                                                                                         column=0,
                                                                                                         **pad)
        tk.Button(flight, text="Aterrizar (Espacio)", width=16, command=self.on_land, bg="#ff6961").grid(row=0,
                                                                                                         column=1,
                                                                                                         **pad)

        # Movimiento (disposición tipo mando)
        move = tk.Frame(self.root, bd=1, relief="groove")
        move.pack(pady=10)

        tk.Button(move, text="↑", width=6, command=lambda: self.do_move("forward")).grid(row=0, column=1, pady=4)
        tk.Button(move, text="←", width=6, command=lambda: self.do_move("left")).grid(row=1, column=0, padx=4)
        tk.Button(move, text="→", width=6, command=lambda: self.do_move("right")).grid(row=1, column=2, padx=4)
        tk.Button(move, text="↓", width=6, command=lambda: self.do_move("back")).grid(row=2, column=1, pady=4)

        tk.Button(move, text="Up", width=6, command=lambda: self.do_move("up")).grid(row=0, column=4, padx=12)
        tk.Button(move, text="Down", width=6, command=lambda: self.do_move("down")).grid(row=2, column=4)

        tk.Button(move, text="CCW (Q)", width=8, command=lambda: self.do_turn("ccw")).grid(row=1, column=5, padx=10)
        tk.Button(move, text="CW (E)", width=8, command=lambda: self.do_turn("cw")).grid(row=1, column=6)

        # Nota
        note = tk.Label(
            self.root,
            fg="#555",
            text=(
                "Consejos: prueba en un espacio amplio, sin personas u obstáculos.\n"
                "Mantén la batería ≥ 30%."
            )
        )
        note.pack(pady=4)

        # --- POSE: pequeño panel con X/Y/Z/Yaw (añadido) ---
        pose_panel = tk.Frame(self.root, bd=1, relief="groove")
        pose_panel.pack(fill="x", **pad)
        tk.Label(pose_panel, textvariable=self.x_var, width=10, anchor="w").grid(row=0, column=0, padx=4, pady=2,
                                                                                 sticky="w")
        tk.Label(pose_panel, textvariable=self.y_var, width=10, anchor="w").grid(row=0, column=1, padx=4, pady=2,
                                                                                 sticky="w")
        tk.Label(pose_panel, textvariable=self.z_var, width=10, anchor="w").grid(row=0, column=2, padx=4, pady=2,
                                                                                 sticky="w")
        tk.Label(pose_panel, textvariable=self.yaw_var, width=12, anchor="w").grid(row=0, column=3, padx=4, pady=2,
                                                                                   sticky="w")

    # Atajos de teclado de cada botón de la interfaz
    def _bind_keys(self):
        self.root.bind("<Up>", lambda e: self.do_move("forward"))
        self.root.bind("<Down>", lambda e: self.do_move("back"))
        self.root.bind("<Left>", lambda e: self.do_move("left"))
        self.root.bind("<Right>", lambda e: self.do_move("right"))
        self.root.bind("<Next>", lambda e: self.do_move("down"))  # PageDown
        self.root.bind("<Prior>", lambda e: self.do_move("up"))  # PageUp
        self.root.bind("<space>", lambda e: self.on_land())
        self.root.bind("<Return>", lambda e: self.on_takeoff())
        self.root.bind("<Key-q>", lambda e: self.do_turn("ccw"))
        self.root.bind("<Key-e>", lambda e: self.do_turn("cw"))

    # Acciones
    def on_connect(self):
        if self.dron.state == "connected":
            return
        try:
            self.dron.connect()
            self.state_var.set(self.dron.state)
            # Telemetría
            self.dron.startTelemetry(freq_hz=5)
            self._telemetry_running = True
            self._schedule_telemetry_pull()
            # --- KEEPALIVE: arrancar al conectar ---
            self._start_keepalive()
            # --- POSE: asegúrate de tener PoseVirtual creada y reseteada ---
            self._ensure_pose_origin()  # POSE
        except Exception as e:
            messagebox.showerror("Conectar", f"No se pudo conectar: {e}")

    def on_disconnect(self):
        self._stop_keepalive()
        try:
            self._telemetry_running = False
            try:
                self.dron.stopTelemetry()
            except Exception:
                pass
            self.dron.disconnect()
        except Exception as e:
            messagebox.showwarning("Desconectar", f"Aviso: {e}")
        finally:
            self.state_var.set("disconnected")
            self.bat_var.set("—")
            self.h_var.set("0 cm")
            self.wifi_var.set("—")
            # POSE: limpiar panel
            self.x_var.set("X: —")
            self.y_var.set("Y: —")
            self.z_var.set("Z: —")
            self.yaw_var.set("Yaw: —")

    def on_takeoff(self):
        if self.dron.state not in ("connected", "flying"):
            messagebox.showwarning("TakeOff", "Conecta primero.")
            return
        bat = getattr(self.dron, "battery_pct", None)
        if isinstance(bat, int) and bat < BAT_MIN_SAFE:
            if not messagebox.askokcancel("Batería baja", f"Batería {bat}%. ¿Despegar igualmente?"):
                return
        try:
            # --- pausa keepalive para no cruzar respuestas ---
            self._pause_keepalive()
            ok = self.dron.takeOff(0.5, blocking=True)  # altura de seguridad
            if not ok:
                messagebox.showerror("TakeOff", "No se pudo despegar.")
            else:
                # POSE: al despegar, si no existía, crea y resetea origen
                self._ensure_pose_origin()
        except Exception as e:
            messagebox.showerror("TakeOff", str(e))
        finally:
            self.root.after(200, self._resume_keepalive)

    def on_land(self):
        # debounce de UI para evitar múltiples lands en ráfaga
        if getattr(self, "_ui_landing", False):
            return
        self._ui_landing = True
        try:
            self._pause_keepalive()
            self.dron.Land(blocking=False)
        except Exception as e:
            messagebox.showerror("Land", str(e))
        finally:
            self.root.after(200, self._resume_keepalive)
            self.root.after(2000, lambda: setattr(self, "_ui_landing", False))

    def apply_speed(self):
        try:
            sp = int(self.speed_var.get())
            sp = max(10, min(100, sp))  # límites SDK
            self.dron.set_speed(sp)
            self.speed_var.set(sp)
        except Exception as e:
            messagebox.showerror("Velocidad", f"No se pudo aplicar velocidad: {e}")

    def do_move(self, cmd: str):
        if self.dron.state != "flying":
            messagebox.showinfo("Movimiento", "Despega primero.")
            return
        try:
            step = int(self.step_var.get())
            step = max(20, min(500, step))  # límites SDK
            self._pause_keepalive()
            getattr(self.dron, cmd)(step)  # forward/back/left/right/up/down
        except Exception as e:
            messagebox.showerror("Movimiento", str(e))
        finally:
            self.root.after(150, self._resume_keepalive)

    def do_turn(self, dir_: str):
        if self.dron.state != "flying":
            messagebox.showinfo("Giro", "Despega primero.")
            return
        try:
            ang = int(self.angle_var.get())
            ang = max(1, min(360, ang))
            self._pause_keepalive()
            if dir_ == "cw":
                self.dron.cw(ang)
            else:
                self.dron.ccw(ang)
            # POSE: si existe pose, actualiza yaw localmente para refresco inmediato
            try:
                pose = getattr(self.dron, "pose", None)
                if pose is not None:
                    if dir_ == "cw":
                        pose.yaw_deg = (float(getattr(pose, "yaw_deg", 0.0)) + float(ang)) % 360.0
                    else:
                        pose.yaw_deg = (float(getattr(pose, "yaw_deg", 0.0)) - float(ang)) % 360.0
            except Exception:
                pass
        except Exception as e:
            messagebox.showerror("Giro", str(e))
        finally:
            self.root.after(150, self._resume_keepalive)

    def on_exit(self):
        try:
            self._telemetry_running = False
            try:
                self.dron.stopTelemetry()
            except Exception:
                pass
            try:
                self._stop_keepalive()
            except Exception:
                pass
            if getattr(self.dron, "state", "") == "connected":
                try:
                    self.dron.disconnect()
                except Exception:
                    pass
        except Exception as e:
            messagebox.showwarning("Salir", f"Aviso: {e}")
        finally:
            self.root.destroy()

    # Telemetría
    def _schedule_telemetry_pull(self):
        if not self._telemetry_running:
            return
        try:
            self.state_var.set(getattr(self.dron, "state", "unknown"))
            bat = getattr(self.dron, "battery_pct", None)
            self.bat_var.set(f"{bat}%" if bat is not None else "—")
            h = getattr(self.dron, "height_cm", 0)
            self.h_var.set(f"{h} cm")
            wifi = getattr(self.dron, "wifi", None)
            self.wifi_var.set(str(wifi) if wifi is not None else "—")

            # --- POSE: leer pose y refrescar panel ---
            pose = getattr(self.dron, "pose", None)
            if pose is not None:
                try:
                    x = float(getattr(pose, "x_cm", 0.0) or 0.0)
                    y = float(getattr(pose, "y_cm", 0.0) or 0.0)
                    z_from_pose = getattr(pose, "z_cm", None)
                    z = float(z_from_pose if z_from_pose is not None else h)
                    self.x_var.set(f"X: {x:.0f} cm")
                    self.y_var.set(f"Y: {y:.0f} cm")
                    self.z_var.set(f"Z: {z:.0f} cm")

                    yaw = getattr(pose, "yaw_deg", None)
                    if yaw is not None:
                        try:
                            yaw_f = float(yaw)
                            self.yaw_var.set(f"Yaw: {yaw_f:.0f}°")
                        except Exception:
                            self.yaw_var.set("Yaw: —")
                    else:
                        self.yaw_var.set("Yaw: —")
                except Exception:
                    pass
            else:
                self.x_var.set("X: —")
                self.y_var.set("Y: —")
                self.z_var.set("Z: —")
                self.yaw_var.set("Yaw: —")

        except Exception:
            pass
        finally:
            self.root.after(200, self._schedule_telemetry_pull)

    # --- KEEPALIVE + PAUSA (para no cruzar respuestas) ---
    def _start_keepalive(self):
        if getattr(self, "_keepalive_running", False):
            return
        self._keepalive_running = True
        self._keepalive_paused = False  # NUEVO

        def _loop():
            while self._keepalive_running:
                try:
                    if self._keepalive_paused:
                        time.sleep(0.05)
                        continue
                    st = getattr(self.dron, "state", "")
                    if st == "flying":
                        self.dron._send("battery?")
                except Exception:
                    pass
                time.sleep(0.5)

        threading.Thread(target=_loop, daemon=True).start()

    def _stop_keepalive(self):
        self._keepalive_running = False

    # helpers de pausa/reanudación del keepalive
    def _pause_keepalive(self):
        self._keepalive_paused = True

    def _resume_keepalive(self):
        self._keepalive_paused = False

    # --- POSE: crea/rehinicializa PoseVirtual al origen si no existe (o al despegar) ---
    def _ensure_pose_origin(self):
        try:
            if getattr(self.dron, "pose", None) is None and hasattr(self.dron, "PoseVirtual"):
                # crea pose virtual anclada al punto actual (0,0,altura actual)
                self.dron.pose = self.dron.PoseVirtual()
                # inicializa Z con la altura barométrica actual
                try:
                    h = float(getattr(self.dron, "height_cm", 0) or 0)
                except Exception:
                    h = 0.0
                try:
                    self.dron.pose.x_cm = 0.0
                    self.dron.pose.y_cm = 0.0
                    self.dron.pose.z_cm = h
                    self.dron.pose.yaw_deg = float(getattr(self.dron.pose, "yaw_deg", 0.0) or 0.0)
                except Exception:
                    pass
        except Exception:
            pass


def main():
    root = tk.Tk()
    if sys.platform == "darwin":
        root.configure(bg="#f0f0f0")
        root.tk_setPalette(
            background="#f0f0f0",
            foreground="black",
            activeBackground="#ffb347",
            activeForeground="black"
        )

    app = MiniRemoteApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()