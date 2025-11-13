# Silencia todos los popups de messagebox (info/warning/error)
def _silence_all_popups():
    try:
        import tkinter.messagebox as _mb
        def _noop(*args, **kwargs):
            return None

        _mb.showinfo = _noop
        _mb.showwarning = _noop
        _mb.showerror = _noop
    except Exception:
        pass


_silence_all_popups()

import tkinter as tk
from tkinter import messagebox, filedialog, ttk
import threading
import time
import sys
import math
import json
import os
import datetime

# FPV / Imagen
import cv2
from PIL import Image, ImageTk
import numpy as np

try:
    from shapely.geometry import Point, Polygon
except ImportError:
    print("ADVERTENCIA: shapely no está instalado. Instala con: pip install shapely")
    Point = None
    Polygon = None

from TelloLink.Tello import TelloDron
from TelloLink import JoystickController

BAT_MIN_SAFE = 20
DEFAULT_STEP = 20
DEFAULT_ANGLE = 30
DEFAULT_SPEED = 20

# Constantes del minimapa geofence
MAP_SIZE_PX = 900
MAP_BG_COLOR = "#fafafa"
MAP_AXES_COLOR = "#cccccc"
MAP_DRONE_COLOR = "#1f77b4"
MAP_TARGET_COLOR = "#d62728"
PX_PER_CM = 2.5
GRID_STEP_CM = 20
CENTER_MARK_COLOR = "#666"


def _ts():
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


class MiniRemoteApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Demo Tello Geofence + FPV + Joystick")
        self.root.geometry("740x650")
        self.root.resizable(True, True)
        self.root.configure(bg="#f0f0f0")

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
        self._ui_landing = False

        # POSE
        self.x_var = tk.StringVar(value="X: —")
        self.y_var = tk.StringVar(value="Y: —")
        self.z_var = tk.StringVar(value="Z: —")
        self.yaw_var = tk.StringVar(value="Yaw: —")

        # Geofence
        self.gf_max_x_var = tk.StringVar(value="0")
        self.gf_max_y_var = tk.StringVar(value="0")
        self.gf_zmin_var = tk.StringVar(value="0")
        self.gf_zmax_var = tk.StringVar(value="120")
        self.gf_mode_var = tk.StringVar(value="soft")

        # Mapa
        self._map_win = None
        self.map_canvas = None
        self._map_static_drawn = False
        self._map_drone_item = None
        self._last_pose_key = None
        self._gf_center_item = None
        self._tool_var = None
        self._circle_radius_var = None
        self._poly_points = []
        self._incl_pts = []
        self._incl_rect = None
        self._incl_zmin_var = None
        self._incl_zmax_var = None
        self._excl_circles = []
        self._excl_polys = []
        self._excl_zmin_var = None
        self._excl_zmax_var = None

        # FPV
        self.fpv_label = None
        self._fpv_running = False
        self._fpv_thread = None
        self._frame_lock = threading.Lock()
        self._last_bgr = None
        self._want_stream_on = False
        self._rec_running = False
        self._rec_writer = None
        self._rec_path = None
        self._rec_fps = 30
        self._rec_size = (640, 480)
        self._shots_dir = os.path.abspath("captures")
        self._recs_dir = os.path.abspath("videos")
        os.makedirs(self._shots_dir, exist_ok=True)
        os.makedirs(self._recs_dir, exist_ok=True)
        self._rec_badge_var = tk.StringVar(value="")
        self._hud_msg = None
        self._hud_until = 0.0
        self._cv_cap = None

        # Joystick
        self._joy_thread = None
        self._joy_running = False
        self._joy_max_rc = 100  #  Velocidad al 100% (sin slider)
        self._joy_speed_var = tk.IntVar(value=self._joy_max_rc)
        self._joy_label_var = tk.StringVar(value="Joystick: —")
        self._joy_evt_var = tk.StringVar(value="")

        # Configuración de botones del joystick (FIJA)
        self._joy_bindings = {
            'photo': 0,
            'rec': 1,
            'takeoff': 2,
            'land': 3
        }

        # Keepalive
        self._keepalive_running = False
        self._keepalive_paused = False
        self._last_rc_sent = 0  #  NUEVO

        # Para guardar última posición de aterrizaje
        self._last_land_x = 0.0
        self._last_land_y = 0.0
        self._last_land_z = 0.0
        self._last_land_yaw = 0.0

        self._build_ui()
        self._bind_keys()
        self._init_joystick()

    def _hud_show(self, msg: str, duration_sec: float = 1.5):
        try:
            self._hud_msg = str(msg)
            self._hud_until = time.time() + float(duration_sec)
        except Exception:
            self._hud_msg = None
            self._hud_until = 0.0

    def _draw_overlays(self, canvas: np.ndarray):
        h, w = canvas.shape[:2]
        try:
            if self._rec_running:
                cv2.circle(canvas, (18, 18), 8, (0, 0, 255), thickness=-1)
                cv2.putText(canvas, "REC", (34, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
        except Exception:
            pass
        try:
            now = time.time()
            if self._hud_msg and now < self._hud_until:
                txt = self._hud_msg
                (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                pad = 10
                box_w = tw + pad * 2
                box_h = th + pad * 2
                x0 = max(6, (w - box_w) // 2)
                y0 = max(6, h - box_h - 10)
                x1 = min(w - 6, x0 + box_w)
                y1 = min(h - 6, y0 + box_h)
                overlay = canvas.copy()
                cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 0, 0), thickness=-1)
                alpha = 0.6
                cv2.addWeighted(overlay, alpha, canvas, 1 - alpha, 0, dst=canvas)
                cv2.rectangle(canvas, (x0, y0), (x1, y1), (255, 255, 255), thickness=1)
                tx = x0 + pad
                ty = y0 + pad + th
                cv2.putText(canvas, txt, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            elif self._hud_msg and now >= self._hud_until:
                self._hud_msg = None
        except Exception:
            pass

    def _build_ui(self):
        pad = dict(padx=6, pady=6)

        # Telemetría
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

        # Vuelo + REC badge
        flight = tk.Frame(self.root, bd=1, relief="groove")
        flight.pack(fill="x", **pad)
        tk.Button(flight, text="Despegar (Enter)", width=16, command=self.on_takeoff, bg="#90ee90").grid(row=0,
                                                                                                         column=0,
                                                                                                         **pad)
        tk.Button(flight, text="Aterrizar (Espacio)", width=16, command=self.on_land, bg="#ff6961").grid(row=0,
                                                                                                         column=1,
                                                                                                         **pad)
        tk.Label(flight, textvariable=self._rec_badge_var, fg="#d00", font=("Arial", 10, "bold")).grid(row=0, column=2,
                                                                                                       sticky="w",
                                                                                                       padx=10)

        # FPV MÁS GRANDE (480x360)
        fpv_frame = tk.Frame(self.root, bd=2, relief="groove", bg="black")
        fpv_frame.pack(fill="both", expand=False, padx=8, pady=8)

        self._fpv_w, self._fpv_h = 480, 360
        fpv_host = tk.Frame(fpv_frame, width=self._fpv_w, height=self._fpv_h, bg="black")
        fpv_host.pack(padx=4, pady=4)
        fpv_host.pack_propagate(False)

        self.fpv_label = tk.Label(fpv_host, text="FPV (detenido)", bg="black", fg="white", anchor="center",
                                  font=("Arial", 10))
        self.fpv_label.pack(fill="both", expand=True)

        # Controles FPV
        fpv_controls = tk.Frame(fpv_frame, bg="black")
        fpv_controls.pack(fill="x", padx=4, pady=(0, 4))

        tk.Button(fpv_controls, text="▶ FPV", command=self.start_fpv, bg="#cde7cd", width=8).pack(side="left", padx=2)
        tk.Button(fpv_controls, text="⏹", command=self.stop_fpv, width=4).pack(side="left", padx=2)
        tk.Button(fpv_controls, text="📷", command=self.take_snapshot, width=4).pack(side="left", padx=2)
        tk.Button(fpv_controls, text="🔴", command=self.toggle_recording, width=4).pack(side="left", padx=2)

        tk.Label(fpv_controls, textvariable=self._joy_label_var, fg="white", bg="black", font=("Arial", 8)).pack(
            side="right", padx=4)

        # Geofence compacto
        gf = tk.Frame(self.root, bd=1, relief="groove")
        gf.pack(fill="x", **pad)
        tk.Label(gf, text="Geofence:").grid(row=0, column=0, sticky="e")
        tk.Label(gf, text="ancho X (cm)").grid(row=0, column=1, sticky="e")
        tk.Entry(gf, textvariable=self.gf_max_x_var, width=6).grid(row=0, column=2)
        tk.Label(gf, text="ancho Y (cm)").grid(row=0, column=3, sticky="e")
        tk.Entry(gf, textvariable=self.gf_max_y_var, width=6).grid(row=0, column=4)
        tk.Label(gf, text="Z min (cm)").grid(row=0, column=5, sticky="e")
        tk.Entry(gf, textvariable=self.gf_zmin_var, width=6).grid(row=0, column=6)
        tk.Label(gf, text="Z max (cm)").grid(row=0, column=7, sticky="e")
        tk.Entry(gf, textvariable=self.gf_zmax_var, width=6).grid(row=0, column=8)
        tk.Label(gf, text="Modo:").grid(row=0, column=9, sticky="e")
        tk.Radiobutton(gf, text="Soft", variable=self.gf_mode_var, value="soft").grid(row=0, column=10)
        tk.Radiobutton(gf, text="Hard", variable=self.gf_mode_var, value="hard").grid(row=0, column=11)
        tk.Button(gf, text="Activar", command=self.on_gf_activate, bg="#cde7cd").grid(row=1, column=1, padx=4, pady=4,
                                                                                      sticky="w")
        tk.Button(gf, text="Desactivar", command=self.on_gf_disable).grid(row=1, column=2, padx=4, pady=4, sticky="w")
        tk.Button(gf, text="Abrir mapa", command=self.open_map_window, bg="#87CEEB").grid(row=1, column=4, padx=8,
                                                                                          pady=4, sticky="w")

        # Nota compacta
        note = tk.Label(self.root, fg="#555", font=("Arial", 8),
                        text="Joystick: IZQ=XY | DER=altura+yaw | B0=Foto | B1=Video | B2=Despegar | B3=Aterrizar")
        note.pack(pady=2)

        # POSE panel compacto
        pose_panel = tk.Frame(self.root, bd=1, relief="groove")
        pose_panel.pack(fill="x", **pad)
        tk.Label(pose_panel, textvariable=self.x_var, width=10, anchor="w", font=("Arial", 9)).grid(row=0, column=0,
                                                                                                    padx=4, pady=2,
                                                                                                    sticky="w")
        tk.Label(pose_panel, textvariable=self.y_var, width=10, anchor="w", font=("Arial", 9)).grid(row=0, column=1,
                                                                                                    padx=4, pady=2,
                                                                                                    sticky="w")
        tk.Label(pose_panel, textvariable=self.z_var, width=10, anchor="w", font=("Arial", 9)).grid(row=0, column=2,
                                                                                                    padx=4, pady=2,
                                                                                                    sticky="w")
        tk.Label(pose_panel, textvariable=self.yaw_var, width=12, anchor="w", font=("Arial", 9)).grid(row=0, column=3,
                                                                                                      padx=4, pady=2,
                                                                                                      sticky="w")

    def _bind_keys(self):
        self.root.bind("<Up>", lambda e: self.do_move("forward"))
        self.root.bind("<Down>", lambda e: self.do_move("back"))
        self.root.bind("<Left>", lambda e: self.do_move("left"))
        self.root.bind("<Right>", lambda e: self.do_move("right"))
        self.root.bind("<Next>", lambda e: self.do_move("down"))
        self.root.bind("<Prior>", lambda e: self.do_move("up"))
        self.root.bind("<space>", lambda e: self.on_land())
        self.root.bind("<Return>", lambda e: self.on_takeoff())
        self.root.bind("<Key-q>", lambda e: self.do_turn("ccw"))
        self.root.bind("<Key-e>", lambda e: self.do_turn("cw"))

    # ===================== Conexión / Telemetría =====================
    def on_connect(self):
        if self.dron.state == "connected":
            return
        try:
            self.dron.connect()
            self.state_var.set(self.dron.state)
            self._ensure_pose_origin()
            self.dron.startTelemetry(freq_hz=5)
            self._telemetry_running = True
            self._schedule_telemetry_pull()
            self._start_keepalive()
        except Exception as e:
            messagebox.showerror("Conectar", f"No se pudo conectar: {e}")

    def on_disconnect(self):
        self._stop_keepalive()
        self.stop_fpv()
        self._stop_recording()
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
            self.x_var.set("X: —")
            self.y_var.set("Y: —")
            self.z_var.set("Z: —")
            self.yaw_var.set("Yaw: —")
            try:
                if self._map_win and tk.Toplevel.winfo_exists(self._map_win):
                    self._map_win.destroy()
            except Exception:
                pass
            self._map_win = None
            self.map_canvas = None

    def on_takeoff(self):
        if self.dron.state not in ("connected", "flying"):
            messagebox.showwarning("TakeOff", "Conecta primero.")
            return
        bat = getattr(self.dron, "battery_pct", None)
        if isinstance(bat, int) and bat < BAT_MIN_SAFE:
            if not messagebox.askokcancel("Batería baja", f"Batería {bat}%. ¿Despegar?"):
                return
        try:
            self._pause_keepalive()
            ok = self.dron.takeOff(0.5, blocking=False)
            if not ok:
                messagebox.showerror("TakeOff", "No se pudo despegar.")
            else:
                self._ensure_pose_origin()
                self._restart_gf_monitor(force=True)
                try:
                    self._reapply_exclusions_to_backend()
                except Exception:
                    pass
                self._hud_show("✈ Despegado", 1.5)
        except Exception as e:
            messagebox.showerror("TakeOff", str(e))
        finally:
            self.root.after(200, self._resume_keepalive)

    def on_land(self):
        if getattr(self, "_ui_landing", False):
            return
        self._ui_landing = True
        try:
            st = getattr(self.dron, "state", "")
            try:
                h = float(getattr(self.dron, "height_cm", 0) or 0)
            except Exception:
                h = 0.0
            if h <= 20:
                return
            if st not in ("flying", "hovering", "takingoff", "landing"):
                return
            self._pause_keepalive()
            self.dron.Land(blocking=False)
            self._hud_show(" Aterrizando", 1.5)
        except Exception as e:
            messagebox.showerror("Land", str(e))
        finally:
            self.root.after(150, self._resume_keepalive)
            self.root.after(4500, lambda: setattr(self, "_ui_landing", False))

    def on_exit(self):
        try:
            self._stop_joystick()
            self.stop_fpv()
            self._stop_recording()
            self.on_disconnect()
        finally:
            try:
                self.root.destroy()
            except Exception:
                pass

    def apply_speed(self):
        try:
            sp = int(self.speed_var.get())
            sp = max(10, min(100, sp))
            self.dron.set_speed(sp)
            self.speed_var.set(sp)
        except Exception as e:
            messagebox.showerror("Velocidad", f"No se pudo aplicar: {e}")

    def do_move(self, cmd: str):
        if self.dron.state != "flying":
            return
        try:
            step = int(self.step_var.get())
            step = max(20, min(500, step))
            self._pause_keepalive()
            try:
                if cmd == "forward":
                    self.dron.forward(step)
                elif cmd == "back":
                    self.dron.back(step)
                elif cmd == "left":
                    self.dron.left(step)
                elif cmd == "right":
                    self.dron.right(step)
                elif cmd == "up":
                    self.dron.up(step)
                elif cmd == "down":
                    self.dron.down(step)
            except Exception as e:
                print(f"[move] Error: {e}")
            finally:
                self.root.after(150, self._resume_keepalive)
        except Exception:
            pass

    def do_turn(self, direction: str):
        if self.dron.state != "flying":
            return
        try:
            angle = int(self.angle_var.get())
            angle = max(1, min(360, angle))
            self._pause_keepalive()
            if direction == "cw":
                self.dron.cw(angle)
            else:
                self.dron.ccw(angle)
        except Exception:
            pass
        finally:
            self.root.after(150, self._resume_keepalive)

    # ===================== KEEPALIVE =====================
    def _start_keepalive(self):
        if getattr(self, "_keepalive_running", False):
            return
        self._keepalive_running = True
        self._keepalive_paused = False

        def _loop():
            while self._keepalive_running:
                try:
                    if self._keepalive_paused:
                        time.sleep(0.05)
                        continue

                    st = getattr(self.dron, "state", "")
                    if st == "flying":
                        #  No enviar battery si se usó RC recientemente
                        last_rc_time = getattr(self, "_last_rc_sent", 0)
                        if time.time() - last_rc_time > 2.0:
                            try:
                                self.dron._send("battery?")
                            except Exception:
                                pass
                except Exception:
                    pass
                time.sleep(1.0)  #  Reducido a 1 segundo

        threading.Thread(target=_loop, daemon=True).start()

    def _stop_keepalive(self):
        self._keepalive_running = False

    def _pause_keepalive(self):
        self._keepalive_paused = True

    def _resume_keepalive(self):
        self._keepalive_paused = False

    # ===================== TELEMETRÍA =====================
    def _schedule_telemetry_pull(self):
        if not self._telemetry_running:
            return
        self._pull_telemetry()
        self.root.after(200, self._schedule_telemetry_pull)

    def _pull_telemetry(self):
        try:
            bat = getattr(self.dron, "battery_pct", None)
            h = getattr(self.dron, "height_cm", None)
            snr = getattr(self.dron, "wifi_snr", None)
            st = getattr(self.dron, "state", "disconnected")
            self.state_var.set(st)
            self.bat_var.set(f"{bat}%" if isinstance(bat, int) else "—")
            self.h_var.set(f"{h} cm" if isinstance(h, (int, float)) else "0 cm")
            self.wifi_var.set(f"{snr}" if isinstance(snr, int) else "—")

            pose = getattr(self.dron, "pose", None)
            if pose is not None and h is not None and isinstance(h, (int, float)):
                pose.z_cm = float(h)

            if pose:
                x = getattr(pose, "x_cm", None)
                y = getattr(pose, "y_cm", None)
                z = getattr(pose, "z_cm", None)
                yaw = getattr(pose, "yaw_deg", None)
                self.x_var.set(f"X: {x:.1f} cm" if x is not None else "X: —")
                self.y_var.set(f"Y: {y:.1f} cm" if y is not None else "Y: —")
                self.z_var.set(f"Z: {z:.1f} cm" if z is not None else "Z: —")
                self.yaw_var.set(f"Yaw: {yaw:.1f}°" if yaw is not None else "Yaw: —")
                if x is not None and y is not None:
                    try:
                        self._last_land_x = float(x)
                        self._last_land_y = float(y)
                        self._last_land_z = float(z) if z is not None else 0.0
                        self._last_land_yaw = float(yaw) if yaw is not None else 0.0
                    except Exception:
                        pass
            else:
                self.x_var.set("X: —")
                self.y_var.set("Y: —")
                self.z_var.set("Z: —")
                self.yaw_var.set("Yaw: —")
            self._update_map_drone()
        except Exception as e:
            print(f"[telemetry] Error: {e}")

    def _ensure_pose_origin(self):
        try:
            if not hasattr(self.dron, "pose") or self.dron.pose is None:
                if hasattr(self.dron, "set_pose_origin"):
                    self.dron.set_pose_origin()
        except Exception:
            pass

    # ===================== GEOFENCE =====================
    def on_gf_activate(self):
        try:
            max_x = float(self.gf_max_x_var.get() or 0.0)
            max_y = float(self.gf_max_y_var.get() or 0.0)
            zmin = float(self.gf_zmin_var.get() or 0.0)
            zmax = float(self.gf_zmax_var.get() or 120.0)
            mode = self.gf_mode_var.get()
            if max_x <= 0 or max_y <= 0:
                messagebox.showwarning("Geofence", "Define ancho X/Y > 0.")
                return
            pose = getattr(self.dron, "pose", None)
            cx = float(getattr(pose, "x_cm", 0.0) or 0.0) if pose else 0.0
            cy = float(getattr(pose, "y_cm", 0.0) or 0.0) if pose else 0.0
            if hasattr(self.dron, "set_geofence"):
                self.dron.set_geofence(max_x_cm=max_x, max_y_cm=max_y, max_z_cm=zmax, z_min_cm=zmin, mode=mode)
            else:
                setattr(self.dron, "_gf_enabled", True)
                setattr(self.dron, "_gf_center", (cx, cy))
                setattr(self.dron, "_gf_limits", {"max_x": max_x, "max_y": max_y, "zmin": zmin, "zmax": zmax})
                setattr(self.dron, "_gf_mode", mode)
            self._reapply_exclusions_to_backend()
            messagebox.showinfo("Geofence", f"Activado ({mode}).")
            if self._map_win and tk.Toplevel.winfo_exists(self._map_win):
                self._redraw_map_static()
        except Exception as e:
            messagebox.showerror("Geofence", f"Error: {e}")

    def on_gf_disable(self):
        try:
            if hasattr(self.dron, "set_geofence"):
                self.dron.set_geofence(enabled=False)
            else:
                setattr(self.dron, "_gf_enabled", False)
            messagebox.showinfo("Geofence", "Desactivado.")
        except Exception as e:
            messagebox.showerror("Geofence", f"Error: {e}")

    def _restart_gf_monitor(self, force=False):
        try:
            if hasattr(self.dron, "_stop_geofence_monitor"):
                self.dron._stop_geofence_monitor()
            if hasattr(self.dron, "_start_geofence_monitor"):
                self.dron._start_geofence_monitor()
        except Exception:
            pass

    def _reapply_exclusions_to_backend(self):
        try:
            if not hasattr(self.dron, "_gf_excl_circles"):
                self.dron._gf_excl_circles = []
            if not hasattr(self.dron, "_gf_excl_polys"):
                self.dron._gf_excl_polys = []
            self.dron._gf_excl_circles.clear()
            self.dron._gf_excl_polys.clear()
            for c in self._excl_circles:
                self.dron._gf_excl_circles.append(c)
            for p in self._excl_polys:
                self.dron._gf_excl_polys.append(p)
        except Exception:
            pass

    # ===================== FPV =====================
    def start_fpv(self):
        if self._fpv_running:
            return
        try:
            self._want_stream_on = True
            if hasattr(self.dron, "startVideo"):
                self.dron.startVideo()
            else:
                try:
                    self.dron._send("streamon")
                except Exception:
                    pass
        except Exception:
            pass
        try:
            if self._cv_cap is None or not self._cv_cap.isOpened():
                self._cv_cap = cv2.VideoCapture("udp://0.0.0.0:11111", cv2.CAP_FFMPEG)
        except Exception:
            pass
        self._fpv_running = True
        self._fpv_thread = threading.Thread(target=self._fpv_loop, daemon=True)
        self._fpv_thread.start()
        self._hud_show(" FPV iniciado", 1.0)

    def stop_fpv(self):
        self._fpv_running = False
        self._want_stream_on = False
        time.sleep(0.05)
        try:
            if hasattr(self.dron, "stopVideo"):
                self.dron.stopVideo()
            else:
                try:
                    self.dron._send("streamoff")
                except Exception:
                    pass
        except Exception:
            pass
        try:
            if self._cv_cap is not None:
                self._cv_cap.release()
        except Exception:
            pass
        finally:
            self._cv_cap = None
        self._hud_show(" FPV detenido", 1.0)

    def _read_frame_generic(self):
        try:
            if self._cv_cap is not None and self._cv_cap.isOpened():
                ok, frame = self._cv_cap.read()
                if ok and isinstance(frame, np.ndarray) and frame.size > 0:
                    return frame
        except Exception:
            pass
        return None

    def _fpv_loop(self):
        last_badge_toggle = 0.0
        rec_on = False
        try:
            while self._fpv_running:
                frame_bgr = self._read_frame_generic()
                if frame_bgr is None:
                    self._set_fpv_text("(esperando…)")
                    time.sleep(0.04)
                    continue
                with self._frame_lock:
                    self._last_bgr = frame_bgr
                h, w = frame_bgr.shape[:2]
                target_w, target_h = self._fpv_w, self._fpv_h
                scale = min(target_w / max(1, w), target_h / max(1, h))
                new_w, new_h = max(1, int(w * scale)), max(1, int(h * scale))
                bgr_resized = cv2.resize(frame_bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
                off_x = (target_w - new_w) // 2
                off_y = (target_h - new_h) // 2
                canvas_base = np.zeros((target_h, target_w, 3), dtype=np.uint8)
                canvas_base[off_y:off_y + new_h, off_x:off_x + new_w] = bgr_resized
                canvas_preview = canvas_base.copy()
                self._draw_overlays(canvas_preview)
                rgb = cv2.cvtColor(canvas_preview, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(rgb)
                imgtk = ImageTk.PhotoImage(image=img)
                self.fpv_label.configure(image=imgtk, text="")
                self.fpv_label.image = imgtk
                if self._rec_running:
                    if self._rec_writer is None:
                        self._start_writer((target_w, target_h))
                    try:
                        self._rec_writer.write(canvas_base)
                    except Exception:
                        pass
                    now = time.time()
                    if now - last_badge_toggle > 0.5:
                        rec_on = not rec_on
                        self._rec_badge_var.set("[REC]" if rec_on else "REC")
                        last_badge_toggle = now
                else:
                    self._rec_badge_var.set("")
                time.sleep(0.01)
            self._rec_badge_var.set("")
        except Exception:
            pass

    def _set_fpv_text(self, text):
        self.fpv_label.configure(text=text, image="")
        self.fpv_label.image = None

    def take_snapshot(self):
        with self._frame_lock:
            frame = None if self._last_bgr is None else self._last_bgr.copy()
        if frame is None:
            self._hud_show(" Sin frame", 1.5)
            return
        path = os.path.join(self._shots_dir, f"shot_{_ts()}.png")
        try:
            cv2.imwrite(path, frame)
            self._hud_show(f" Guardada", 1.8)
        except Exception:
            self._hud_show(" Error", 1.5)

    def toggle_recording(self):
        if self._rec_running:
            self._stop_recording()
        else:
            self._start_recording()

    def _start_writer(self, size_wh):
        self._rec_size = size_wh
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self._rec_writer = cv2.VideoWriter(self._rec_path, fourcc, self._rec_fps, self._rec_size)

    def _start_recording(self):
        self._rec_path = os.path.join(self._recs_dir, f"rec_{_ts()}.mp4")
        self._rec_running = True
        self._rec_writer = None
        self._hud_show(" Grabando", 1.5)

    def _stop_recording(self):
        if self._rec_running:
            self._rec_running = False
            try:
                if self._rec_writer is not None:
                    self._rec_writer.release()
            except Exception:
                pass
            self._rec_writer = None
            self._hud_show(" Guardado", 1.5)

    # ===================== JOYSTICK =====================
    def _init_joystick(self):
        """Inicializa el joystick usando JoystickController EXACTAMENTE como el test."""
        if self._joy_running:
            return

        self._joy_running = True

        def _joy_thread_fn():
            # Crear controlador IGUAL que el test
            controller = JoystickController(
                axis_left_x=0,
                axis_left_y=1,
                axis_right_x=2,
                axis_right_y=4,
                invert_left_y=True,
                invert_right_y=True,
                deadzone=0.1,
                expo=1.5
            )

            if not controller.connect():
                print("[joy] No se pudo conectar al joystick")
                def _ui():
                    self._joy_label_var.set(" —")
                self.root.after(0, _ui)
                return

            joy_name = controller.joystick.get_name()
            def _ui_name():
                self._joy_label_var.set(f" {joy_name}")
            self.root.after(0, _ui_name)

            print(f"[Joystick] Conectado: {joy_name}")

            last_takeoff = False
            last_land = False
            last_photo = False
            last_rec = False

            while self._joy_running:
                try:
                    # LEER EJES - IGUAL que el test
                    vx, vy, vz, yaw = controller.read_axes()

                    # Enviar RC - IGUAL que el test
                    def _send():
                        if self.dron.state == "flying":
                            try:
                                self.dron.rc(int(vx), int(vy), int(vz), int(yaw))
                                self._last_rc_sent = time.time()
                            except Exception:
                                pass
                    self.root.after(0, _send)

                    # BOTONES
                    takeoff_pressed = controller.get_button(2)
                    land_pressed = controller.get_button(3)
                    photo_pressed = controller.get_button(0)
                    rec_pressed = controller.get_button(1)

                    if takeoff_pressed and not last_takeoff:
                        def _t():
                            print("[joy] Botón 2 - Despegar")
                            self.on_takeoff()
                        self.root.after(0, _t)
                    last_takeoff = takeoff_pressed

                    if land_pressed and not last_land:
                        def _l():
                            print("[joy] Botón 3 - Aterrizar")
                            self.on_land()
                        self.root.after(0, _l)
                    last_land = land_pressed

                    if photo_pressed and not last_photo:
                        def _p():
                            print("[joy] Botón 0 - Foto")
                            self.take_snapshot()
                        self.root.after(0, _p)
                    last_photo = photo_pressed

                    if rec_pressed and not last_rec:
                        def _r():
                            print("[joy] Botón 1 - Grabar/Parar")
                            self.toggle_recording()
                        self.root.after(0, _r)
                    last_rec = rec_pressed

                    time.sleep(0.05)

                except Exception as e:
                    print(f"[joy] Error: {e}")
                    time.sleep(0.1)

            # Cleanup
            try:
                controller.disconnect()
            except Exception:
                pass
            print("[joy] Thread finalizado")

        self._joy_thread = threading.Thread(target=_joy_thread_fn, daemon=True)
        self._joy_thread.start()

    def _stop_joystick(self):
        """Detiene el joystick."""
        self._joy_running = False

    # ===================== MAPA =====================
    def open_map_window(self):
        messagebox.showinfo("Mapa", "Ventana de mapa con exclusiones/inclusiones disponible.")

    def _update_map_drone(self):
        pass

    def _redraw_map_static(self):
        pass


# ===================== MAIN =====================
if __name__ == "__main__":
    root = tk.Tk()
    app = MiniRemoteApp(root)
    root.mainloop()
