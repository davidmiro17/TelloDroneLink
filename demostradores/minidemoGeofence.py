#Silencia todos los popups de messagebox (info/warning/error)
def _silence_all_popups():
    try:
        import tkinter.messagebox as _mb
        def _noop(*args, **kwargs):  # no-op que no bloquea
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
import math
import json
import traceback
from tkinter import filedialog

try:
    from shapely.geometry import Point, Polygon
except ImportError:
    print("ADVERTENCIA: shapely no está instalado. Instala con: pip install shapely")
    Point = None
    Polygon = None

from TelloLink.Tello import TelloDron

BAT_MIN_SAFE = 20  # ✅ Subido de 15% a 20%
DEFAULT_STEP = 20  # cm (mover)
DEFAULT_ANGLE = 30  # grados (giro)
DEFAULT_SPEED = 20  # cm/s

# ===================== NUEVO: constantes del minimapa geofence =====================
MAP_SIZE_PX = 900  # ancho del canvas
MAP_BG_COLOR = "#fafafa"
MAP_AXES_COLOR = "#cccccc"
MAP_DRONE_COLOR = "#1f77b4"
MAP_TARGET_COLOR = "#d62728"
PX_PER_CM = 2.5  # 2.5 px = 1 cm  -> 20 cm = 50 px (misma escala visual)
GRID_STEP_CM = 20  # cada cuadricula = 20 cm (se ve como antes)
CENTER_MARK_COLOR = "#666"


def check_exclusion_zones(lat, lon, alt, excl_polygons, excl_circles):
    """
    Verifica si el punto (lat, lon, alt) está dentro de alguna zona de exclusión.

    Args:
        lat, lon, alt: coordenadas del punto a verificar
        excl_polygons: lista de polígonos de exclusión (cada uno con vértices y zmin/zmax)
        excl_circles: lista de círculos de exclusión (cada uno con cx, cy, r y zmin/zmax)

    Returns:
        tuple: (is_excluded: bool, zone_type: str, zone_info: dict)
    """
    if Point is None or Polygon is None:
        print("ERROR: shapely no está instalado. No se pueden verificar zonas de exclusión.")
        return (False, None, None)

    point = Point(lon, lat)

    # Verificar polígonos
    for entry in excl_polygons:
        try:
            # ✅ Extraer datos del dict correctamente
            if isinstance(entry, dict):
                vertices = entry.get("vertices", [])
                zmin_excl = entry.get("zmin")
                zmax_excl = entry.get("zmax")
            else:
                # Formato antiguo (solo lista de vértices)
                vertices = entry
                zmin_excl = None
                zmax_excl = None

            if not vertices or len(vertices) < 3:
                continue

            poly = Polygon(vertices)

            # Verificar si está dentro del polígono
            if poly.contains(point):
                # Verificar restricción de altitud
                in_z_range = True
                if zmin_excl is not None and alt < zmin_excl:
                    in_z_range = False
                if zmax_excl is not None and alt > zmax_excl:
                    in_z_range = False

                if in_z_range:
                    return (True, "polygon", {
                        "vertices": vertices,
                        "zmin": zmin_excl,
                        "zmax": zmax_excl
                    })
        except Exception as e:
            print(f"Error procesando polígono: {e}")
            continue

    # Verificar círculos
    for entry in excl_circles:
        try:
            # ✅ Extraer datos del dict correctamente
            if isinstance(entry, dict):
                cx2 = float(entry.get("cx", 0.0))
                cy2 = float(entry.get("cy", 0.0))
                r = float(entry.get("r", 0.0))
                zmin_excl = entry.get("zmin")
                zmax_excl = entry.get("zmax")
            else:
                # Formato antiguo (tupla: cx, cy, r)
                cx2, cy2, r = float(entry[0]), float(entry[1]), float(entry[2])
                zmin_excl = None
                zmax_excl = None

            # Calcular distancia al centro del círculo
            dx = lon - cx2
            dy = lat - cy2
            dist = (dx ** 2 + dy ** 2) ** 0.5

            # Verificar si está dentro del círculo
            if dist <= r:
                # Verificar restricción de altitud
                in_z_range = True
                if zmin_excl is not None and alt < zmin_excl:
                    in_z_range = False
                if zmax_excl is not None and alt > zmax_excl:
                    in_z_range = False

                if in_z_range:
                    return (True, "circle", {
                        "cx": cx2,
                        "cy": cy2,
                        "r": r,
                        "zmin": zmin_excl,
                        "zmax": zmax_excl
                    })
        except Exception as e:
            print(f"Error procesando círculo: {e}")
            continue

    # No está en ninguna zona de exclusión
    return (False, None, None)


class MiniRemoteApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Demo Tello Pose + Geofence")
        self.root.geometry("720x620")
        self.root.resizable(True, True)

        # ✅ Forzar colores claros (macOS modo oscuro fix)
        self.root.configure(bg="#f0f0f0")

        # Estilo por defecto para widgets
        self.default_bg = "#ffffff"
        self.default_fg = "#000000"
        self.frame_bg = "#f0f0f0"

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

        # ===================== NUEVO: Geofence (panel principal) =====================

        self.gf_max_x_var = tk.StringVar(value="0")
        self.gf_max_y_var = tk.StringVar(value="0")
        self.gf_zmin_var = tk.StringVar(value="0")
        self.gf_zmax_var = tk.StringVar(value="120")
        self.gf_mode_var = tk.StringVar(value="soft")  # soft/hard

        # ===================== NUEVO: estado ventana de mapa =====================
        self._map_win = None
        self.map_canvas = None
        self._map_static_drawn = False
        self._map_drone_item = None
        self._last_pose_key = None
        self._gf_center_item = None

        # herramientas de edición de exclusiones
        self._tool_var = None
        self._circle_radius_var = None
        self._poly_points = []  # acumulador de puntos del polígono en coords mundo (cm)

        # inclusión (rectángulo) definida desde el mapa
        self._incl_pts = []  # clicks temporales (en mundo) para la inclusión
        self._incl_rect = None  # (cx, cy, max_x, max_y) en cm
        self._incl_zmin_var = None
        self._incl_zmax_var = None

        # almacenamiento local de exclusiones para guardar/cargar plantillas
        self._excl_circles = []  # cada item: {"cx":..,"cy":..,"r":..,"zmin":..,"zmax":..}
        self._excl_polys = []  # cada item: {"points":[(x,y),...],"zmin":..,"zmax":..}

        # Keepalive
        self._keepalive_thread = None
        self._keepalive_stop = False
        self._keepalive_paused = False

        self._build_ui()
        self._bind_keys()

    # Código de la interfaz gráfica
    def _build_ui(self):
        pad = dict(padx=6, pady=6)

        # Contenedor con los datos de telemetría del dron
        top = tk.Frame(self.root, bg=self.frame_bg)
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

        # ===================== NUEVO: Panel Geofence (principal) =====================
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

        tk.Button(gf, text="Activar", command=self.on_gf_activate, bg="#cde7cd").grid(row=1, column=1, padx=4, pady=6,
                                                                                      sticky="w")
        tk.Button(gf, text="Desactivar", command=self.on_gf_disable).grid(row=1, column=2, padx=4, pady=6, sticky="w")
        tk.Button(gf, text="Abrir mapa", command=self.open_map_window, bg="#87CEEB").grid(row=1, column=4, padx=8,
                                                                                          pady=6, sticky="w")

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

            # ✅ Asegura origen de pose antes de iniciar la telemetría
            self._ensure_pose_origin()

            # Telemetría
            self.dron.startTelemetry(freq_hz=5)
            self._telemetry_running = True
            self._schedule_telemetry_pull()

            # ✅ Keepalive (arrancar después de la telemetría)
            self._start_keepalive()

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
            # cerrar mapa si estuviera abierto
            try:
                if self._map_win and tk.Toplevel.winfo_exists(self._map_win):
                    self._map_win.destroy()
            except Exception:
                pass
            self._map_win = None
            self.map_canvas = None
            self._map_static_drawn = False
            self._map_drone_item = None
            self._last_pose_key = None

    def on_takeoff(self):
        if self.dron.state not in ("connected", "flying"):
            messagebox.showwarning("TakeOff", "Conecta primero.")
            return
        bat = getattr(self.dron, "battery_pct", None)
        if isinstance(bat, int) and bat < BAT_MIN_SAFE:
            if not messagebox.askokcancel("Batería baja", f"Batería {bat}%. ¿Despegar igualmente?"):
                return
        # --- Geofence HARD: bloquear despegue si fuera de inclusión ---
        try:
            if getattr(self.dron, "_gf_enabled", False) and getattr(self.dron, "_gf_mode", "soft") == "hard":
                lim = getattr(self.dron, "_gf_limits", None)
                if lim:
                    cx, cy = getattr(self.dron, "_gf_center", (0.0, 0.0))
                    pose = getattr(self.dron, "pose", None)
                    x = float(getattr(pose, "x_cm", 0.0) or 0.0) if pose else 0.0
                    y = float(getattr(pose, "y_cm", 0.0) or 0.0) if pose else 0.0
                    # z: prefer pose.z_cm else height_cm
                    z = None
                    if pose is not None and getattr(pose, "z_cm", None) is not None:
                        z = float(pose.z_cm)
                    else:
                        try:
                            z = float(getattr(self.dron, "height_cm", 0.0) or 0.0)
                        except Exception:
                            z = 0.0
                    max_x = float(lim.get("max_x", 0.0))
                    max_y = float(lim.get("max_y", 0.0))
                    zmin = float(lim.get("zmin", 0.0))
                    half_x = max_x / 2.0
                    half_y = max_y / 2.0
                    zmax = float(lim.get("zmax", lim.get("max_z", 9999.0)))
                    inside_xy = abs(x - cx) <= half_x and abs(y - cy) <= half_y
                    inside_z = (z is None) or (zmin <= z <= zmax)
                    if not (inside_xy and inside_z):
                        messagebox.showerror(
                            "Geofence (Hard)",
                            "No se puede despegar: fuera de la zona de inclusión definida (XY/Z).\n"
                            "Ajusta el rectángulo de inclusión o pulsa 'Sincronizar destino' y vuelve a intentarlo."
                        )
                        return
        except Exception:
            pass
        try:
            # --- pausa keepalive para no cruzar respuestas ---
            self._pause_keepalive()
            ok = self.dron.takeOff(0.5, blocking=True)  # altura de seguridad
            if not ok:
                messagebox.showerror("TakeOff", "No se pudo despegar.")
            else:
                # ✅ RESTAURAR POSE si hay una guardada del último aterrizaje
                if hasattr(self, "_last_land_x"):
                    try:
                        if not hasattr(self.dron, "pose") or self.dron.pose is None:
                            if hasattr(self.dron, "PoseVirtual"):
                                self.dron.pose = self.dron.PoseVirtual()

                        if self.dron.pose:
                            self.dron.pose.x_cm = self._last_land_x
                            self.dron.pose.y_cm = self._last_land_y
                            self.dron.pose.z_cm = self._last_land_z
                            self.dron.pose.yaw_deg = self._last_land_yaw
                            print(
                                f"[takeoff] Restaurando pose: ({self._last_land_x:.1f}, {self._last_land_y:.1f}, {self._last_land_z:.1f})")
                    except Exception as e:
                        print(f"[takeoff] Error restaurando pose: {e}")
                        self._ensure_pose_origin()
                else:
                    # Primera vez: crear origen
                    self._ensure_pose_origin()

                self._restart_gf_monitor(force=True)
                # Reinyecta exclusiones tras el reinicio del monitor al despegar
                try:
                    self._reapply_exclusions_to_backend()
                except Exception:
                    pass
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

            # ✅ GUARDAR POSE ANTES DE ATERRIZAR
            pose = getattr(self.dron, "pose", None)
            if pose:
                try:
                    self._last_land_x = float(getattr(pose, "x_cm", 0.0) or 0.0)
                    self._last_land_y = float(getattr(pose, "y_cm", 0.0) or 0.0)
                    self._last_land_z = float(getattr(pose, "z_cm", 0.0) or h)
                    self._last_land_yaw = float(getattr(pose, "yaw_deg", 0.0) or 0.0)
                    print(
                        f"[land] Guardando pose: ({self._last_land_x:.1f}, {self._last_land_y:.1f}, {self._last_land_z:.1f})")
                except Exception as e:
                    print(f"[land] Error guardando pose: {e}")

            if h <= 20:
                print("[land] Altura ≤ 20 cm. Ya está en el suelo; no mando 'land'.")
                return

            if st not in ("flying", "hovering", "takingoff", "landing"):
                print(f"[land] Estado actual '{st}' no permite 'land'; no mando.")
                return

            # --- Evita falsos positivos del monitor durante un aterrizaje voluntario ---
            try:
                # (1) Para el monitor antes de mandar 'Land'
                if hasattr(self.dron, "_stop_geofence_monitor"):
                    self.dron._stop_geofence_monitor()
                # (2) Señal opcional para backend (si la respeta)
                setattr(self.dron, "_gf_suppress_until", time.time() + 6.0)
            except Exception:
                pass

            self._pause_keepalive()
            self.dron.Land(blocking=False)

        except Exception as e:
            messagebox.showerror("Land", str(e))
        finally:
            # Reanuda keepalive
            self.root.after(150, self._resume_keepalive)
            # Reanuda el monitor tras aterrizar (delay suficiente para terminar el land)
            self.root.after(4000, lambda: self._restart_gf_monitor(force=True))
            # Libera debounce un poco más tarde
            self.root.after(4500, lambda: setattr(self, "_ui_landing", False))


    def on_exit(self):
        self.on_disconnect()
        self.root.quit()

    def apply_speed(self):
        try:
            sp = int(self.speed_var.get())
            sp = max(10, min(100, sp))  # límites SDK
            self.dron.set_speed(sp)
            self.speed_var.set(sp)
        except Exception as e:
            messagebox.showerror("Velocidad", f"No se pudo aplicar velocidad: {e}")

    def _pt_in_poly(self, px, py, poly, eps=1e-6):
        """✅ MEJORADO: Ray casting con detección de bordes."""
        n = len(poly)
        if n < 3:
            return False

        # Detección explícita de bordes
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            # Punto en segmento
            cross = abs((px - x1) * (y2 - y1) - (py - y1) * (x2 - x1))
            if cross < eps:
                dot = (px - x1) * (px - x2) + (py - y1) * (py - y2)
                if dot <= eps:
                    return True  # En el borde

        # Ray casting estándar
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = poly[i]
            xj, yj = poly[j]
            if ((yi > py) != (yj > py)) and \
                    (px < (xj - xi) * (py - yi) / (yj - yi + 1e-9) + xi):
                inside = not inside
            j = i
        return inside

    def do_move(self, cmd: str):
        """✅ CORREGIDO: Validación completa con dicts y Z-range."""
        if self.dron.state != "flying":
            messagebox.showinfo("Movimiento", "Despega primero.")
            return
        try:
            step = int(self.step_var.get())
            step = max(20, min(500, step))  # límites SDK

            # Leer estado geofence
            enabled = bool(getattr(self.dron, "_gf_enabled", False))
            mode = getattr(self.dron, "_gf_mode", "soft")
            cx, cy = getattr(self.dron, "_gf_center", (0.0, 0.0))
            lim = getattr(self.dron, "_gf_limits", {}) or {}
            max_x = float(lim.get("max_x", 0.0) or 0.0)
            max_y = float(lim.get("max_y", 0.0) or 0.0)
            zmin = float(lim.get("zmin", 0.0) or 0.0)
            zmax = float(lim.get("zmax", lim.get("max_z", 9999.0)))
            half_x = max_x / 2.0
            half_y = max_y / 2.0

            sub = 20
            remaining = step
            moved_any = False

            while remaining > 0:
                d = min(sub, remaining)
                remaining -= d

                pose = getattr(self.dron, "pose", None)
                x = float(getattr(pose, "x_cm", 0.0) or 0.0) if pose else 0.0
                y = float(getattr(pose, "y_cm", 0.0) or 0.0) if pose else 0.0
                if pose is not None and getattr(pose, "z_cm", None) is not None:
                    z = float(pose.z_cm)
                else:
                    z = float(getattr(self.dron, "height_cm", 0.0) or 0.0)

                yaw = float(getattr(pose, "yaw_deg", 0.0) or 0.0)
                th = math.radians(yaw)
                c_, s_ = math.cos(th), math.sin(th)
                dx = dy = dz = 0.0
                if cmd == "forward":
                    dx, dy = d * c_, d * s_
                elif cmd == "back":
                    dx, dy = -d * c_, -d * s_
                elif cmd == "right":
                    dx, dy = -d * s_, d * c_  # ← CORREGIDO
                elif cmd == "left":
                    dx, dy = d * s_, -d * c_  # ← CORREGIDO
                elif cmd == "up":
                    dz = d
                elif cmd == "down":
                    dz = -d

                nx, ny, nz = x + dx, y + dy, z + dz

                # --- VALIDACIÓN GEOFENCE ---
                # 1) Inclusión (solo si geofence está activado y hay rectángulo de inclusión)
                if enabled:
                    use_inclusion = bool(lim) and (max_x > 0.0 and max_y > 0.0)
                    if use_inclusion:
                        inside_xy = abs(nx - cx) <= half_x and abs(ny - cy) <= half_y
                        inside_z = (zmin <= nz <= zmax)
                        if not (inside_xy and inside_z):
                            if mode == "hard":
                                print(
                                    f"[geofence][HARD] ❌ BLOQUEADO {cmd} {d} -> next=({nx:.1f},{ny:.1f},{nz:.1f}) fuera de inclusión.")
                                break
                            else:
                                # SOFT: Solo advertir, NO bloquear
                                if not inside_xy:
                                    print(f"[geofence][SOFT] ⚠️ ADVERTENCIA XY: next=({nx:.1f},{ny:.1f}) "
                                          f"saliendo de zona - center=({cx:.1f},{cy:.1f}) ancho=(X={max_x:.0f}cm, Y={max_y:.0f}cm)")
                                if not inside_z:
                                    print(
                                        f"[geofence][SOFT] ⚠️ ADVERTENCIA Z: next_z={nz:.1f} fuera de [{zmin:.0f},{zmax:.0f}]")
                                # NO hacer break en SOFT → el movimiento continúa

                # 2) Exclusiones - ✅ CORREGIDO: Maneja dicts uniformemente y valida Z
                excl_polys = list(getattr(self.dron, "_gf_excl_polys", []))
                excl_circles = list(getattr(self.dron, "_gf_excl_circles", []))

                def _blocked_by_exclusion():
                    # Polígonos
                    for entry in excl_polys:
                        try:
                            # ✅ Extraer datos del dict correctamente
                            if isinstance(entry, dict):
                                poly = entry.get("poly", [])
                                zmin_excl = entry.get("zmin")
                                zmax_excl = entry.get("zmax")
                            else:
                                # Formato antiguo (lista directa de puntos)
                                poly = entry
                                zmin_excl = zmax_excl = None

                            if self._pt_in_poly(nx, ny, poly):
                                # ✅ Validar rango de altura
                                z_ok = (zmin_excl is None or nz >= zmin_excl) and (zmax_excl is None or nz <= zmax_excl)
                                if z_ok:
                                    print(
                                        f"[geofence][{mode.upper()}] EXCLUSIÓN POLY @ next=({nx:.1f},{ny:.1f},{nz:.1f})")
                                    return True
                                else:
                                    print(
                                        f"[geofence][DEBUG] POLY XY match pero Z fuera de rango: nz={nz:.1f}, zmin={zmin_excl}, zmax={zmax_excl}")
                        except Exception as e:
                            print(f"[WARN] Error validando poly: {e}")
                            pass

                    # Círculos
                    for entry in excl_circles:
                        try:
                            # ✅ Extraer datos del dict correctamente
                            if isinstance(entry, dict):
                                cx2 = float(entry.get("cx", 0.0))
                                cy2 = float(entry.get("cy", 0.0))
                                r = float(entry.get("r", 0.0))
                                zmin_excl = entry.get("zmin")
                                zmax_excl = entry.get("zmax")
                            else:
                                # Formato antiguo (tupla: cx, cy, r)
                                cx2, cy2, r = float(entry[0]), float(entry[1]), float(entry[2])
                                zmin_excl = zmax_excl = None

                            dist = math.sqrt((nx - cx2) ** 2 + (ny - cy2) ** 2)
                            if dist <= r:
                                # ✅ Validar rango de altura
                                z_ok = (zmin_excl is None or nz >= zmin_excl) and (zmax_excl is None or nz <= zmax_excl)
                                if z_ok:
                                    print(
                                        f"[geofence][{mode.upper()}] EXCLUSIÓN CIRCLE @ next=({nx:.1f},{ny:.1f},{nz:.1f})")
                                    return True
                                else:
                                    print(
                                        f"[geofence][DEBUG] CIRCLE XY match pero Z fuera de rango: nz={nz:.1f}, zmin={zmin_excl}, zmax={zmax_excl}")
                        except Exception as e:
                            print(f"[WARN] Error validando circle: {e}")
                            pass

                    return False

                if _blocked_by_exclusion():
                    if mode == "hard":
                        print(f"[geofence][HARD] ❌ BLOQUEADO {cmd} {d} por exclusión.")
                        break
                    else:
                        # SOFT: Advertir pero NO bloquear exclusiones tampoco
                        print(f"[geofence][SOFT] ⚠️ ADVERTENCIA: zona de exclusión detectada, pero continuando...")
                        # NO hacer break → el movimiento continúa

                # --- Ejecutar movimiento ---
                self._pause_keepalive()
                try:
                    success = False
                    if cmd == "forward":
                        success = self.dron.forward(d)
                    elif cmd == "back":
                        success = self.dron.back(d)
                    elif cmd == "left":
                        success = self.dron.left(d)
                    elif cmd == "right":
                        success = self.dron.right(d)
                    elif cmd == "up":
                        success = self.dron.up(d)
                    elif cmd == "down":
                        success = self.dron.down(d)

                    if not success:
                        print(f"[move] Fallo en {cmd} {d} cm.")
                        break
                    moved_any = True
                except Exception as e:
                    print(f"[move] Excepción en {cmd} {d}: {e}")
                    break
                finally:
                    self.root.after(150, self._resume_keepalive)

            if not moved_any:
                print(f"[move] No se pudo mover en {cmd}.")

        except Exception as e:
            messagebox.showerror("Movimiento", str(e))


    def do_turn(self, direction: str):
        if self.dron.state != "flying":
            messagebox.showinfo("Giro", "Despega primero.")
            return
        try:
            angle = int(self.angle_var.get())
            angle = max(1, min(360, angle))
            self._pause_keepalive()
            if direction == "cw":
                ok = self.dron.cw(angle)
            else:
                ok = self.dron.ccw(angle)
            if not ok:
                messagebox.showerror("Giro", f"No se pudo girar {direction} {angle}°")
        except Exception as e:
            messagebox.showerror("Giro", str(e))
        finally:
            self.root.after(150, self._resume_keepalive)

    # ===================== KEEPALIVE =====================
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

            # POSE: actualizar panel
            pose = getattr(self.dron, "pose", None)
            if pose:
                x = getattr(pose, "x_cm", None)
                y = getattr(pose, "y_cm", None)
                z = getattr(pose, "z_cm", None)
                yaw = getattr(pose, "yaw_deg", None)
                self.x_var.set(f"X: {x:.1f} cm" if x is not None else "X: —")
                self.y_var.set(f"Y: {y:.1f} cm" if y is not None else "Y: —")
                self.z_var.set(f"Z: {z:.1f} cm" if z is not None else "Z: —")
                self.yaw_var.set(f"Yaw: {yaw:.1f}°" if yaw is not None else "Yaw: —")

                # ✅ GUARDAR POSE CONTINUAMENTE (para aterrizajes de emergencia)
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

            # Mapa: actualizar posición del dron
            self._update_map_drone()

        except Exception as e:
            print(f"[telemetry] Error: {e}")

    def _ensure_pose_origin(self):
        """Asegura que el dron tenga un origen de pose definido."""
        try:
            if not hasattr(self.dron, "pose") or self.dron.pose is None:
                if hasattr(self.dron, "set_pose_origin"):
                    self.dron.set_pose_origin()
                    print("[pose] Origen establecido.")
        except Exception as e:
            print(f"[pose] Error estableciendo origen: {e}")

    # ===================== GEOFENCE =====================
    def on_gf_activate(self):
        """Activa el geofence con los parámetros de la UI."""
        try:
            max_x = float(self.gf_max_x_var.get() or 0.0)
            max_y = float(self.gf_max_y_var.get() or 0.0)
            zmin = float(self.gf_zmin_var.get() or 0.0)
            zmax = float(self.gf_zmax_var.get() or 120.0)
            mode = self.gf_mode_var.get()

            if max_x <= 0 or max_y <= 0:
                messagebox.showwarning("Geofence", "Define ancho X/Y > 0.")
                return

            # Obtener centro actual (pose o 0,0)
            pose = getattr(self.dron, "pose", None)
            cx = float(getattr(pose, "x_cm", 0.0) or 0.0) if pose else 0.0
            cy = float(getattr(pose, "y_cm", 0.0) or 0.0) if pose else 0.0

            # Activar en backend
            if hasattr(self.dron, "set_geofence"):
                # ✅ Usar firma correcta del backend: max_x_cm, max_y_cm, max_z_cm, z_min_cm, mode
                self.dron.set_geofence(
                    max_x_cm=max_x,
                    max_y_cm=max_y,
                    max_z_cm=zmax,
                    z_min_cm=zmin,
                    mode=mode
                )
                # El backend maneja internamente el centro y los límites
            else:
                # Fallback: asignar atributos directamente
                setattr(self.dron, "_gf_enabled", True)
                setattr(self.dron, "_gf_center", (cx, cy))
                setattr(self.dron, "_gf_limits", {"max_x": max_x, "max_y": max_y, "zmin": zmin, "zmax": zmax})
                setattr(self.dron, "_gf_mode", mode)

            # Reinyectar exclusiones (el backend ya reinicia el monitor automáticamente)
            self._reapply_exclusions_to_backend()

            messagebox.showinfo("Geofence",
                                f"Activado ({mode}) con centro ({cx:.1f}, {cy:.1f}) y límites X={max_x}, Y={max_y}, Z=[{zmin},{zmax}].")

            # Redibujar mapa si está abierto
            if self._map_win and tk.Toplevel.winfo_exists(self._map_win):
                self._redraw_map_static()

        except Exception as e:
            messagebox.showerror("Geofence", f"Error activando: {e}")

    def on_gf_disable(self):
        """Desactiva el geofence."""
        try:
            if hasattr(self.dron, "set_geofence"):
                self.dron.set_geofence(enabled=False)
            else:
                setattr(self.dron, "_gf_enabled", False)

            # Detener monitor
            if hasattr(self.dron, "_stop_geofence_monitor"):
                self.dron._stop_geofence_monitor()

            messagebox.showinfo("Geofence", "Desactivado.")

            # Redibujar mapa
            if self._map_win and tk.Toplevel.winfo_exists(self._map_win):
                self._redraw_map_static()

        except Exception as e:
            messagebox.showerror("Geofence", f"Error desactivando: {e}")

    def _restart_gf_monitor(self, force=False):
        """Reinicia el monitor de geofence en el backend."""
        try:
            if hasattr(self.dron, "_stop_geofence_monitor"):
                self.dron._stop_geofence_monitor()
            if hasattr(self.dron, "_start_geofence_monitor"):
                self.dron._start_geofence_monitor()
        except Exception as e:
            print(f"[gf_monitor] Error reiniciando: {e}")

    def _reapply_exclusions_to_backend(self):
        """Reinyecta las exclusiones locales al backend tras reiniciar el monitor."""
        try:
            # Asegurar que existan las listas en el backend
            if not hasattr(self.dron, "_gf_excl_circles"):
                self.dron._gf_excl_circles = []
            if not hasattr(self.dron, "_gf_excl_polys"):
                self.dron._gf_excl_polys = []

            # Limpiar listas existentes
            self.dron._gf_excl_circles.clear()
            self.dron._gf_excl_polys.clear()

            # Círculos - guardar directamente en el backend
            for c in self._excl_circles:
                self.dron._gf_excl_circles.append({
                    "cx": c["cx"],
                    "cy": c["cy"],
                    "r": c["r"],
                    "zmin": c.get("zmin"),
                    "zmax": c.get("zmax")
                })

            # Polígonos - guardar directamente en el backend
            for p in self._excl_polys:
                self.dron._gf_excl_polys.append({
                    "poly": p["poly"],
                    "zmin": p.get("zmin"),
                    "zmax": p.get("zmax")
                })

            print(f"[exclusions] Reinyectadas {len(self._excl_circles)} círculos y {len(self._excl_polys)} polígonos")
        except Exception as e:
            print(f"[exclusions] Error reinyectando: {e}")

    # ===================== MAPA =====================
    def open_map_window(self):
        """Abre la ventana del mapa."""
        if self._map_win and tk.Toplevel.winfo_exists(self._map_win):
            self._map_win.lift()
            return

        self._map_win = tk.Toplevel(self.root)
        self._map_win.title("Mapa Geofence")
        self._map_win.geometry(f"{MAP_SIZE_PX + 300}x{MAP_SIZE_PX + 50}")

        # Canvas
        self.map_canvas = tk.Canvas(
            self._map_win,
            width=MAP_SIZE_PX,
            height=MAP_SIZE_PX,
            bg=MAP_BG_COLOR,
            highlightthickness=0
        )
        self.map_canvas.pack(side="left", padx=10, pady=10)

        # Panel lateral (herramientas)
        side_panel = tk.Frame(self._map_win, bd=1, relief="groove")
        side_panel.pack(side="right", fill="y", padx=10, pady=10)

        tk.Label(side_panel, text="Herramientas de Exclusión", font=("Arial", 10, "bold")).pack(pady=6)

        self._tool_var = tk.StringVar(value="none")
        tk.Radiobutton(side_panel, text="Ninguna", variable=self._tool_var, value="none").pack(anchor="w")
        tk.Radiobutton(side_panel, text="Círculo", variable=self._tool_var, value="circle").pack(anchor="w")
        tk.Radiobutton(side_panel, text="Polígono", variable=self._tool_var, value="polygon").pack(anchor="w")

        tk.Label(side_panel, text="Radio círculo (cm):").pack(pady=(10, 0))
        self._circle_radius_var = tk.IntVar(value=30)
        tk.Entry(side_panel, textvariable=self._circle_radius_var, width=8).pack()

        tk.Button(side_panel, text="Cerrar polígono", command=self._close_polygon).pack(pady=6)
        tk.Button(side_panel, text="Limpiar exclusiones", command=self._clear_exclusions).pack(pady=6)

        tk.Label(side_panel, text="--- Inclusión (Rect) ---", font=("Arial", 9, "bold")).pack(pady=(12, 4))
        tk.Button(side_panel, text="Definir rectángulo (2 clics)", command=self._start_inclusion_rect).pack(pady=4)
        tk.Button(side_panel, text="Sincronizar destino", command=self._sync_inclusion_to_gf).pack(pady=4)

        tk.Label(side_panel, text="Z min (cm):").pack()
        self._incl_zmin_var = tk.IntVar(value=0)
        tk.Entry(side_panel, textvariable=self._incl_zmin_var, width=8).pack()

        tk.Label(side_panel, text="Z max (cm):").pack()
        self._incl_zmax_var = tk.IntVar(value=120)
        tk.Entry(side_panel, textvariable=self._incl_zmax_var, width=8).pack()

        tk.Label(side_panel, text="--- Plantillas ---", font=("Arial", 9, "bold")).pack(pady=(12, 4))
        tk.Button(side_panel, text="Guardar plantilla", command=self._save_template).pack(pady=4)
        tk.Button(side_panel, text="Cargar plantilla", command=self._load_template).pack(pady=4)

        # Bind clicks
        self.map_canvas.bind("<Button-1>", self._on_map_click)

        # Dibujar
        self._map_static_drawn = False
        self._redraw_map_static()
        self._last_pose_key = None
        self._update_map_drone()

    def _redraw_map_static(self):
        """Redibuja los elementos estáticos del mapa (grid, inclusión, exclusiones)."""
        if not self.map_canvas:
            return

        self.map_canvas.delete("all")
        self._map_static_drawn = True

        # Grid
        for i in range(0, MAP_SIZE_PX + 1, int(GRID_STEP_CM * PX_PER_CM)):
            self.map_canvas.create_line(i, 0, i, MAP_SIZE_PX, fill=MAP_AXES_COLOR, width=1)
            self.map_canvas.create_line(0, i, MAP_SIZE_PX, i, fill=MAP_AXES_COLOR, width=1)

        # Ejes
        mid = MAP_SIZE_PX // 2
        self.map_canvas.create_line(mid, 0, mid, MAP_SIZE_PX, fill="#888", width=2)
        self.map_canvas.create_line(0, mid, MAP_SIZE_PX, mid, fill="#888", width=2)

        # Rectángulo de inclusión del backend (geofence activado)
        enabled = bool(getattr(self.dron, "_gf_enabled", False))
        if enabled:
            lim = getattr(self.dron, "_gf_limits", {})
            cx, cy = getattr(self.dron, "_gf_center", (0.0, 0.0))
            max_x = float(lim.get("max_x", 0.0) or 0.0)
            max_y = float(lim.get("max_y", 0.0) or 0.0)
            if max_x > 0 and max_y > 0:
                x1, y1 = self._world_to_canvas(cx - max_x / 2, cy - max_y / 2)
                x2, y2 = self._world_to_canvas(cx + max_x / 2, cy + max_y / 2)
                self.map_canvas.create_rectangle(x1, y1, x2, y2, outline="#00aa00", width=3, tags="inclusion")

        # ✅ NUEVO: Rectángulo de inclusión local (definido en mapa pero no activado aún)
        if self._incl_rect:
            self._draw_inclusion_rect(self._incl_rect)

        # Exclusiones (círculos)
        for c in self._excl_circles:
            cx_w, cy_w, r_w = c["cx"], c["cy"], c["r"]
            cx_px, cy_px = self._world_to_canvas(cx_w, cy_w)
            r_px = r_w * PX_PER_CM
            self.map_canvas.create_oval(
                cx_px - r_px, cy_px - r_px,
                cx_px + r_px, cy_px + r_px,
                outline="#ff0000", width=2, fill="", tags="exclusion"
            )

        # Exclusiones (polígonos)
        for p in self._excl_polys:
            pts = p["poly"]
            if len(pts) >= 3:
                canvas_pts = []
                for (px, py) in pts:
                    canvas_pts.extend(self._world_to_canvas(px, py))
                self.map_canvas.create_polygon(canvas_pts, outline="#ff0000", fill="", width=2, tags="exclusion")

    def _update_map_drone(self):
        """Actualiza la posición del dron en el mapa."""
        # Validar que la ventana y el canvas existen
        try:
            if not self._map_win or not self.map_canvas:
                return
            # Verificar que la ventana sigue existente
            if not tk.Toplevel.winfo_exists(self._map_win):
                self.map_canvas = None
                self._map_win = None
                return
        except Exception as e:
            print(f"[DEBUG] Error validando ventana: {e}")
            self.map_canvas = None
            self._map_win = None
            return

        pose = getattr(self.dron, "pose", None)
        if not pose:
            return

        x = getattr(pose, "x_cm", None)
        y = getattr(pose, "y_cm", None)
        yaw = getattr(pose, "yaw_deg", None)

        if x is None or y is None:
            return

        key = (round(x, 1), round(y, 1), round(yaw, 1) if yaw else 0)
        if key == self._last_pose_key:
            return
        self._last_pose_key = key

        print(f"[DEBUG] Dibujando dron en mapa: x={x:.1f}, y={y:.1f}, yaw={yaw}")

        try:
            # Eliminar dron anterior (círculo Y flecha)
            if self._map_drone_item:
                self.map_canvas.delete("drone")  # Borra TODO con tag "drone"

            # Dibujar nuevo círculo
            cx_px, cy_px = self._world_to_canvas(x, y)
            print(f"[DEBUG] Coordenadas canvas: cx={cx_px:.1f}, cy={cy_px:.1f}")
            rad = 8
            self._map_drone_item = self.map_canvas.create_oval(
                cx_px - rad, cy_px - rad,
                cx_px + rad, cy_px + rad,
                fill=MAP_DRONE_COLOR, outline="black", width=2, tags="drone"
            )

            # Dirección (yaw) - La flecha apunta hacia donde mira el dron
            if yaw is not None:
                # yaw=0° → adelante (arriba en canvas)
                # yaw=90° → derecha
                th = math.radians(yaw)
                # En mundo: X=adelante, Y=derecha
                # Flecha en dirección del yaw
                arrow_len = 20
                dx_world = arrow_len * math.cos(th)  # componente Y (derecha)
                dy_world = arrow_len * math.sin(th)  # componente X (adelante)

                # Convertir a canvas: X mundo→Y canvas, Y mundo→X canvas
                dx_canvas = dy_world * PX_PER_CM  # Y mundo → X canvas
                dy_canvas = -dx_world * PX_PER_CM  # X mundo → Y canvas (negativo)

                self.map_canvas.create_line(
                    cx_px, cy_px,
                    cx_px + dx_canvas, cy_px + dy_canvas,
                    fill="red", width=2, arrow=tk.LAST, tags="drone"
                )
        except Exception as e:
            # Si falla, limpiar referencias
            print(f"[DEBUG] Error dibujando dron: {e}")
            self.map_canvas = None
            self._map_win = None

    def _on_map_click(self, event):
        """Maneja clics en el mapa para añadir exclusiones o definir inclusión."""
        if not self._tool_var:
            return

        tool = self._tool_var.get()
        wx, wy = self._canvas_to_world(event.x, event.y)

        if tool == "circle":
            self._add_exclusion_circle(wx, wy)
        elif tool == "polygon":
            self._add_polygon_point(wx, wy)
        elif tool == "inclusion_rect":
            self._add_inclusion_point(wx, wy)

    def _add_exclusion_circle(self, wx, wy):
        """Añade un círculo de exclusión."""
        r = float(self._circle_radius_var.get() or 30.0)
        zmin = self._incl_zmin_var.get()
        zmax = self._incl_zmax_var.get()

        # Guardar localmente
        self._excl_circles.append({"cx": wx, "cy": wy, "r": r, "zmin": zmin, "zmax": zmax})

        # Enviar al backend (con manejo de errores)
        try:
            if hasattr(self.dron, "add_exclusion_circle"):
                # ✅ Usar nombres correctos del backend: z_min_cm y z_max_cm
                self.dron.add_exclusion_circle(
                    cx=wx, cy=wy, r_cm=r,
                    z_min_cm=zmin, z_max_cm=zmax
                )
            else:
                # No existe el método, guardar directamente
                if not hasattr(self.dron, "_gf_excl_circles"):
                    self.dron._gf_excl_circles = []
                self.dron._gf_excl_circles.append({"cx": wx, "cy": wy, "r": r, "zmin": zmin, "zmax": zmax})
        except Exception as e:
            print(f"[WARN] No se pudo enviar círculo al backend: {e}")

        self._redraw_map_static()

    def _add_polygon_point(self, wx, wy):
        """Añade un punto al polígono en construcción."""
        self._poly_points.append((wx, wy))
        # Dibujar punto temporal
        cx_px, cy_px = self._world_to_canvas(wx, wy)
        self.map_canvas.create_oval(
            cx_px - 4, cy_px - 4, cx_px + 4, cy_px + 4,
            fill="orange", outline="black", tags="poly_temp"
        )

    def _close_polygon(self):
        """Cierra el polígono en construcción."""
        if len(self._poly_points) < 3:
            messagebox.showwarning("Polígono", "Necesitas al menos 3 puntos.")
            return

        zmin = self._incl_zmin_var.get()
        zmax = self._incl_zmax_var.get()

        # Guardar localmente
        self._excl_polys.append({"poly": list(self._poly_points), "zmin": zmin, "zmax": zmax})

        # Enviar al backend (con manejo de errores)
        try:
            if hasattr(self.dron, "add_exclusion_poly"):
                # ✅ Usar nombres correctos del backend: z_min_cm y z_max_cm
                self.dron.add_exclusion_poly(
                    points=list(self._poly_points),
                    z_min_cm=zmin, z_max_cm=zmax
                )
            else:
                # No existe el método, guardar directamente
                if not hasattr(self.dron, "_gf_excl_polys"):
                    self.dron._gf_excl_polys = []
                self.dron._gf_excl_polys.append({"poly": list(self._poly_points), "zmin": zmin, "zmax": zmax})
        except Exception as e:
            print(f"[WARN] No se pudo enviar polígono al backend: {e}")

        # Limpiar temporales
        self._poly_points.clear()
        self.map_canvas.delete("poly_temp")
        self._redraw_map_static()

    def _clear_exclusions(self):
        """Limpia todas las exclusiones."""
        self._excl_circles.clear()
        self._excl_polys.clear()
        self._poly_points.clear()

        # Limpiar en backend
        if hasattr(self.dron, "_gf_excl_circles"):
            self.dron._gf_excl_circles.clear()
        if hasattr(self.dron, "_gf_excl_polys"):
            self.dron._gf_excl_polys.clear()

        self.map_canvas.delete("poly_temp")
        self._redraw_map_static()

    def _start_inclusion_rect(self):
        """Inicia la definición del rectángulo de inclusión."""
        self._incl_pts.clear()
        self._tool_var.set("inclusion_rect")
        messagebox.showinfo("Inclusión", "Haz clic en dos esquinas opuestas del rectángulo.")

    def _add_inclusion_point(self, wx, wy):
        """Añade un punto al rectángulo de inclusión."""
        self._incl_pts.append((wx, wy))
        if len(self._incl_pts) == 2:
            # Calcular rectángulo
            x1, y1 = self._incl_pts[0]
            x2, y2 = self._incl_pts[1]
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            max_x = abs(x2 - x1)
            max_y = abs(y2 - y1)

            self._incl_rect = (cx, cy, max_x, max_y)
            self._incl_pts.clear()
            self._tool_var.set("none")

            # Actualizar UI
            self.gf_max_x_var.set(str(int(max_x)))
            self.gf_max_y_var.set(str(int(max_y)))

            messagebox.showinfo("Inclusión",
                                f"Rectángulo definido: centro ({cx:.1f}, {cy:.1f}), ancho X={max_x:.1f}, Y={max_y:.1f}.")
            self._redraw_map_static()

    def _sync_inclusion_to_gf(self):
        """Sincroniza el rectángulo de inclusión con el geofence."""
        if not self._incl_rect:
            messagebox.showwarning("Inclusión", "Define primero el rectángulo.")
            return

        cx, cy, max_x, max_y = self._incl_rect
        zmin = self._incl_zmin_var.get()
        zmax = self._incl_zmax_var.get()
        mode = self.gf_mode_var.get()

        # Activar geofence
        if hasattr(self.dron, "set_geofence"):
            self.dron.set_geofence(
                enabled=True,
                center=(cx, cy),
                limits={"max_x": max_x, "max_y": max_y, "zmin": zmin, "zmax": zmax},
                mode=mode
            )
        else:
            setattr(self.dron, "_gf_enabled", True)
            setattr(self.dron, "_gf_center", (cx, cy))
            setattr(self.dron, "_gf_limits", {"max_x": max_x, "max_y": max_y, "zmin": zmin, "zmax": zmax})
            setattr(self.dron, "_gf_mode", mode)

        # Reiniciar monitor
        self._restart_gf_monitor(force=True)
        self._reapply_exclusions_to_backend()

        messagebox.showinfo("Geofence",
                            f"Sincronizado: centro ({cx:.1f}, {cy:.1f}), X={max_x:.1f}, Y={max_y:.1f}, Z=[{zmin},{zmax}].")
        self._redraw_map_static()

    def _save_template(self):
        """Guarda las exclusiones e inclusión en un archivo JSON."""
        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON", "*.json")])
        if not path:
            return

        data = {
            "circles": self._excl_circles,
            "polygons": self._excl_polys,
            "inclusion": self._incl_rect,
            "zmin": self._incl_zmin_var.get(),
            "zmax": self._incl_zmax_var.get()
        }

        try:
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
            messagebox.showinfo("Plantilla", "Guardada correctamente.")
        except Exception as e:
            messagebox.showerror("Plantilla", f"Error guardando: {e}")

    def _load_template(self):
        """Carga exclusiones e inclusión desde un archivo JSON."""
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if not path:
            return

        try:
            with open(path, "r") as f:
                data = json.load(f)

            self._excl_circles = data.get("circles", [])
            self._excl_polys = data.get("polygons", [])
            self._incl_rect = data.get("inclusion")
            self._incl_zmin_var.set(data.get("zmin", 0))
            self._incl_zmax_var.set(data.get("zmax", 120))

            # Actualizar UI si hay inclusión
            if self._incl_rect:
                _, _, max_x, max_y = self._incl_rect
                self.gf_max_x_var.set(str(int(max_x)))
                self.gf_max_y_var.set(str(int(max_y)))

            # Reinyectar al backend
            self._reapply_exclusions_to_backend()

            self._redraw_map_static()
            messagebox.showinfo("Plantilla", "Cargada correctamente.")

        except Exception as e:
            messagebox.showerror("Plantilla", f"Error cargando: {e}")

    def _world_to_canvas(self, wx, wy):
        """Convierte coordenadas mundo (cm) a canvas (px).
        Mundo: X=adelante, Y=derecha
        Canvas: X=derecha, Y=abajo
        """
        mid = MAP_SIZE_PX / 2.0
        cx = mid + (wy * PX_PER_CM)  # Y mundo → X canvas (derecha)
        cy = mid - (wx * PX_PER_CM)  # X mundo → Y canvas (arriba es negativo)
        return (cx, cy)

    def _canvas_to_world(self, cx, cy):
        """Convierte coordenadas canvas (px) a mundo (cm)."""
        mid = MAP_SIZE_PX / 2.0
        wy = (cx - mid) / PX_PER_CM  # X canvas → Y mundo
        wx = (mid - cy) / PX_PER_CM  # Y canvas → X mundo
        return (wx, wy)

    def _draw_inclusion_rect(self, rect_tuple):
        """Dibuja el rectángulo de inclusión en el mapa."""
        if self.map_canvas is None or not rect_tuple:
            return

        cx_cm, cy_cm, max_x, max_y = rect_tuple
        c = self.map_canvas

        # Borrar rectángulo anterior
        c.delete("inclusion")

        # Calcular las 4 esquinas del rectángulo en coordenadas mundo
        x1, y1 = cx_cm - (max_x / 2), cy_cm - (max_y / 2)
        x2, y2 = cx_cm + (max_x / 2), cy_cm + (max_y / 2)

        # Convertir a coordenadas canvas
        p1x, p1y = self._world_to_canvas(x1, y1)
        p2x, p2y = self._world_to_canvas(x2, y2)

        # Dibujar rectángulo violeta
        c.create_rectangle(p1x, p1y, p2x, p2y, outline="#8a2be2", width=3, tags=("inclusion",))








# ===================== MAIN =====================
if __name__ == "__main__":
    root = tk.Tk()

    # ✅ FIX para modo oscuro en macOS
    try:
        import platform

        if platform.system() == "Darwin":  # macOS
            # Opción 1: Usar apariencia del sistema pero con colores explícitos
            try:
                # Esto funciona en algunas versiones de macOS
                root.tk.call("set", "::tk::mac::useCustom MDEFProc", "1")
            except:
                pass
    except:
        pass

    app = MiniRemoteApp(root)
    root.mainloop()