class TelloDron(object):

    def __init__(self, id=None):
        print(f"TelloDron inicializado (ID: {id if id else 'sin ID'})")

        # Identificación y estado
        self.id = id
        self.state = "disconnected"  # Posibles: disconnected, connected, takingOff, flying, landing

        # Telemetría
        self.height_cm = 0
        self.battery_pct = None
        self.temp_c = None
        self.wifi = None
        self.flight_time_s = 0
        self.telemetry_ts = None

        # Backend djitellopy
        self._tello = None

        # Pose virtual
        from TelloLink.modules.tello_pose import PoseVirtual
        self.pose = PoseVirtual()

        # Flags internos usados por goto, mission y geofence
        self._goto_abort = False
        self._mission_abort = False
        self._gf_enabled = False
        self._gf_monitoring = False

        self._landing_in_progress = False
        self._takeoff_in_progress = False


    # --- Métodos "colgados" desde los módulos ---
    from TelloLink.modules.tello_camera import stream_on, stream_off, get_frame, snapshot
    from TelloLink.modules.tello_connect import connect, _connect, disconnect, _send, _require_connected
    from TelloLink.modules.tello_takeOff import takeOff, _takeOff, _ascend_to_target
    from TelloLink.modules.tello_land import Land, _land
    from TelloLink.modules.tello_telemetry import startTelemetry, stopTelemetry
    from TelloLink.modules.tello_move import _move, up, down, set_speed, forward, back, left, right, rc
    from TelloLink.modules.tello_heading import rotate, cw, ccw
    from TelloLink.modules.tello_video import start_video, stop_video, show_video_blocking
    from TelloLink.modules.tello_pose import PoseVirtual
    from TelloLink.modules.tello_goto import goto_rel, abort_goto
    from TelloLink.modules.tello_mission import run_mission, abort_mission
    from TelloLink.modules.tello_geofence import set_geofence, disable_geofence, recenter_geofence, add_exclusion_poly, add_exclusion_circle, clear_exclusions
