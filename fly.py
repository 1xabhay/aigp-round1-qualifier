from __future__ import annotations

import threading
import time

import numpy as np
from pymavlink import mavutil

from aigp_pilot import control, course, parsers, raceconfig, raceline

HOVER_THRUST = 0.29
ATTITUDE_IGNORE = mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def main() -> None:
    cfg = raceconfig.MEASURED
    print(f">>> CONFIG {cfg.name}: v_max={cfg.v_max:.1f} climb_max={cfg.climb_max:.1f}")

    conn = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
    print("Waiting for heartbeat...", flush=True)
    conn.wait_heartbeat()
    print("Connected.", flush=True)

    stop = threading.Event()

    def heartbeat() -> None:
        while not stop.is_set():
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )
            stop.wait(0.5)

    threading.Thread(target=heartbeat, daemon=True).start()

    state = {
        "pos": None,
        "vel": None,
        "att": None,
        "active": 0,
        "finished": False,
        "rstart": 1,
        "got_rs": False,
        "collisions": 0,
    }

    def pump() -> None:
        while True:
            msg = conn.recv_match(blocking=False)
            if msg is None:
                return
            typ = msg.get_type()
            if typ == "LOCAL_POSITION_NED":
                state["pos"] = np.array([msg.x, msg.y, msg.z], dtype=float)
                state["vel"] = np.array([msg.vx, msg.vy, msg.vz], dtype=float)
            elif typ == "ATTITUDE":
                state["att"] = np.array([msg.roll, msg.pitch, msg.yaw], dtype=float)
            elif typ == "COLLISION":
                state["collisions"] += 1
            elif typ == "ENCAPSULATED_DATA":
                raw = bytes(msg.data)
                if raw and raw[0] == parsers.RACE_STATUS_ID:
                    rs = parsers.parse_race_status(raw)
                    state["active"] = rs.active_gate_index
                    state["finished"] = rs.finished
                    state["rstart"] = rs.race_start_boot_time_ms
                    state["got_rs"] = True

    def send_rates(rates, thrust: float) -> None:
        conn.mav.set_attitude_target_send(
            0,
            conn.target_system,
            conn.target_component,
            ATTITUDE_IGNORE,
            [1.0, 0.0, 0.0, 0.0],
            float(rates[0]),
            float(rates[1]),
            float(rates[2]),
            float(thrust),
        )

    def fly_velocity(vn_des: float, ve_des: float, target_z: float, vz_ff: float, yaw0: float, affn: float, affe: float) -> None:
        vel = state["vel"]
        att = state["att"]
        vn = clamp(vn_des, -cfg.vh_max, cfg.vh_max)
        ve = clamp(ve_des, -cfg.vh_max, cfg.vh_max)
        an = clamp(cfg.kph_vel * (vn - vel[0]) + affn, -cfg.ah_max, cfg.ah_max)
        ae = clamp(cfg.kph_vel * (ve - vel[1]) + affe, -cfg.ah_max, cfg.ah_max)
        roll_des, pitch_des = control.accel_to_tilt(an, ae, att[2], tilt_max=cfg.tilt_rad)
        rates = control.attitude_rate_command(att, [roll_des, pitch_des, yaw0], kp=cfg.kp_att, max_rate=cfg.max_rate)
        rates[0] = -rates[0]
        az = control.vertical_accel(target_z, state["pos"][2], vel[2], vz_ff=vz_ff, kpz_pos=cfg.kpz_pos, kpz_vel=cfg.kpz_vel)
        thrust = control.collective_thrust(az, att[0], att[1], hover=HOVER_THRUST)
        send_rates(rates, thrust)

    print("Waiting for fix; arming.", flush=True)
    while state["pos"] is None or state["att"] is None:
        pump()
        time.sleep(0.01)

    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    print(">>> ARMED. Restart the run at the start line, then start the countdown.", flush=True)
    while not (state["got_rs"] and state["rstart"] < 0 and state["active"] == 0 and abs(state["pos"][2]) < 10.0):
        pump()
        send_rates([0.0, 0.0, 0.0], 0.0)
        time.sleep(1 / 50)

    print(">>> Fresh start detected. Start the race.", flush=True)
    while state["rstart"] < 0:
        pump()
        send_rates([0.0, 0.0, 0.0], 0.0)
        time.sleep(1 / 50)

    countdown = time.time()
    while time.time() - countdown < 3.3:
        pump()
        send_rates([0.0, 0.0, 0.0], 0.0)
        time.sleep(1 / 50)

    print(">>> GO.", flush=True)
    yaw0 = float(state["att"][2])
    start_pos = state["pos"].copy()
    pts, cum = course.build_path(start_pos)
    vprof = course.build_profile(pts, cum, cfg)
    total = cum[-1]
    last_log = 0.0
    race_start = time.time()

    while time.time() - race_start < 150:
        pump()
        pos = state["pos"]
        s = raceline.project_arc(pts, cum, pos)
        v_set = raceline.speed_at(cum, vprof, s)
        nearest = raceline.point_at_arc(pts, cum, s)
        ahead = raceline.point_at_arc(pts, cum, s + cfg.look_ahead)
        delta = ahead - nearest
        horizontal = delta[:2]
        horizontal_len = float(np.linalg.norm(horizontal))
        tangent = horizontal / horizontal_len if horizontal_len > 1e-6 else np.zeros(2)
        slope = delta[2] / horizontal_len if horizontal_len > 1e-6 else 0.0
        velocity = v_set * tangent + cfg.kph_ct * (nearest[:2] - pos[:2])
        speed = float(np.linalg.norm(velocity))
        if speed > cfg.vh_max:
            velocity *= cfg.vh_max / speed
        curv = raceline.path_curvature_vector(pts, cum, s)
        aff = cfg.curv_ff * v_set * v_set * curv
        fly_velocity(velocity[0], velocity[1], nearest[2] - cfg.alt_bias, v_set * slope, yaw0, float(aff[0]), float(aff[1]))

        if state["finished"] or state["active"] >= len(course.GATES) or s >= total - 1.0:
            break

        now = time.time()
        if now - last_log > 0.75:
            xy_speed = float(np.linalg.norm(state["vel"][:2]))
            print(f"gate={state['active']} s={s:5.1f}/{total:.0f} speed={xy_speed:.1f}", flush=True)
            last_log = now
        time.sleep(1 / 50)

    elapsed = time.time() - race_start
    stop.set()
    send_rates([0.0, 0.0, 0.0], 0.0)
    print(f">>> result: gates={state['active']}/{len(course.GATES)} time={elapsed:.1f}s collisions={state['collisions']}", flush=True)


if __name__ == "__main__":
    main()
