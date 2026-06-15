"""
Config-driven race-line follower. Flies a named RaceConfig (presets in
aigp_pilot.raceconfig) over the Round-1 course.

  path  = smooth Catmull-Rom spline through the gate centres (course.build_path)
  speed = physics time-optimal profile v(s) (course.build_profile / raceline)
  track = drive v(s) along the path tangent + cross-track correction onto the
          centreline + vertical feedforward (climb rate) so altitude never lags
  rate cascade: accel -> body-frame tilt -> attitude rate loop -> body rates ;
                thrust = altitude hold.  FORWARD-ONLY (no retreat).

    uv run python racecourse_smooth.py [config]    # default: v9
    (launch FIRST, then ONE restart at the start line, then start the race)
"""
from __future__ import annotations

import math
import os
import sys
import threading
import time

import numpy as np
from pymavlink import mavutil

from aigp_pilot import (analyze, control, course, fusion, parsers, raceconfig,
                        raceline, telemetry, vision_truth)
from aigp_pilot.raceline import point_at_arc, project_arc, speed_at

G = 9.81
HOVER = 0.29
ATT_IGNORE = mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def main():
    cfg_name = sys.argv[1] if len(sys.argv) > 1 else "measured"  # best clean/reliable lap
    cfg = raceconfig.get(cfg_name)
    vision_mode = sys.argv[2] if len(sys.argv) > 2 else None  # none / shadow(log) / fuse(correct)
    if vision_mode == "none":
        vision_mode = None
    gate_off = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0   # A/B: inject a wrong map (Y m)
    GATES = [g + np.array([0.0, gate_off, 0.0]) for g in course.GATES]  # the pilot's BELIEVED map
    VISION_GAIN, VISION_MAX_AGE, CORR_MAX = 0.8, 0.3, 2.5      # trust fraction, freshness, clamp(m)
    worker = None
    if vision_mode in ("shadow", "fuse"):
        from aigp_pilot.vision_thread import VisionWorker
        worker = VisionWorker()
        worker.start()
        print(">>> VISION %s mode%s" % (
            vision_mode, ": correcting target toward seen gate (clamped %.1fm)" % CORR_MAX
            if vision_mode == "fuse" else ": logging divergence, NOT steering"), flush=True)
    if gate_off:
        print(">>> INJECTED MAP ERROR: gates shifted %.1f m in Y — pilot believes the WRONG map"
              % gate_off, flush=True)
    # unpack into locals so the nested control closures use THIS config
    KPZ_POS, KPZ_VEL = cfg.kpz_pos, cfg.kpz_vel
    KPH_CT, KPH_VEL = cfg.kph_ct, cfg.kph_vel
    VH_MAX, AH_MAX = cfg.vh_max, cfg.ah_max
    TILT_DES_MAX = cfg.tilt_rad
    KP_ATT, MAX_RATE = cfg.kp_att, cfg.max_rate
    LOOK_AHEAD, ALT_BIAS = cfg.look_ahead, cfg.alt_bias
    CURV_FF = cfg.curv_ff
    print(">>> CONFIG %s: v_max=%.1f kph_ct=%.1f kph_vel=%.1f max_rate=%.1f tilt=%.0f"
          % (cfg.name, cfg.v_max, cfg.kph_ct, cfg.kph_vel, cfg.max_rate, cfg.tilt_deg),
          flush=True)
    conn = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
    print("Waiting for heartbeat...", flush=True)
    conn.wait_heartbeat()
    print("Connected.", flush=True)
    stop = threading.Event()

    def hb():
        while not stop.is_set():
            conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            stop.wait(0.5)
    threading.Thread(target=hb, daemon=True).start()

    st = {"pos": None, "vel": None, "att": None, "active": 0, "finished": False,
          "rstart": 1, "got_rs": False, "thr": 0.0}
    logger = telemetry.RunLogger(cfg.name)
    race_t0 = [0.0]   # set at GO; mutable so pump() can stamp collisions

    def pump():
        while True:
            m = conn.recv_match(blocking=False)
            if m is None:
                return
            t = m.get_type()
            if t == "LOCAL_POSITION_NED":
                st["pos"] = np.array([m.x, m.y, m.z]); st["vel"] = np.array([m.vx, m.vy, m.vz])
            elif t == "ATTITUDE":
                st["att"] = np.array([m.roll, m.pitch, m.yaw])
            elif t == "COLLISION":
                tc = time.time() - race_t0[0]
                logger.collision(tc, getattr(m, "id", -1), getattr(m, "threat_level", -1))
                print("!!! COLLISION t=%.1f id=%s threat=%s s=?"
                      % (tc, getattr(m, "id", -1), getattr(m, "threat_level", -1)), flush=True)
            elif t == "ENCAPSULATED_DATA":
                raw = bytes(m.data)
                if raw and parsers.encapsulated_msg_id(raw) == parsers.RACE_STATUS_ID:
                    rs = parsers.parse_race_status(raw)
                    st["active"] = rs.active_gate_index
                    st["finished"] = rs.finished
                    st["rstart"] = rs.race_start_boot_time_ms
                    st["got_rs"] = True

    def send_rates(rates, thr):
        conn.mav.set_attitude_target_send(0, conn.target_system, conn.target_component,
                                          ATT_IGNORE, [1.0, 0, 0, 0],
                                          float(rates[0]), float(rates[1]), float(rates[2]), float(thr))

    def thr_hold(tz, vz_ff=0.0):
        # cascaded altitude loop + tilt-compensated collective (pure, tested in control.py)
        az = control.vertical_accel(tz, st["pos"][2], st["vel"][2], vz_ff=vz_ff,
                                    kpz_pos=KPZ_POS, kpz_vel=KPZ_VEL)
        roll, pitch = st["att"][0], st["att"][1]
        return control.collective_thrust(az, roll, pitch, hover=HOVER)

    def fly_vel(vN_des, vE_des, target_z, vz_ff, yaw0, affN=0.0, affE=0.0):
        vel = st["vel"]
        vN = clamp(vN_des, -VH_MAX, VH_MAX)
        vE = clamp(vE_des, -VH_MAX, VH_MAX)
        # velocity-tracking accel + curvature feedforward (affN/affE = v^2*kappa)
        aN = clamp(KPH_VEL * (vN - vel[0]) + affN, -AH_MAX, AH_MAX)
        aE = clamp(KPH_VEL * (vE - vel[1]) + affE, -AH_MAX, AH_MAX)
        # world accel -> body-frame tilt (heading-robust; pure, tested in control.py)
        roll_des, pitch_des = control.accel_to_tilt(aN, aE, st["att"][2], tilt_max=TILT_DES_MAX)
        des = np.array([roll_des, pitch_des, yaw0])
        rates = control.attitude_rate_command(st["att"], des, kp=KP_ATT, max_rate=MAX_RATE)
        rates[0] = -rates[0]
        thr = thr_hold(target_z, vz_ff)
        st["thr"] = thr
        send_rates(rates, thr)

    print("Waiting for fix; arming.", flush=True)
    while st["pos"] is None or st["att"] is None:
        pump(); time.sleep(0.01)
    conn.mav.command_long_send(conn.target_system, conn.target_component,
                               mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    print(">>> ARMED, throttle down. RESTART the run (drone must respawn AT the start line)...", flush=True)
    # require a genuinely fresh state: race not started, no gates passed, and the
    # drone actually near the ground (else we'd build a path from a crashed origin).
    while not (st["got_rs"] and st["rstart"] < 0 and st["active"] == 0
               and abs(st["pos"][2]) < 10.0):
        pump(); send_rates([0, 0, 0], 0.0); time.sleep(1 / 50)
    print(">>> Fresh state at start line. START THE RACE (3-2-1) NOW...", flush=True)
    while st["rstart"] < 0:
        pump(); send_rates([0, 0, 0], 0.0); time.sleep(1 / 50)
    flip = time.time()
    while time.time() - flip < 3.3:
        pump(); send_rates([0, 0, 0], 0.0); time.sleep(1 / 50)
    print(">>> GO.", flush=True)
    yaw0 = st["att"][2]
    race_t0[0] = time.time()

    start_pos = st["pos"].copy()
    pts, cum = course.build_path(start_pos, cfg, gates=GATES)   # path from BELIEVED map
    total = cum[-1]
    vprof = course.build_profile(pts, cum, cfg)           # the physics-derived racing speed v(s)
    n_gates = len(GATES)
    print(">>> speed profile: min %.1f  max %.1f m/s over %.0f m"
          % (min(vprof), max(vprof), total), flush=True)
    # Per-gate map estimator: vision corrects each gate's position; we REBUILD the
    # spline through the corrected gates (a correctly-shaped line, no wedging).
    gate_est = fusion.GateMapEstimator(GATES, alpha=0.15, clamp=CORR_MAX)
    last = 0.0
    ticks = 0
    t_race = time.time()
    while time.time() - t_race < 150:
        pump()
        ticks += 1
        pos = st["pos"]
        # VISION: update the active gate's estimate; rebuild the path through the
        # corrected gates a few times a second (only in 'fuse'; shadow just logs).
        if worker is not None:
            ob = worker.latest()
            gate_est.update(st["active"], pos, st["att"][2], ob,
                            (time.time() - ob.wall) if ob else 1e9,
                            max_age=VISION_MAX_AGE, gain=1.0)   # full lateral -> corrected gate
            if vision_mode == "fuse" and ticks % 6 == 0 and gate_est.max_offset() > 0.05:
                pts, cum = course.build_path(start_pos, cfg, gates=gate_est.corrected())
                vprof = course.build_profile(pts, cum, cfg)
                total = cum[-1]
        s = project_arc(pts, cum, pos)
        v_set = speed_at(cum, vprof, s)                   # physics speed profile at our position
        nearest = point_at_arc(pts, cum, s)               # closest point on the (corrected) line
        ahead = point_at_arc(pts, cum, s + LOOK_AHEAD)
        d3 = ahead - nearest                              # 3D path direction over the lead
        dh = d3[:2]
        Lh = float(np.linalg.norm(dh))
        tan = dh / Lh if Lh > 1e-6 else np.zeros(2)       # horizontal tangent (unit)
        slope = d3[2] / Lh if Lh > 1e-6 else 0.0          # vertical per horizontal (NED)
        ct = nearest[:2] - pos[:2]                        # cross-track error -> back to the line
        v = v_set * tan + KPH_CT * ct                     # profile speed + cross-track correction
        vmag = float(np.linalg.norm(v))
        if vmag > VH_MAX:
            v = v * (VH_MAX / vmag)
        vz_ff = v_set * slope                             # match the climb rate (no altitude lag)
        target_z = nearest[2] - ALT_BIAS                  # hold the path altitude AT the gate
        # curvature feedforward: centripetal accel v^2*kappa to pre-turn into bends
        affN = affE = 0.0
        if CURV_FF > 0.0:
            cv = raceline.path_curvature_vector(pts, cum, s)
            affN = CURV_FF * v_set * v_set * float(cv[0])
            affE = CURV_FF * v_set * v_set * float(cv[1])
        fly_vel(v[0], v[1], target_z, vz_ff, yaw0, affN, affE)
        now = time.time()
        # instrument: one Sample per tick (measured car data)
        a = st["att"]
        logger.record(telemetry.Sample(
            t=now - race_t0[0], x=pos[0], y=pos[1], z=pos[2],
            vx=st["vel"][0], vy=st["vel"][1], vz=st["vel"][2],
            roll=a[0], pitch=a[1], yaw=a[2], thr=st["thr"],
            v_set=float(v_set), s=float(s), active=int(st["active"]), wall=now))
        if st["finished"] or st["active"] >= n_gates or s >= total - 1.0:
            print(">>> FINISHED / reached path end. active=%d s=%.1f/%.1f"
                  % (st["active"], s, total), flush=True)
            break
        if now - last > 0.5:
            spd = float(np.linalg.norm(st["vel"][:2]))
            line = ("active=%d s=%5.1f/%.0f pos=(%.1f,%.1f,%.1f) v_set=%.1f spd=%.1f ctrl=%.0fHz"
                    % (st["active"], s, total, pos[0], pos[1], pos[2], v_set, spd,
                       ticks / max(now - t_race, 1e-3)))
            if worker is not None:
                # SHADOW: what vision sees vs what the map says — log the divergence, don't steer
                obs = worker.latest()
                tyaw, tpitch, trng = vision_truth.active_gate_truth(
                    pos, st["att"][2], GATES, st["active"])
                if obs is not None and obs.detected:
                    age = now - obs.wall
                    line += ("  | VIS fps=%.1f age=%.2fs gates=%d  yaw v/m=%+.0f/%+.0f deg(div %+.0f)"
                             "  rng v/m=%.0f/%.0f m" % (
                                 worker.fps(), age, obs.n_gates,
                                 math.degrees(obs.yaw_off), math.degrees(tyaw),
                                 math.degrees(obs.yaw_off - tyaw), obs.range_m, trng))
                else:
                    line += "  | VIS fps=%.1f no-detection" % worker.fps()
                line += " mapfix=%.2fm%s" % (gate_est.max_offset(),
                                             "(REBUILT)" if vision_mode == "fuse" else "")
            print(line, flush=True)
            last = now
        time.sleep(1 / 50)

    elapsed = time.time() - t_race
    stop.set(); send_rates([0, 0, 0], 0.0)
    if worker is not None:
        worker.stop()
        st_v = worker.stats()
        print(">>> VISION shadow: %d usable frames (~%.1f fps): %d complete + %d partial-top"
              % (worker.frames, worker.fps(), worker.completes, worker.partials), flush=True)
        print(">>> STREAM HEALTH: frames started=%d completed=%d (%.0f%% complete)  "
              "chunks/frame=%.1f  est chunk-loss=%.0f%%" % (
                  st_v["started"], st_v["completed"], 100 * st_v["completion_rate"],
                  st_v["avg_chunks_per_frame"], 100 * st_v["est_chunk_loss"]), flush=True)
        print(">>> STREAM DIAG: peak_pending=%d  dropped=%d (avg %.0f%% complete when dropped)  "
              "dup=%d stale=%d  rcvbuf=%.1fMB" % (
                  st_v["peak_pending"], st_v["frames_dropped"],
                  100 * st_v["avg_evicted_completeness"], st_v["duplicate_chunks"],
                  st_v.get("stale_chunks", 0), worker.rcvbuf_effective / 1e6), flush=True)
    # instrument: write the full-rate telemetry + measure the car's real limits
    limits = analyze.measured_limits(logger.samples)
    csv_path = "runs/%s_%dgates.csv" % (cfg.name, st["active"])
    try:
        os.makedirs("runs", exist_ok=True)
        logger.write_csv(csv_path)
        os.makedirs("configs", exist_ok=True)
        with open("configs/runlog.md", "a") as f:
            f.write("- config=%s gates=%d/%d elapsed=%.1fs finished=%s collisions=%d "
                    "MEASURED v_top=%.1f a_lon=%.1f a_brk=%.1f a_lat=%.1f climb=%.1f -> %s\n"
                    % (cfg.name, st["active"], n_gates, elapsed, st["finished"],
                       len(logger.collisions), limits["v_top"], limits["a_lon"],
                       limits["a_brk"], limits["a_lat"], limits["climb"], csv_path))
    except OSError:
        pass
    print(">>> %s: gates=%d/%d  elapsed=%.1fs  collisions=%d" % (
        cfg.name, st["active"], n_gates, elapsed, len(logger.collisions)), flush=True)
    print(">>> MEASURED CAR LIMITS: v_top=%.1f a_lon=%.1f a_brk=%.1f a_lat=%.1f climb=%.1f m/s(^2)"
          % (limits["v_top"], limits["a_lon"], limits["a_brk"], limits["a_lat"], limits["climb"]),
          flush=True)
    print(">>> telemetry -> %s  (analyse with analyze.measured_limits)" % csv_path, flush=True)
    print("Done.", flush=True)


if __name__ == "__main__":
    main()
