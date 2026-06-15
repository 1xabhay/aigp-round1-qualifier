# AI-GP Round 1 Qualifier

Small extraction of the pilot that qualified on the AI-GP Round 1 six-gate
course. It is intentionally simple: known gate map, measured physics, spline
path, speed profile, rate controller.

Verified run:

- Config: `measured`
- Gates: `6/6`
- Time: `24.2 s`
- Collisions: `0`

Documented baseline from the project notes: about `24.7 s`, beating the `36 s`
reference.

## How I Ran It

The simulator ran on Windows. The pilot ran on my Mac. Because the simulator only
sends MAVLink to `127.0.0.1`, I used `relay.py` on Windows to forward simulator
telemetry to the Mac and send commands back.

### Windows

Put `relay.py` next to the simulator and run:

```cmd
python relay.py --mac-ip <MAC_LAN_IP>
```

Example:

```cmd
python relay.py --mac-ip 192.168.1.235
```

Allow Python through Windows Firewall.

### Mac

Find the Mac IP:

```bash
ipconfig getifaddr en0
```

Install and start the pilot:

```bash
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install -e .
python3 fly.py
```

Then, in the simulator:

1. Restart the run so the drone respawns at the start line.
2. Start the race countdown.
3. The pilot holds through the countdown, then flies.

Expected console:

```text
Waiting for heartbeat...
Connected.
Waiting for fix; arming.
>>> ARMED, throttle down. RESTART the run...
>>> Fresh state at start line. START THE RACE...
>>> GO.
```

## Three Lessons From The Engine

1. The sim is an acro/rate interface: the useful command is body rates plus
   collective thrust, not position setpoints.
2. Correct telemetry matters: position and velocity come from `LOCAL_POSITION_NED`;
   using the wrong frame makes altitude and control feel haunted.
3. A strong game agent can be physics-first: measure hover, signs, speed, climb,
   and gate geometry, then build a controller around those numbers.

## What To Read

- `fly.py`: live Round 1 runner.
- `src/aigp_pilot/raceconfig.py`: numeric configs, including `measured`.
- `src/aigp_pilot/course.py`: Round 1 gate centers.
- `src/aigp_pilot/control.py`: rate-control cascade.
- `src/aigp_pilot/raceline.py`: path and speed profile.

Quick smoke test:

```bash
python3 -m pytest -q
```
