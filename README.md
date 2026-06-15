# AI-GP Round 1 Qualifier

An autonomous drone-racing pilot for the AI-GP Round 1 simulator.

This repository is an exhibition cut: the smallest useful version of the code
that flew the six-gate qualifier. It keeps the working flight runner, the numeric
race configuration, the course model, and the physics-based controller.

## Result

Verified local run:

| config | gates | time | collisions |
|---|---:|---:|---:|
| `measured` | `6/6` | `24.2 s` | `0` |

Project-note baseline: about `24.7 s`

## Why It Is Interesting

This is not an end-to-end neural pilot. It is a compact game-agent stack built
from measured simulator physics:

1. Hard-code the known Round 1 gate centers.
2. Build a smooth spline through the gates.
3. Compute a speed profile from measured limits.
4. Track the path with a rate-control cascade.
5. Send MAVLink body rates and collective thrust to the simulator.

That makes the project readable: each file maps to one racing idea.

## Run It

The original setup used two machines:

- Windows: AI-GP simulator.
- Mac: Python pilot.

The simulator sends MAVLink only to localhost, so `relay.py` runs on Windows and
bridges packets to the Mac.

### Windows: Start The Relay

Copy `relay.py` to the Windows simulator machine, then run:

```cmd
python relay.py --mac-ip <MAC_LAN_IP>
```

Example:

```cmd
python relay.py --mac-ip 192.168.1.235
```

Allow Python through Windows Firewall.

### Mac: Start The Pilot

Find the Mac IP:

```bash
ipconfig getifaddr en0
```

Install and run:

```bash
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install -e .
python3 fly.py
```

Then use the simulator:

1. Restart the run so the drone respawns at the start line.
2. Start the race countdown.
3. The pilot holds through countdown and launches at `GO`.

Expected pilot log:

```text
Waiting for heartbeat...
Connected.
Waiting for fix; arming.
>>> ARMED, throttle down. RESTART the run...
>>> Fresh state at start line. START THE RACE...
>>> GO.
```

## Three Engine Lessons

- The sim is an acro/rate interface. The winning command is body rates plus
  collective thrust, not position setpoints.
- Frame choice matters. Position and velocity must come from `LOCAL_POSITION_NED`;
  the wrong velocity frame breaks altitude control.
- Physics beats guessing. Measure hover thrust, rate signs, speed, climb, and gate
  geometry, then build the controller around those numbers.

## Code Map

- `fly.py`: live flight runner and start guard.
- `relay.py`: Windows localhost-to-LAN MAVLink bridge.
- `src/aigp_pilot/raceconfig.py`: numeric race configs.
- `src/aigp_pilot/course.py`: Round 1 gate centers.
- `src/aigp_pilot/raceline.py`: spline path and speed profile.
- `src/aigp_pilot/control.py`: acceleration-to-rate control cascade.
- `src/aigp_pilot/telemetry.py`: run logging.

## Test

```bash
python3 -m pytest -q
```
