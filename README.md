# AI-GP Round 1 Qualifier

This repo is a small, faithful extraction of the AI-GP Round 1 qualifying pilot.
It is the archived working `racecourse_smooth.py` path plus only the modules it
needs to fly the known six-gate Round 1 course.

Known result from the project docs:

- Config: `measured`
- Gates: `6/6`
- Lap time: about `24.7 s`
- Reference time beaten: `36 s`

Local verification run:

- Config: `measured`
- Gates: `6/6`
- Lap time: `24.2 s`
- Collisions: `0`

The simulator is noisy, so live runs can vary by several seconds. The important
first check is a clean `6/6` completion.

## What It Does

The pilot does not use a neural network. It is a compact racing controller:

1. Uses the known Round 1 gate centers.
2. Builds a smooth Catmull-Rom path through the gates.
3. Computes a speed profile from measured limits.
4. Tracks the path with a rate-control cascade.
5. Sends MAVLink body rates and collective thrust to the simulator.

This is useful for learning because the code is small enough to read, and each
piece maps to a concrete racing idea.

## First-Time Runbook

This setup assumes the AI-GP simulator runs on Windows and the Python pilot runs
on a Mac. The simulator sends MAVLink to localhost, so `relay.py` forwards packets
between the Windows machine and the Mac.

### 1. Find Your Mac IP

On the Mac:

```bash
ipconfig getifaddr en0
```

Use that IP in the Windows relay command below.

### 2. Start The Relay On Windows

Put `relay.py` on the Windows machine next to the simulator, then run:

```cmd
python relay.py --mac-ip <MAC_LAN_IP>
```

Example:

```cmd
python relay.py --mac-ip 192.168.1.235
```

Allow Python through Windows Firewall if prompted.

### 3. Install The Pilot On Mac

From this repo:

```bash
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install -e .
```

### 4. Launch The Pilot Before Starting The Race

```bash
python3 fly.py
```

Default config is `measured`, the qualifying config.

Optional later-progress config:

```bash
python3 fly.py track1
```

### 5. Start The Sim Run

The pilot intentionally waits at zero throttle until it sees a fresh race start.

Expected console flow:

```text
Waiting for heartbeat...
Connected.
Waiting for fix; arming.
>>> ARMED, throttle down. RESTART the run...
>>> Fresh state at start line. START THE RACE...
>>> GO.
```

Use this sequence:

1. Start `python3 fly.py`.
2. In the simulator, restart the run so the drone respawns at the start line.
3. Start the race countdown.
4. The pilot holds through the countdown, then flies.

## If The Drone Sits Still

Usually the start guard is doing its job. Restart the sim run after the pilot is
already listening, then start the countdown.

The pilot refuses to launch unless:

- race status is available,
- the race has not started yet,
- active gate is `0`,
- the drone is near ground level.

That prevents building a path from a crashed or stale position.

## Files

- `fly.py` is the live Round 1 qualifying runner.
- `relay.py` is the Windows localhost-to-LAN UDP bridge.
- `src/aigp_pilot/raceconfig.py` contains `measured` and `track1`.
- `src/aigp_pilot/course.py` contains the Round 1 gate centers.
- `src/aigp_pilot/control.py` contains the rate cascade.
- `src/aigp_pilot/raceline.py` contains path and speed-profile math.
- `src/aigp_pilot/telemetry.py` and `analyze.py` record and summarize runs.

## Quick Test

Without the simulator:

```bash
python3 -m pytest -q
```
