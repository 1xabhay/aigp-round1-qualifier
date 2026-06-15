# AI-GP Round 1 Qualifier

This is the small extraction of the working Round 1 qualifying pilot. It is not
the rebuild stack and not a new controller. It preserves the archived
`racecourse_smooth.py` path and the modules it needs to fly the known six-gate
Round 1 map.

Default config: `measured`

Known result from the project docs: clean Round 1 lap, 6/6 gates, about 24.7 s.

## Run

Use the simulator relay as before, then:

```bash
python3 -m pip install -e .
python3 fly.py
```

Optional config:

```bash
python3 fly.py track1
```

The runner waits for the same guarded start sequence:

1. Launch the pilot first.
2. Restart the run so the drone respawns at the start line.
3. Start the race countdown.

## Files

- `fly.py` is the qualifying live runner copied from `archive/scripts_v1/racecourse_smooth.py`.
- `src/aigp_pilot/raceconfig.py` contains the flown `measured` and `track1` numeric configs.
- `src/aigp_pilot/course.py` contains the Round 1 gate centers in flight-frame NED.
- `src/aigp_pilot/control.py`, `raceline.py`, `telemetry.py`, `parsers.py`, and `analyze.py` are the direct support modules.
