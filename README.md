# AI-GP Round 1 Qualifier

| config | gates | time | collisions |
|---|---:|---:|---:|
| `measured` | `6/6` | `24.0 s` | `0` |

## Run

```bash
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install -e .
python3 fly.py
```

Start the simulator run after the pilot is waiting.

## Files

- `fly.py`
- `src/aigp_pilot/raceconfig.py`
- `src/aigp_pilot/course.py`
- `src/aigp_pilot/raceline.py`
- `src/aigp_pilot/control.py`
- `src/aigp_pilot/parsers.py`

## Test

```bash
python3 -m pytest -q
```
