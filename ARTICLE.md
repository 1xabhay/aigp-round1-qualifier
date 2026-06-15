# The Small Pilot That Qualified

The Round 1 AI-GP pilot is interesting because the final working version is not
a giant model or a mysterious bag of tricks. It is a compact racing controller:
map the gates, draw a smooth line through them, assign a speed profile from the
measured limits of the simulator, then drive that line with a rate-based drone
controller.

The important lesson was that the simulator did not want position setpoints or
absolute attitudes. It wanted body rates and collective thrust. Once that was
clear, the pilot became much more like a race engineer’s tool: position error
became desired velocity, velocity error became acceleration, acceleration became
body tilt, and tilt became rate commands. A small altitude loop added the thrust
needed to hold the climbing course.

The qualifying configuration is deliberately plain. It uses the known Round 1
gate centers, a measured hover thrust, measured rate signs, a climb-aware speed
profile, and a guarded start sequence so the path is built from a fresh spawn
instead of a crashed or stale state. That was enough to fly all six gates cleanly
and beat the reference time.

For AI games, this is a useful kind of agent: not end-to-end magic, but measured
game physics turned into reliable autonomy. The code is small because the idea is
sharp.
