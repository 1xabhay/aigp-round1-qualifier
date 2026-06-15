"""
UDP relay — RUN THIS ON THE WINDOWS PC (where FlightSim.exe runs).

Why it exists: the simulator only ever sends to 127.0.0.1 (localhost) and has
no IP setting. To develop on the Mac (Option C) we drop this relay in the
middle. It pretends to be the pilot (binds the localhost ports the sim sends
to), forwards everything across the LAN to the Mac, and forwards the Mac's
commands back to the sim.

    WINDOWS                                         MAC
    FlightSim.exe --localhost--> relay.py --LAN--> 192.168.1.235
                  <--commands--          <--------

It needs NOTHING installed — only the Python standard library. Run it in a
terminal on Windows ALONGSIDE the simulator:

    python relay.py --mac-ip 192.168.1.235

Leave it running, then start the flight in the sim. Ctrl-C to stop.

How the two-way MAVLink routing works:
  - The sim sends telemetry to 127.0.0.1:14550. We bind that port, so we get
    it, and we remember the sim's source address.
  - We forward telemetry to the Mac:14550 using a second socket.
  - The Mac replies (commands/heartbeats) to that second socket.
  - We forward those back to the remembered sim address, sent FROM port 14550
    so the sim sees them coming from where it expects its ground station.
Vision (5600) is one-way, so it's just receive-and-forward.
"""

from __future__ import annotations

import argparse
import select
import socket


def make_bound(port: int) -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", port))
    return s


def make_sender() -> socket.socket:
    return socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def main():
    ap = argparse.ArgumentParser(description="AI-GP localhost<->LAN UDP relay (run on Windows)")
    ap.add_argument("--mac-ip", required=True, help="the Mac's LAN IP, e.g. 192.168.1.235")
    ap.add_argument("--mavlink-port", type=int, default=14550, help="local port the sim sends MAVLink to")
    ap.add_argument("--vision-port", type=int, default=5600, help="local port the sim sends vision to")
    # Destination ports on the Mac. Default to the same ports (real two-machine
    # setup); only differ for single-machine loopback testing.
    ap.add_argument("--mac-mavlink-port", type=int, default=None)
    ap.add_argument("--mac-vision-port", type=int, default=None)
    args = ap.parse_args()

    mac = args.mac_ip
    mav_port = args.mavlink_port
    vid_port = args.vision_port
    mac_mav_port = args.mac_mavlink_port if args.mac_mavlink_port is not None else mav_port
    mac_vid_port = args.mac_vision_port if args.mac_vision_port is not None else vid_port

    # --- MAVLink (two-way) ---
    mav_from_sim = make_bound(mav_port)    # sim sends telemetry here (localhost)
    mav_to_mac = make_sender()             # we talk to the Mac through this one
    # --- Vision (one-way: sim -> Mac) ---
    vid_from_sim = make_bound(vid_port)
    vid_to_mac = make_sender()

    sim_mav_addr = None  # learned from the first sim packet; where commands go back
    counts = {"tel": 0, "cmd": 0, "vid": 0}

    print("Relay running on Windows.")
    print(f"  MAVLink: localhost:{mav_port}  <-->  {mac}:{mav_port}")
    print(f"  Vision : localhost:{vid_port}   -->  {mac}:{vid_port}")
    print("  Start the flight in the sim now. Ctrl-C to stop.\n", flush=True)

    socks = [mav_from_sim, mav_to_mac, vid_from_sim]
    try:
        while True:
            readable, _, _ = select.select(socks, [], [], 1.0)
            for s in readable:
                if s is mav_from_sim:
                    data, addr = s.recvfrom(65536)
                    sim_mav_addr = addr  # remember where to send commands back
                    mav_to_mac.sendto(data, (mac, mac_mav_port))
                    counts["tel"] += 1
                elif s is mav_to_mac:
                    data, _ = s.recvfrom(65536)
                    if sim_mav_addr is not None:
                        # send back to the sim FROM the bound :14550 socket
                        mav_from_sim.sendto(data, sim_mav_addr)
                        counts["cmd"] += 1
                elif s is vid_from_sim:
                    data, _ = s.recvfrom(65536)
                    vid_to_mac.sendto(data, (mac, mac_vid_port))
                    counts["vid"] += 1
            # heartbeat-ish status line so you can see it's alive
            print(
                "\r  telemetry->mac: %-8d  commands->sim: %-8d  vision->mac: %-8d"
                % (counts["tel"], counts["cmd"], counts["vid"]),
                end="", flush=True,
            )
    except KeyboardInterrupt:
        print("\nRelay stopped.")


if __name__ == "__main__":
    main()
