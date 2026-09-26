#!/usr/bin/env python3
"""Command-line client for the DUM-E arm and claw firmware (Python 3, standard library only).

Talks the same 64-byte frames as the ROS2 drivers, using the protocol extension
(shared/include/dume/ArmProtocol.h, ClawProtocol.h). By default it uses the tool port 81, so it
works while ROS2 is connected to port 80.

Examples:
  python dume_cli.py arm status --loop
  python dume_cli.py arm home            (home all)      python dume_cli.py arm home 2
  python dume_cli.py arm jog 1 -10       python dume_cli.py arm reset
  python dume_cli.py arm speedtest 4 --start 500 --max 8000 --inc 500
  python dume_cli.py claw close          python dume_cli.py claw force 400
  python dume_cli.py claw move a close 50 800
  python dume_cli.py claw status --loop
Use --host to override the default IPs (arm 192.168.212.203, claw 192.168.212.202).
"""

import argparse
import random
import socket
import struct
import sys
import time

FRAME = 64
EXT_MAGIC = 0xD5
EXT_VERSION = 1

# ------------------------------------------------------------------------------------------------
# Arm
# ------------------------------------------------------------------------------------------------
ARM_REQ = struct.Struct("<BBBB6i6B3i18x")
ARM_RESP = struct.Struct("<BBBB6i14Bi18x")
assert ARM_REQ.size == FRAME and ARM_RESP.size == FRAME

ARM_CMD = {"home": 1, "home_axis": 2, "reset": 3, "stop": 4, "speedtest": 5, "disable": 6,
           "sethome": 7, "jog": 8, "enable": 9}
ARM_STATE = ["DISABLED", "READY", "HOMING", "SPEEDTEST", "E-STOP"]
ARM_FAULTS = ["E-stop open", "E-stop latched", "endstop hit", "homing failed", "comm timeout",
              "PC e-stop", "waiting for setpoint sync"]
RESULT = ["-", "accepted", "REJECTED", "bad argument"]

# ------------------------------------------------------------------------------------------------
# Claw
# ------------------------------------------------------------------------------------------------
CLAW_REQ = struct.Struct("<5B5BbBHH48x")
CLAW_RESP = struct.Struct("<10B3B3B3h3hbB34x")
assert CLAW_REQ.size == FRAME and CLAW_RESP.size == FRAME

CLAW_CMD = {"open": 1, "close": 2, "force": 3, "move": 4, "stop": 5, "reset": 6, "cal": 7}
CLAW_STATE = ["idle", "open", "opening", "closing", "GRIPPED", "moving", "FAULT"]
FINGER_STATE = ["idle", "braked", "opening", "open", "closing", "GRIPPED", "moving", "FAULT",
                "disabled"]
FINGER_FAULT = ["", "overcurrent", "timeout", "no sensor", "e-stop"]


class Link:
    def __init__(self, host, port, timeout=2.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.msg = 0

    def exchange(self, frame: bytes) -> bytes:
        self.sock.sendall(frame)
        data = b""
        while len(data) < FRAME:
            chunk = self.sock.recv(FRAME - len(data))
            if not chunk:
                raise ConnectionError("connection closed by the controller")
            data += chunk
        return data

    def next_msg(self):
        self.msg = (self.msg + 1) & 0xFF
        return self.msg


def bits(mask, n=6):
    return "".join(str(i + 1) if mask & (1 << i) else "." for i in range(n))


# ------------------------------------------------------------------------------------------------
def arm_frame(link, cmd=0, arg=0, seq=0, params=(0, 0, 0), estop=0):
    return ARM_REQ.pack(link.next_msg(), estop, 0, 0, *([0] * 6),
                        EXT_MAGIC, EXT_VERSION, cmd, arg, seq, 0,  # extFlags 0: setpoints ignored
                        *params)


def arm_decode(data):
    v = ARM_RESP.unpack(data)
    r = dict(msg=v[0], error=v[1], active=v[3], pos=[p / 1000.0 for p in v[4:10]])
    (r["magic"], r["version"], r["state"], r["homed"], r["endstops"], r["endstop_enabled"],
     r["faults"], r["homing_axis"], r["homing_phase"], r["last_seq"], r["last_result"],
     r["drivers"], r["net_high"], r["st_axis"]) = v[10:24]
    r["st_speed"] = v[24]
    return r


def arm_print(r):
    state = ARM_STATE[r["state"]] if r["state"] < len(ARM_STATE) else r["state"]
    faults = [n for i, n in enumerate(ARM_FAULTS) if r["faults"] & (1 << i)]
    print(f"{state:9s} drivers {'on ' if r['drivers'] else 'off'} ENABLE net "
          f"{'HIGH' if r['net_high'] else 'LOW '}  homed {bits(r['homed'])}  "
          f"endstops {''.join('-' if not r['endstop_enabled'] & (1 << i) else ('X' if r['endstops'] & (1 << i) else '.') for i in range(6))}")
    print("   " + "  ".join(f"A{i + 1} {p:8.2f}" for i, p in enumerate(r["pos"])))
    if r["state"] == 2:
        print(f"   homing A{r['homing_axis']} phase {r['homing_phase']}")
    if r["st_axis"]:
        print(f"   speed test A{r['st_axis']} at {r['st_speed']} steps/s")
    if faults:
        print("   faults: " + ", ".join(faults))


def run_command(link, build, decode, seq_index="last_seq"):
    """Send a command with a fresh sequence number and wait until the controller reports it."""
    seq = random.randint(1, 255)
    for _ in range(50):
        # Re-sending the same sequence number polls without repeating the command.
        reply = decode(link.exchange(build(seq)))
        if reply[seq_index] == seq:
            res = reply["last_result"]
            return RESULT[res] if res < len(RESULT) else res
        time.sleep(0.02)
    return "no acknowledgement (command queue full?)"


def arm_main(args):
    link = Link(args.host or "192.168.212.203", args.port)
    c = args.cmd
    if c == "status":
        while True:
            arm_print(arm_decode(link.exchange(arm_frame(link))))
            if not args.loop:
                return
            time.sleep(args.interval)
            print()
    if c == "estop":
        link.exchange(arm_frame(link, estop=1))
        print("emergency stop sent")
        return

    axis, params = 0, (0, 0, 0)
    if c == "home":
        cmd = ARM_CMD["home_axis"] if args.axis else ARM_CMD["home"]
        axis = args.axis or 0
    elif c == "jog":
        cmd, axis, params = ARM_CMD["jog"], args.axis, (round(args.deg * 1000), 0, 0)
    elif c == "sethome":
        cmd = ARM_CMD["sethome"]
        axis = 0 if args.axis == "all" else int(args.axis)
        params = (round((args.deg or 0.0) * 1000), 0, 0)
    elif c == "speedtest":
        cmd, axis, params = ARM_CMD["speedtest"], args.axis, (args.start, args.max, args.inc)
    else:
        cmd = ARM_CMD[c]
    result = run_command(link, lambda seq: arm_frame(link, cmd, axis, seq, params), arm_decode)
    print(f"{c}: {result}")
    arm_print(arm_decode(link.exchange(arm_frame(link))))


# ------------------------------------------------------------------------------------------------
FINGERS = {"a": 1, "b": 2, "c": 4, "all": 0}


def claw_frame(link, cmd=0, seq=0, mask=0, arg=0, effort=0, duration=0, target=0, estop=0):
    return CLAW_REQ.pack(link.next_msg(), estop, 0, 0, 0,
                         EXT_MAGIC, EXT_VERSION, cmd, seq, mask, arg, effort, duration, target)


def claw_decode(data):
    v = CLAW_RESP.unpack(data)
    return dict(msg=v[0], error=v[1], active=v[3], gripper=v[4], magic=v[5], state=v[7],
                last_seq=v[8], last_result=v[9], finger_state=v[10:13], finger_fault=v[13:16],
                current=v[16:19], peak=v[19:22], rssi=v[22], flags=v[23])


def claw_print(r):
    state = CLAW_STATE[r["state"]] if r["state"] < len(CLAW_STATE) else r["state"]
    extra = []
    if r["flags"] & 1:
        extra.append("calibration")
    if r["flags"] & 2:
        extra.append("SENSOR FAULT")
    print(f"claw {state}  gripperState {r['gripper']}  RSSI {r['rssi']} dBm  {' '.join(extra)}")
    for i, name in enumerate("ABC"):
        fs = r["finger_state"][i]
        ff = r["finger_fault"][i]
        print(f"   {name}: {FINGER_STATE[fs] if fs < len(FINGER_STATE) else fs:8s} "
              f"{r['current'][i]:5d} mA (peak {r['peak'][i]:5d})  {FINGER_FAULT[ff] if ff < len(FINGER_FAULT) else ff}")


def claw_main(args):
    link = Link(args.host or "192.168.212.202", args.port)
    c = args.cmd
    if c == "status":
        while True:
            claw_print(claw_decode(link.exchange(claw_frame(link))))
            if not args.loop:
                return
            time.sleep(args.interval)
            print()
    if c == "estop":
        link.exchange(claw_frame(link, estop=1))
        print("emergency stop sent")
        return

    kw = {}
    if c in ("open", "close", "stop", "reset"):
        kw["mask"] = FINGERS[args.finger]
    elif c == "force":
        kw.update(mask=FINGERS[args.finger], target=args.milliamps)
    elif c == "move":
        kw.update(mask=FINGERS[args.finger], arg=1 if args.direction == "close" else -1,
                  effort=args.pct, duration=args.ms)
    elif c == "cal":
        kw["arg"] = 1 if args.onoff == "on" else 0
    cmd = CLAW_CMD[c]
    result = run_command(link, lambda seq: claw_frame(link, cmd, seq, **kw), claw_decode)
    print(f"{c}: {result}")
    claw_print(claw_decode(link.exchange(claw_frame(link))))


# ------------------------------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", help="IP address (default: arm .203 / claw .202)")
    p.add_argument("--port", type=int, default=81, help="81 = tool port (default), 80 = ROS2 port")
    boards = p.add_subparsers(dest="board", required=True)

    arm = boards.add_parser("arm").add_subparsers(dest="cmd", required=True)
    s = arm.add_parser("status")
    s.add_argument("--loop", action="store_true")
    s.add_argument("--interval", type=float, default=0.5)
    arm.add_parser("home").add_argument("axis", type=int, nargs="?")
    j = arm.add_parser("jog")
    j.add_argument("axis", type=int)
    j.add_argument("deg", type=float)
    sh = arm.add_parser("sethome", help="sethome <axis> <deg> | sethome all")
    sh.add_argument("axis")
    sh.add_argument("deg", type=float, nargs="?")
    st = arm.add_parser("speedtest")
    st.add_argument("axis", type=int)
    st.add_argument("--start", type=int, default=0)
    st.add_argument("--max", type=int, default=0)
    st.add_argument("--inc", type=int, default=0)
    for name in ("reset", "stop", "enable", "disable", "estop"):
        arm.add_parser(name)

    claw = boards.add_parser("claw").add_subparsers(dest="cmd", required=True)
    s = claw.add_parser("status")
    s.add_argument("--loop", action="store_true")
    s.add_argument("--interval", type=float, default=0.5)
    for name in ("open", "close", "stop", "reset"):
        claw.add_parser(name).add_argument("finger", nargs="?", default="all", choices=FINGERS)
    f = claw.add_parser("force")
    f.add_argument("milliamps", type=int)
    f.add_argument("finger", nargs="?", default="all", choices=FINGERS)
    m = claw.add_parser("move")
    m.add_argument("finger", choices=FINGERS)
    m.add_argument("direction", choices=("close", "open"))
    m.add_argument("pct", type=int)
    m.add_argument("ms", type=int)
    claw.add_parser("cal").add_argument("onoff", choices=("on", "off"))
    claw.add_parser("estop")

    args = p.parse_args()
    try:
        (arm_main if args.board == "arm" else claw_main)(args)
    except KeyboardInterrupt:
        pass
    except OSError as e:
        sys.exit(f"connection problem: {e}")


if __name__ == "__main__":
    main()
