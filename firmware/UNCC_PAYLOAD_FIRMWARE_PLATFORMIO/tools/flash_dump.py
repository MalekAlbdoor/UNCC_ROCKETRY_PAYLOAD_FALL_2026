#!/usr/bin/env python3
"""Pull the flight log off the payload's flash over USB and turn it into CSV.

    pip install pyserial
    python tools/flash_dump.py COM5              # dump -> flight_log.bin + flight_log.csv
    python tools/flash_dump.py COM5 --erase      # dump, then ask the payload to erase its flash
    python tools/flash_dump.py --decode old.bin  # just convert a .bin saved earlier

The payload only dumps or erases while it sits on the ground (PAD or LANDED).
Each power-up starts a new session in the log; the CSV has a `session` column that counts them.
"""
import argparse, csv, struct, sys, time, zlib

REC_FMT = "<BBHI3h3h3hHhBihHHIhHBBiihhHhhhHHH"     # must match struct Telem in main.cpp
REC = struct.calcsize(REC_FMT)
assert REC == 77
PAGE, PER_PAGE = 256, 3
FIELDS = ["magic", "flags", "seq", "ms", "acc_x", "acc_y", "acc_z", "gyr_x", "gyr_y", "gyr_z",
          "hig_x", "hig_y", "hig_z", "peak", "temp", "state", "alt", "vel", "maxAlt", "maxVel",
          "pres", "airT", "rh", "fix", "sats", "lat", "lon", "gpsAlt", "gpsVup", "gpsSpd",
          "roll", "pitch", "yaw", "vbat", "heading", "magField"]
STATES = ["PAD", "BOOST", "COAST", "DESCENT", "LANDED"]


def dump(port, baud):
    import serial
    with serial.Serial(port, baud, timeout=5) as s:
        s.reset_input_buffer()
        s.write(b"d")
        while True:                                  # skip the live status lines until the header
            line = s.readline()
            if not line:
                sys.exit("no answer from the payload (is it on the ground, and is the port right?)")
            if line.startswith(b"DUMP "):
                n = int(line.split()[1])
                break
            if b"refusing" in line or b"NOT FOUND" in line:
                sys.exit(line.decode(errors="replace").strip())
        data = bytearray()
        t0 = time.time()
        while len(data) < n:
            chunk = s.read(min(65536, n - len(data)))
            if not chunk:
                sys.exit(f"timed out after {len(data)} of {n} bytes")
            data += chunk
            print(f"\r{len(data) / 1024:.0f} / {n / 1024:.0f} KB", end="", file=sys.stderr)
        print(file=sys.stderr)
        tail = s.read_until(b"\n") + s.read_until(b"\n")
        want = None
        for part in tail.split(b"\n"):
            if part.startswith(b"CRC "):
                want = int(part.split()[1], 16)
        got = zlib.crc32(bytes(data)) & 0xFFFFFFFF
        if want is None or want != got:
            print(f"WARNING: checksum mismatch (payload {want}, received {got:08x}); the dump may be damaged",
                  file=sys.stderr)
        else:
            print(f"checksum ok, {n} bytes in {time.time() - t0:.1f} s", file=sys.stderr)
        return bytes(data)


def erase(port, baud):
    import serial
    with serial.Serial(port, baud, timeout=2) as s:
        s.write(b"E")
        time.sleep(0.3)
        s.write(b"Y")
        s.timeout = 600
        while True:
            line = s.readline()
            if not line or b"done" in line or b"timed out" in line or b"refusing" in line:
                print(line.decode(errors="replace").strip())
                break


def decode(data):
    rows, session, last_ms = [], 0, None
    for p in range(0, len(data) - PAGE + 1, PAGE):
        for i in range(PER_PAGE):
            raw = data[p + i * REC: p + (i + 1) * REC]
            if raw[0] != ord("X"):
                continue
            v = struct.unpack(REC_FMT, raw)
            row = dict(zip(FIELDS, v))
            if last_ms is not None and row["ms"] < last_ms:
                session += 1                         # the clock restarted: a new power-up
            last_ms = row["ms"]
            row["session"] = session
            row["state"] = STATES[row["state"]] if row["state"] < len(STATES) else row["state"]
            row["alt_m"] = row["alt"] / 100
            row["vel_ms"] = row["vel"] / 10
            row["acc_g"] = (row["acc_x"] ** 2 + row["acc_y"] ** 2 + row["acc_z"] ** 2) ** 0.5 * 0.000976
            row["pres_hPa"] = row["pres"] / 100
            row["lat_deg"] = row["lat"] / 1e7
            row["lon_deg"] = row["lon"] / 1e7
            row["vbat_V"] = row["vbat"] / 1000
            rows.append(row)
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", help="serial port of the payload, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", default="flight_log", help="output name without extension")
    ap.add_argument("--erase", action="store_true", help="erase the payload's flash after a good dump")
    ap.add_argument("--decode", metavar="BIN", help="convert an existing .bin instead of reading the payload")
    a = ap.parse_args()

    if a.decode:
        data = open(a.decode, "rb").read()
    elif a.port:
        data = dump(a.port, a.baud)
        open(a.out + ".bin", "wb").write(data)
    else:
        ap.error("give a serial port or --decode")

    rows = decode(data)
    if rows:
        cols = list(rows[0].keys())
        with open(a.out + ".csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            w.writerows(rows)
    print(f"{len(rows)} records, {rows[-1]['session'] + 1 if rows else 0} session(s) -> {a.out}.csv")

    if a.erase and a.port and not a.decode:
        if rows or len(data) == 0:
            erase(a.port, a.baud)


if __name__ == "__main__":
    main()
