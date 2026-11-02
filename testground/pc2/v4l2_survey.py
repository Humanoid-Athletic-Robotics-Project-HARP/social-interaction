"""List what V4L2 video devices offer: formats, frame sizes and frame rates. Query
only: it opens each device without streaming, so it doesn't disturb a program
already reading one (Unitree's videohub reads the head camera's colour node).

  python3 v4l2_survey.py /dev/video0 /dev/video2
"""

import fcntl
import os
import struct
import sys

QUERYCAP, ENUM_FMT, ENUM_SIZES, ENUM_RATES = 0x80685600, 0xC0405602, 0xC02C564A, 0xC034564B
CAPTURE, META = 1, 13   # V4L2 buffer types


def ioctl(fd, request, buf):
    """The filled-in buffer, or None once an enumeration runs out."""
    try:
        fcntl.ioctl(fd, request, buf)
        return buf
    except OSError:
        return None


def rates(fd, pix, w, h):
    out, k = set(), 0
    while True:
        iv = ioctl(fd, ENUM_RATES, bytearray(struct.pack("<5I", k, pix, w, h, 0) + bytes(32)))
        if iv is None:
            return sorted(out)
        num, den = struct.unpack_from("<II", iv, 20)
        out.add(round(den / num) if num else 0)
        k += 1


def sizes(fd, pix):
    out, j = [], 0
    while True:
        sz = ioctl(fd, ENUM_SIZES, bytearray(struct.pack("<3I", j, pix, 0) + bytes(32)))
        if sz is None:
            return out
        w, h = struct.unpack_from("<II", sz, 12)
        out.append(f"{w}x{h}@{'/'.join(map(str, rates(fd, pix, w, h)))}")
        j += 1


def main():
    for dev in sys.argv[1:]:
        fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
        cap = ioctl(fd, QUERYCAP, bytearray(104))
        card, bus = (cap[a:b].split(b"\0")[0].decode() for a, b in ((16, 48), (48, 80)))
        device_caps = struct.unpack_from("<I", cap, 88)[0]
        kind = CAPTURE if device_caps & 0x1 else META
        print(f"{dev}: {card} ({bus}), {'video' if kind == CAPTURE else 'metadata'}")
        i = 0
        while True:
            fmt = ioctl(fd, ENUM_FMT, bytearray(struct.pack("<II", i, kind) + bytes(56)))
            if fmt is None:
                break
            pix = struct.unpack_from("<I", fmt, 44)[0]
            fourcc = pix.to_bytes(4, "little").decode(errors="replace")
            found = sizes(fd, pix) if kind == CAPTURE else []
            name = fmt[12:44].split(b"\0")[0].decode()
            print(f"  {fourcc} {name}: {', '.join(found) or '-'}")
            i += 1
        os.close(fd)
    return 0


if __name__ == "__main__":
    sys.exit(main())
