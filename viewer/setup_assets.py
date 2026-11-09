#!/usr/bin/env python3
"""Install the pinned Three.js dependency for offline sensor views."""
import argparse
import hashlib
import io
from pathlib import Path
import tarfile
import urllib.request

VERSION = "0.160.0"
URL = "https://registry.npmjs.org/three/-/three-%s.tgz" % VERSION
SHA256 = "1ee2f935c4f555814b388e87b5ef78a44856bd2e9d0feb88643a6e193fb42856"
FILES = {"package/build/three.module.js": "three.module.js",
         "package/examples/jsm/controls/OrbitControls.js": "OrbitControls.js", "package/LICENSE": "LICENSE"}


def install(data, destination):
    if hashlib.sha256(data).hexdigest() != SHA256:
        raise ValueError("Three.js archive hash does not match the pinned dependency")
    with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as archive:
        selected = {name: archive.extractfile(member).read() for member, name in FILES.items()}
    destination.mkdir(parents=True, exist_ok=True)
    for name, body in selected.items():
        (destination / name).write_bytes(body)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, help="install from an already downloaded archive")
    args = parser.parse_args()
    if args.archive:
        data = args.archive.read_bytes()
    else:
        with urllib.request.urlopen(URL, timeout=30) as response:
            data = response.read(20 * 1024 * 1024)
    install(data, Path(__file__).resolve().parent / ".assets")
    print("Installed Three.js %s; sensor views can run without internet." % VERSION)


if __name__ == "__main__":
    main()
