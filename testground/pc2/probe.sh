#!/usr/bin/env bash
# Read-only survey of PC2 before recording: the system, what's installed, whether the
# lidar answers, disk, load, what's running, and (when Unitree's SDK imports) every
# DDS topic on the robot's domain. Changes nothing.
#
#   ./probe.sh [INTERFACE]   (default eth0, PC2's port on the robot network)
here=$(cd "$(dirname "$0")" && pwd)
interface=${1:-eth0}
section() { printf '\n== %s\n' "$1"; }

section System
uname -a
grep PRETTY_NAME /etc/os-release
head -1 /etc/nv_tegra_release 2>/dev/null
uptime

section "Disk and memory"
df -h "$here" | tail -1
free -h | head -2

section Network
ip -br addr
for host in 192.168.123.120 192.168.123.161; do   # the lidar, PC1
  if ping -c 2 -W 1 "$host" >/dev/null 2>&1; then echo "$host answers"; else echo "$host: no answer"; fi
done

section ROS
ls /opt/ros 2>/dev/null || echo "no /opt/ros"
echo "ROS_DISTRO=${ROS_DISTRO-} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION-} CYCLONEDDS_HOME=${CYCLONEDDS_HOME-}"

section Python
python3 --version
python3 - <<'EOF'
import importlib
for name in ("unitree_sdk2py", "cyclonedds", "numpy"):
    try:
        module = importlib.import_module(name)
        print(f"{name}: {getattr(module, '__version__', '?')} at {module.__file__}")
    except Exception as e:
        print(f"{name}: not importable ({type(e).__name__}: {e})")
EOF

section Processes
ps -eo pid,user,pcpu,pmem,etime,args --sort=-pcpu \
  | grep -i -E 'lidar|livox|ros|dds|unitree|docker|python' | grep -v -E 'grep|probe.sh' | head -40

section "DDS topics"
if python3 -c "import unitree_sdk2py" 2>/dev/null; then
  python3 "$here/topics.py" --interface "$interface"
else
  echo "skipped: unitree_sdk2py isn't importable"
fi
