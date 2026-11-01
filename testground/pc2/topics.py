"""List every DDS topic published on the robot's domain, with its type and how many
publishers it has, by listening to DDS discovery for a few seconds. Read-only.

  python3 topics.py [--seconds 3] [--interface eth0]
"""

import argparse
import sys
import time

from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
from cyclonedds.domain import DomainParticipant
from unitree_sdk2py.core.channel import ChannelFactoryInitialize


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--interface", default="eth0")
    parser.add_argument("--domain", type=int, default=0)
    args = parser.parse_args()

    # The SDK sets up the domain on the robot's network; this participant shares it.
    ChannelFactoryInitialize(args.domain, args.interface)
    participant = DomainParticipant(args.domain)
    reader = BuiltinDataReader(participant, BuiltinTopicDcpsPublication)
    time.sleep(args.seconds)
    topics = {}
    for pub in reader.take(N=100000):
        topics.setdefault((pub.topic_name, pub.type_name), set()).add(str(pub.participant_key))
    print(f"{len(topics)} topics on domain {args.domain} ({args.interface}):")
    for (name, kind), participants in sorted(topics.items()):
        print(f"  {name:50s} {kind}  ({len(participants)} publisher{'s' * (len(participants) > 1)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
