"""Print a Fast DDS profile for our image in HIM's simulator (entrypoint.sh uses it).

HIM's sim profile (/fastdds.xml) keeps everything on 127.0.0.1 and finds peers
by unicast at the default participant ids, 0-4 only. Our image runs about ten
participants (our node, the lidar, five Nav2 nodes, the launcher, the course
goal), so with it Nav2's nodes never found each other and its bringup hung.
This is the same localhost transport, with explicit peers for ids 0 to
PARTICIPANTS - 1 on ROS_DOMAIN_ID. The robot doesn't set a profile and uses
normal discovery.
"""

import os

PARTICIPANTS = 32
# Fast DDS's well-known ports: PB + DG * domain + d1 + PG * participant id (metatraffic unicast).
PB, DG, PG, D1 = 7400, 250, 2, 10


def main():
    domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    peers = "\n".join(
        f"          <locator><udpv4><address>127.0.0.1</address>"
        f"<port>{PB + DG * domain + D1 + PG * i}</port></udpv4></locator>"
        for i in range(PARTICIPANTS))
    print(f"""<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_localhost</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList>
        <address>127.0.0.1</address>
      </interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="harp_udp" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udp_localhost</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
      <builtin>
        <metatrafficUnicastLocatorList>
          <locator><udpv4><address>127.0.0.1</address></udpv4></locator>
        </metatrafficUnicastLocatorList>
        <initialPeersList>
{peers}
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>""")


if __name__ == "__main__":
    main()
