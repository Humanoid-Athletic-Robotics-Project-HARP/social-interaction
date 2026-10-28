# HARP's submission image for him-sim2real-management (its containers/INSTRUCTIONS.md).
# Built by build.sh from a staged context: him/ is that repo's walker example, ours is the rest.
FROM ros:foxy-ros-base
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-pip python3-colcon-common-extensions \
    ros-foxy-rosidl-default-generators ros-foxy-rmw-fastrtps-cpp ros-foxy-std-srvs \
    ros-foxy-geometry-msgs ros-foxy-nav-msgs ros-foxy-sensor-msgs ros-foxy-tf2-ros \
    ros-foxy-nav2-controller ros-foxy-nav2-planner ros-foxy-nav2-navfn-planner \
    ros-foxy-nav2-recoveries ros-foxy-nav2-bt-navigator ros-foxy-nav2-lifecycle-manager \
    ros-foxy-nav2-regulated-pure-pursuit-controller ros-foxy-spatio-temporal-voxel-layer \
    && rm -rf /var/lib/apt/lists/*
# ONNX Runtime 1.16.3 and MuJoCo 3.2.3 are the newest for Foxy's Python 3.8 on ARM64.
RUN python3 -m pip install --no-cache-dir numpy==1.24.4 onnxruntime==1.16.3 PyYAML==6.0.2 \
    mujoco==3.2.3 "zipp>=3.1"
# The shared client expects its files under /example.
WORKDIR /example
COPY him/container/interfaces /example/interfaces
RUN source /opt/ros/foxy/setup.bash && colcon build --base-paths interfaces \
    --packages-select him_safety_interfaces
COPY him/container/controllers.py him/container/client.py him/container/trial.py \
     him/container/command.py /example/
COPY him/container/robot_safety /example/robot_safety
COPY him/container/policy_safety /example/policy_safety
COPY him/container/project /example/project
COPY him/environment /harp/model
COPY harp_node.py entrypoint.sh /harp/
COPY harp /harp/harp
COPY nav2 /harp/nav2
RUN python3 -c "import sys; sys.path.insert(0, '/example'); from controllers import ExamplePolicy; ExamplePolicy('walker', '/example/project')" \
    && python3 -c "import sys; sys.path.insert(0, '/harp'); from harp.leg_odometry import LegOdometry; LegOdometry('/harp/model/g1_robot.xml')"
ENTRYPOINT ["/bin/bash", "/harp/entrypoint.sh"]
