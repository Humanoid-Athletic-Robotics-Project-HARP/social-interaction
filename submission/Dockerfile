# HARP's submission image for him-sim2real-management (its containers/INSTRUCTIONS.md).
# Built by build.sh from a staged context: him/ is that repo's walker example, ours is the rest.
# Two stages: "build" compiles and installs, the image keeps only what runs (about 310 MB
# saved, against 970 MB in one stage; context/submission-image.md).
FROM ros:foxy-ros-base AS build
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-pip python3-colcon-common-extensions \
    ros-foxy-rosidl-default-generators ros-foxy-rmw-fastrtps-cpp ros-foxy-std-srvs \
    ros-foxy-nav2-common ros-foxy-nav2-core ros-foxy-nav2-costmap-2d ros-foxy-nav2-util \
    ros-foxy-nav2-msgs ros-foxy-nav-2d-utils ros-foxy-angles ros-foxy-tf2-geometry-msgs \
    ros-foxy-visualization-msgs libboost-dev git \
    && rm -rf /var/lib/apt/lists/*
# ONNX Runtime 1.16.3 and MuJoCo 3.2.3 are the newest for Foxy's Python 3.8 on ARM64.
RUN python3 -m pip install --no-cache-dir numpy==1.24.4 onnxruntime==1.16.3 PyYAML==6.0.2 \
    mujoco==3.2.3 "zipp>=3.1"
# Unitree's SDK, for harp/lidar_relay.py on the robot: CycloneDDS 0.10.2 (the version the SDK's
# bindings need; no wheel exists for Python 3.8 on ARM64) and the SDK at the commit in the README.
ENV CYCLONEDDS_HOME=/opt/cyclonedds
RUN git clone -q --depth 1 -b 0.10.2 https://github.com/eclipse-cyclonedds/cyclonedds /tmp/cyclonedds \
    && cmake -S /tmp/cyclonedds -B /tmp/cyclonedds/build -DCMAKE_INSTALL_PREFIX=$CYCLONEDDS_HOME \
       -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DENABLE_SHM=OFF \
    && cmake --build /tmp/cyclonedds/build --target install -j \
    && rm -rf /tmp/cyclonedds \
    && python3 -m pip install --no-cache-dir cyclonedds==0.10.2 \
       "git+https://github.com/unitreerobotics/unitree_sdk2_python.git@814556d15970dd2ecf1c9984e845ca02ab07e206"
# Never imported: OpenCV (the SDK's camera code), SymPy (ONNX Runtime's tools), PyOpenGL
# (MuJoCo's rendering). About 200 MB.
RUN python3 -m pip uninstall -y -q opencv-python sympy mpmath PyOpenGL
COPY him/container/interfaces /example/interfaces
RUN source /opt/ros/foxy/setup.bash && cd /example && colcon build --base-paths interfaces \
    --packages-select him_safety_interfaces
# Our Foxy port of Nav2's Graceful Controller (nav2_graceful_controller/HARP_PATCHES.md).
COPY nav2_graceful_controller /harp_ws/src/nav2_graceful_controller
RUN source /opt/ros/foxy/setup.bash && cd /harp_ws && colcon build \
    --packages-select nav2_graceful_controller --cmake-args -DBUILD_TESTING=OFF

FROM ros:foxy-ros-base
SHELL ["/bin/bash", "-c"]
# Empty stand-ins for packages Foxy's debs depend on but never load: build headers, and
# the parts of PCL that bring VTK, Mesa's GL drivers, OpenMPI and a Fortran compiler.
# STVL names all of PCL (it loads common, filters, kdtree, octree, search, sample
# consensus), behaviortree_cpp_v3 names libboost-all-dev, pcl_conversions is headers only.
RUN for p in libboost-all-dev libncurses5-dev libzmq3-dev libeigen3-dev libpcl-dev \
      libilmbase-dev libopenexr-dev libopenvdb-dev libtbb-dev libpcl-apps1.10 \
      libpcl-features1.10 libpcl-io1.10 libpcl-keypoints1.10 libpcl-ml1.10 \
      libpcl-outofcore1.10 libpcl-people1.10 libpcl-recognition1.10 libpcl-registration1.10 \
      libpcl-segmentation1.10 libpcl-stereo1.10 libpcl-surface1.10 libpcl-tracking1.10 \
      libpcl-visualization1.10; do \
      mkdir -p /tmp/stub/$p/DEBIAN \
      && printf 'Package: %s\nVersion: 1:99\nArchitecture: all\nMaintainer: HARP\nDescription: empty stand-in\n' \
         "$p" > /tmp/stub/$p/DEBIAN/control \
      && dpkg-deb -b /tmp/stub/$p /tmp/stub/$p.deb >/dev/null; done \
    && dpkg -i /tmp/stub/*.deb >/dev/null && rm -rf /tmp/stub
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-rmw-fastrtps-cpp ros-foxy-std-srvs \
    ros-foxy-geometry-msgs ros-foxy-nav-msgs ros-foxy-sensor-msgs ros-foxy-tf2-ros \
    ros-foxy-nav2-controller ros-foxy-nav2-planner ros-foxy-nav2-navfn-planner \
    ros-foxy-nav2-recoveries ros-foxy-nav2-bt-navigator ros-foxy-nav2-lifecycle-manager \
    ros-foxy-nav2-regulated-pure-pursuit-controller ros-foxy-spatio-temporal-voxel-layer \
    ros-foxy-nav2-common ros-foxy-nav-2d-utils ros-foxy-angles ros-foxy-visualization-msgs \
    ros-foxy-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*
ENV CYCLONEDDS_HOME=/opt/cyclonedds
COPY --from=build /usr/local/lib/python3.8/dist-packages /usr/local/lib/python3.8/dist-packages
COPY --from=build /opt/cyclonedds /opt/cyclonedds
COPY --from=build /example/install /example/install
COPY --from=build /harp_ws/install /harp_ws/install
# The shared client expects its files under /example.
WORKDIR /example
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
# The moves when G1_TASK_COMMAND is empty (entrypoint.sh).
ENV HARP_DEFAULT_TASK='{"to": [0.5, 0.5]}'
ENTRYPOINT ["/bin/bash", "/harp/entrypoint.sh"]
