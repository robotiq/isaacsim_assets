# The robotiq/ros gripper stack, to drive the simulated gripper end to end with the same
# launch file used for the real one. Host networking, so DDS reaches Isaac Sim (run it with
# fastdds_udp.xml, see grippers/ros2/README.md).
#   docker build -t robotiq-ros tests/ros2                          # Jazzy, as robotiq/ros
#   docker build -t robotiq-ros --build-arg ROS_DISTRO=humble tests/ros2
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}
ARG ROS_DISTRO

RUN apt-get update && apt-get install -y --no-install-recommends \
        git python3-colcon-common-extensions python3-rosdep \
        ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-ros2controlcli ros-${ROS_DISTRO}-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
# topic_based_ros2_control, the plugin sim_topic_based loads: a binary release on Humble
# only; elsewhere built from source, as the robotiq/ros README documents.
# Humble EOL: simplify — keep the source build, drop the apt branch.
RUN if [ "$ROS_DISTRO" = humble ]; then \
        apt-get update \
        && apt-get install -y --no-install-recommends ros-humble-topic-based-ros2-control \
        && rm -rf /var/lib/apt/lists/*; \
    else \
        git clone --depth 1 https://github.com/PickNikRobotics/topic_based_ros2_control src/topic_based_ros2_control; \
    fi

# Pin with --build-arg ROBOTIQ_ROS_REF=<branch, tag or sha> to test against a release. The
# ADD changes whenever the ref moves, so a rebuild fetches robotiq/ros again instead of
# reusing the cached clone of an older commit.
ARG ROBOTIQ_ROS_REF=main
ADD https://api.github.com/repos/robotiq/ros/commits/${ROBOTIQ_ROS_REF} /tmp/robotiq_ros_ref.json
RUN git clone https://github.com/robotiq/ros src/ros \
    && git -C src/ros checkout "$ROBOTIQ_ROS_REF" \
    && git -C src/ros submodule update --init extern/grippers

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && PKGS="src/ros/grippers $( [ -d src/topic_based_ros2_control ] && echo src/topic_based_ros2_control || true )" \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && apt-get update \
    && rosdep install --from-paths $PKGS --ignore-src -y -r \
        --skip-keys="ament_clang_format ament_clang_tidy rviz2 joint_state_publisher_gui ros_testing" \
    && rm -rf /var/lib/apt/lists/* \
    && colcon build --base-paths $PKGS \
        --packages-up-to robotiq_description robotiq_controllers robotiq_msgs $( [ -d src/topic_based_ros2_control ] && echo topic_based_ros2_control || true ) \
        --cmake-args -DBUILD_TESTING=OFF
