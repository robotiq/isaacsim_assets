# The robotiq/ros gripper stack (humble), to drive the simulated gripper end to end with
# the same launch file used for the real one. Host networking, so DDS reaches Isaac Sim.
#   docker build -t robotiq-ros-humble tests/ros2
FROM ros:humble

RUN apt-get update && apt-get install -y --no-install-recommends \
        git python3-colcon-common-extensions python3-rosdep ros-humble-xacro \
        ros-humble-topic-based-ros2-control ros-humble-ros2-controllers \
        ros-humble-ros2controlcli ros-humble-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# Pin with --build-arg ROBOTIQ_ROS_REF=<tag or sha> to test against a release.
ARG ROBOTIQ_ROS_REF=main
RUN git clone https://github.com/robotiq/ros /ws/src/ros \
    && git -C /ws/src/ros checkout "$ROBOTIQ_ROS_REF" \
    && git -C /ws/src/ros submodule update --init extern/grippers
WORKDIR /ws
RUN . /opt/ros/humble/setup.sh \
    && rosdep update --rosdistro humble \
    && apt-get update \
    && rosdep install --from-paths src/ros/grippers --ignore-src -y -r \
        --skip-keys="ament_clang_format ament_clang_tidy rviz2 joint_state_publisher_gui" \
    && rm -rf /var/lib/apt/lists/* \
    && colcon build --base-paths src/ros/grippers --packages-up-to robotiq_description robotiq_controllers robotiq_msgs --cmake-args -DBUILD_TESTING=OFF
