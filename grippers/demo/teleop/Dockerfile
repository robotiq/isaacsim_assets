# Dev-facing image for the Isaac teleop ROS side. Optional: the stack runs
# natively on Humble and Jazzy alike (see README.md). Reach for this to try the
# other distro without touching your workstation, or to reproduce a
# distro-specific bug a colleague is seeing.
#
# Isaac Sim stays on the host -- it needs the GPU, and the container only ever
# talks to it over ROS topics.
#
# Build via run.sh, which passes ROS_DISTRO. Keep the package list in step with
# the isaac-teleop-launch-smoke job in .gitlab-ci.yml.
ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-moveit-servo \
        ros-${ROS_DISTRO}-ur-description \
        ros-${ROS_DISTRO}-ur-moveit-config \
        ros-${ROS_DISTRO}-joy \
        ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /teleop_entrypoint.sh
ENTRYPOINT ["/teleop_entrypoint.sh"]

# The teleop scripts are bind-mounted here by run.sh, so editing one does not
# need a rebuild.
WORKDIR /workspace
CMD ["ros2", "launch", "/workspace/teleop.launch.py"]
