# DDS isolation for the Isaac Sim teleop demo.  Source this in EVERY terminal.
# Lives beside the demo rather than inside one rig: both the PhysX teleop and
# anything else that publishes need it.
#
# WHY THIS EXISTS
# ---------------
# This machine sits on a shared office network.  With the ROS 2
# defaults (ROS_DOMAIN_ID=0 + subnet-wide discovery) it joined a shared DDS
# graph containing ~50 nodes belonging to OTHER machines, including live
# robot drivers:
#
#     /frankahardwareinterface     Franka FR3 hardware interface
#     /fr3_arm_controller
#     /dashboard_client            UR dashboard client
#     /controller_manager  (x2)
#     /gello_joint_publisher       GELLO teleop rig
#
# MoveIt Servo publishes to /forward_position_controller/commands (set by
# ur_moveit_config's ur_servo.yaml).  On the shared graph that topic already
# had two REMOTE subscribers.  Running the teleop stack unisolated would have
# streamed joint position commands straight to somebody else's robot.
#
# Everything in this demo runs on this one laptop, so restricting discovery to
# localhost costs nothing and removes the hazard entirely.
#
# USAGE
# -----
#   Isaac Sim terminal (CLEAN shell - do NOT source /opt/ros/jazzy first;
#   its Python collides with Isaac's bundled interpreter):
#       source ~/isaac-demo-dds.sh
#       ~/isaacsim/isaac-sim.sh
#
#   ROS consumer terminals:
#       source /opt/ros/jazzy/setup.bash
#       source ~/isaac-demo-dds.sh
#       ros2 launch .../ur5e_servo.launch.py
#
# Both sides must agree on all three variables below or they will not see
# each other.

# Jazzy's discovery-range control (replaces the deprecated ROS_LOCALHOST_ONLY).
# LOCALHOST = only ever discover peers on this machine.
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# Defense in depth: even if discovery range were widened by accident, domain 42
# keeps us off the default domain 0 that the shop floor is using.
export ROS_DOMAIN_ID=42

# Isaac Sim's setup_ros_env.sh defaults to this; pin it so both sides match.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# NOTE: deliberately does NOT set ROS_DISTRO.  Isaac's setup_ros_env.sh only
# wires up its bundled Jazzy libraries when ROS_DISTRO is unset, and the repo
# requires Isaac to run from a shell where /opt/ros/jazzy has NOT been sourced.
