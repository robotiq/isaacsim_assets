#!/usr/bin/env python3
"""Verify ur5e_servo.launch.py builds a coherent launch description.

Runs on ROS 2 Humble and Jazzy alike, in CI (see .gitlab-ci.yml,
isaac-teleop-launch-smoke) or by hand. Needs no GPU and no Isaac Sim: the
launch file only assembles parameters, so a container with moveit_servo,
ur_description and ur_moveit_config installed is enough.

Every assertion checks the launch description against something read from the
environment — a file on disk, the generated SRDF, the installed configs. None
of them restate the mapping _servo_compat already owns; a table of
"humble -> servo_node_main" here would just agree with the module by
construction and catch nothing.
"""
import importlib.util
import os
import sys

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _servo_compat as compat  # noqa: E402

failures = []


def check(label, condition, detail=""):
    if condition:
        print(f"  ok    {label}")
    else:
        print(f"  FAIL  {label}{': ' + detail if detail else ''}")
        failures.append(label)


def load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "ur5e_servo_launch", os.path.join(HERE, "ur5e_servo.launch.py")
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def servo_node_of(launch_description):
    for entity in launch_description.entities:
        package = getattr(entity, "node_package", None)
        if package is not None and "moveit_servo" in str(package):
            return entity
    raise AssertionError("no moveit_servo node in the launch description")


def resolved(substitutions):
    """Flatten a launch substitution list to a plain string."""
    from launch import LaunchContext

    context = LaunchContext()
    if isinstance(substitutions, str):
        return substitutions
    return "".join(s.perform(context) for s in substitutions)


def main():
    print(f"ROS_DISTRO={os.environ.get('ROS_DISTRO', '?')}  new_servo={compat.new_servo()}")

    launch_module = load_launch_module()
    launch_description = launch_module.generate_launch_description()
    node = servo_node_of(launch_description)

    # 1. The executable this launch will exec must exist in the install tree.
    executable = resolved(node.node_executable)
    lib = os.path.join(get_package_prefix("moveit_servo"), "lib", "moveit_servo")
    check(
        f"servo executable {executable!r} exists in {lib}",
        os.path.exists(os.path.join(lib, executable)),
        f"contents: {sorted(os.listdir(lib))}",
    )

    context = launch_module.build_servo_context()
    servo_cfg = context["servo_yaml"]
    kinematics = context["kinematics_yaml"]["robot_description_kinematics"]

    # 2. The planning group must be the one the generated SRDF actually names,
    #    and must carry kinematics.
    srdf_xml = compat.run_xacro(
        "ur_moveit_config", "srdf/ur.srdf.xacro", name="ur5e", prefix=""
    )
    move_group = servo_cfg.get("move_group_name")
    check(
        f"move_group {move_group!r} matches the generated SRDF",
        move_group == compat.srdf_group_name(srdf_xml),
        f"SRDF says {compat.srdf_group_name(srdf_xml)!r}",
    )
    check(
        f"kinematics provided for {move_group!r}",
        move_group in kinematics and bool(kinematics[move_group]),
    )
    check(
        "kinematics carry a solver",
        any("kinematics_solver" in str(k) for k in kinematics.get(move_group, {})),
    )

    # 3. Nothing undeclared survives, where the declared set is knowable.
    declared = compat.declared_servo_params()
    if declared is None:
        print("  skip  declared-parameter filtering (this Servo publishes no manifest)")
    else:
        undeclared = sorted(set(servo_cfg) - declared)
        check("every servo parameter is declared", not undeclared, f"undeclared: {undeclared}")

    # 4. Exactly one idle-protection mechanism. This is the assertion that
    #    catches the silent regression, and it needs no distro knowledge.
    old_style = servo_cfg.get("low_latency_mode") is True
    new_style = (
        servo_cfg.get("halt_all_joints_in_cartesian_mode") is True
        and servo_cfg.get("halt_all_joints_in_joint_mode") is True
        and "incoming_command_timeout" in servo_cfg
    )
    check(
        "exactly one idle-protection mechanism is configured",
        old_style != new_style,
        f"low_latency_mode={servo_cfg.get('low_latency_mode')} "
        f"halt_cartesian={servo_cfg.get('halt_all_joints_in_cartesian_mode')} "
        f"halt_joint={servo_cfg.get('halt_all_joints_in_joint_mode')} "
        f"timeout={servo_cfg.get('incoming_command_timeout')}",
    )
    # ...and the mechanism chosen must be one this Servo declares.
    if declared is not None:
        chosen = "low_latency_mode" if old_style else "halt_all_joints_in_cartesian_mode"
        check(f"chosen mechanism {chosen!r} is declared", chosen in declared)

    # 5. Settings the teleop stack depends on, independent of distro.
    check("joint_topic points at the filtered arm topic",
          servo_cfg.get("joint_topic") == "/joint_states_arm")
    check("override_velocity_scaling_factor is non-zero",
          servo_cfg.get("override_velocity_scaling_factor") == 1.0)
    check("angular scale raised for interactive teleop",
          servo_cfg.get("scale", {}).get("rotational") == 3.0,
          f"scale={servo_cfg.get('scale')}")

    # 6. The group must exist in the installed kinematics.yaml too, whichever
    #    layout that file uses.
    raw = yaml.safe_load(
        open(os.path.join(get_package_share_directory("ur_moveit_config"), "config/kinematics.yaml"))
    )
    check("installed kinematics.yaml yields exactly one group",
          len(compat.unwrap_kinematics(raw)) == 1,
          f"groups: {list(compat.unwrap_kinematics(raw))}")

    print()
    if failures:
        print(f"FAILED ({len(failures)}): {', '.join(failures)}")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
