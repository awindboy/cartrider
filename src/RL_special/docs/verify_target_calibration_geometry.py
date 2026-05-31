#!/usr/bin/env python3
import importlib.util
import math
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_ROOT / "RL_special" / "specialist_policy_node.py"


def load_node_class():
    spec = importlib.util.spec_from_file_location("specialist_policy_node", NODE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.CartAlignSpecialistPolicyNode


def wrap_to_pi(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def assert_close(name: str, actual: float, expected: float, tol: float = 1.0e-9):
    if abs(actual - expected) > tol:
        raise AssertionError(
            f"{name}: actual={actual:.12f}, expected={expected:.12f}, tol={tol}"
        )


def verify_canonical_targets(node_cls) -> int:
    cases = [
        {
            "name": "rear canonical offset",
            "robot_type": "rear",
            "pose": (0.80, 0.20, math.radians(-25.0)),
            "base_to_axle": 0.120,
            "target_offset": 0.50,
        },
        {
            "name": "front canonical offset",
            "robot_type": "front",
            "pose": (0.80, 0.20, math.radians(25.0)),
            "base_to_axle": 0.095,
            "target_offset": 0.55,
        },
        {
            "name": "front passed offset target",
            "robot_type": "front",
            "pose": (0.30, 0.05, math.radians(25.0)),
            "base_to_axle": 0.095,
            "target_offset": 0.55,
        },
        {
            "name": "rear passed offset target",
            "robot_type": "rear",
            "pose": (0.30, 0.35, math.radians(-25.0)),
            "base_to_axle": 0.120,
            "target_offset": 0.50,
        },
    ]

    for case in cases:
        x, y, theta = case["pose"]
        yaw_error = wrap_to_pi(-theta)
        base_x = -x if case["robot_type"] == "front" else x
        base_y = -y if case["robot_type"] == "front" else y
        axle_x = base_x - case["base_to_axle"]
        axle_y = base_y
        offset_sign = 1.0 if case["robot_type"] == "front" else -1.0
        expected_x = (
            axle_x
            + offset_sign * case["target_offset"] * math.cos(yaw_error)
        )
        expected_y = (
            axle_y
            + offset_sign * case["target_offset"] * math.sin(yaw_error)
        )

        actual_x, actual_y, actual_yaw = node_cls._canonical_target_from_pose(
            case["robot_type"],
            x,
            y,
            theta,
            case["base_to_axle"],
            case["target_offset"],
        )
        assert_close(f"{case['name']} x", actual_x, expected_x)
        assert_close(f"{case['name']} y", actual_y, expected_y)
        assert_close(f"{case['name']} yaw", actual_yaw, yaw_error)

    return len(cases)


def target_state_after_rotation(node_cls, x, y, yaw, rotate_rad):
    cos_yaw = math.cos(rotate_rad)
    sin_yaw = math.sin(rotate_rad)
    return (
        cos_yaw * x + sin_yaw * y,
        -sin_yaw * x + cos_yaw * y,
        node_cls._wrap_to_pi(yaw - rotate_rad),
    )


def target_state_after_motion(x, y, yaw, signed_distance_m):
    return (
        x - signed_distance_m,
        y,
        yaw,
    )


def expected_calibration_goal_x(robot_x_target_frame_m: float, axis_sign: float) -> float:
    if axis_sign > 0.0:
        return max(0.0, robot_x_target_frame_m)
    return min(0.0, robot_x_target_frame_m)


def verify_calibration_cases(node_cls) -> int:
    checked = 0

    scenarios = [
        ("front", 1.0, 1.0, [-0.45, -0.25, -0.10, 0.03, 0.10, 0.20]),
        ("rear", -1.0, -1.0, [-0.20, -0.10, -0.03, 0.10, 0.25, 0.45]),
    ]

    for robot_type, axis_sign, motion_sign, x_values in scenarios:
        for x_local in x_values:
            for y_local in (-0.24, -0.12, -0.04, 0.04, 0.12, 0.24):
                for yaw_deg in (-35.0, -15.0, 0.0, 15.0, 35.0):
                    yaw = math.radians(yaw_deg)
                    (
                        robot_x_t,
                        robot_y_t,
                        robot_heading_t,
                    ) = node_cls._robot_pose_in_target_frame(
                        x_local,
                        y_local,
                        yaw,
                    )
                    rotate_out, move_distance = (
                        node_cls._compute_calibration_rotate_out_and_move_distance(
                            x_local,
                            y_local,
                            yaw,
                            axis_sign,
                            motion_sign,
                        )
                    )
                    after_rotate = target_state_after_rotation(
                        node_cls,
                        x_local,
                        y_local,
                        yaw,
                        rotate_out,
                    )
                    after_move = target_state_after_motion(
                        *after_rotate,
                        motion_sign * move_distance,
                    )
                    (
                        moved_robot_x_t,
                        moved_robot_y_t,
                        moved_heading_t,
                    ) = node_cls._robot_pose_in_target_frame(*after_move)

                    goal_x_t = expected_calibration_goal_x(robot_x_t, axis_sign)
                    assert_close(
                        f"{robot_type} goal x",
                        moved_robot_x_t,
                        goal_x_t,
                        1.0e-7,
                    )
                    assert_close(
                        f"{robot_type} goal y",
                        moved_robot_y_t,
                        0.0,
                        1.0e-7,
                    )

                    expected_move_distance = math.hypot(
                        goal_x_t - robot_x_t,
                        -robot_y_t,
                    )
                    assert_close(
                        f"{robot_type} move distance",
                        move_distance,
                        expected_move_distance,
                        1.0e-9,
                    )

                    dx = moved_robot_x_t - robot_x_t
                    dy = moved_robot_y_t - robot_y_t
                    assert_close(
                        f"{robot_type} move norm",
                        math.hypot(dx, dy),
                        move_distance,
                        1.0e-7,
                    )

                    expected_heading_t = wrap_to_pi(robot_heading_t + rotate_out)
                    assert_close(
                        f"{robot_type} moved heading",
                        wrap_to_pi(moved_heading_t - expected_heading_t),
                        0.0,
                        1.0e-7,
                    )

                    if move_distance > 1.0e-9:
                        along = (
                            dx * math.cos(moved_heading_t)
                            + dy * math.sin(moved_heading_t)
                        )
                        cross = (
                            dx * math.sin(moved_heading_t)
                            - dy * math.cos(moved_heading_t)
                        )
                        assert_close(
                            f"{robot_type} along heading",
                            along,
                            motion_sign * move_distance,
                            1.0e-7,
                        )
                        assert_close(
                            f"{robot_type} lateral heading",
                            cross,
                            0.0,
                            1.0e-7,
                        )

                    rotate_back = after_move[2]
                    final_state = target_state_after_rotation(
                        node_cls,
                        *after_move,
                        rotate_back,
                    )
                    _, final_y_t, final_heading_t = (
                        node_cls._robot_pose_in_target_frame(*final_state)
                    )
                    if abs(final_y_t) > 1.0e-7 or abs(final_heading_t) > 1.0e-7:
                        raise AssertionError(
                            f"{robot_type}: final state invalid, "
                            f"y_T={final_y_t:.12f}, heading_T={final_heading_t:.12f}, "
                            f"state={(x_local, y_local, yaw_deg)}"
                        )
                    checked += 1

    return checked


def main() -> None:
    node_cls = load_node_class()
    canonical_count = verify_canonical_targets(node_cls)
    calibration_count = verify_calibration_cases(node_cls)
    print(
        "OK: "
        f"canonical_cases={canonical_count}, "
        f"calibration_cases={calibration_count}"
    )


if __name__ == "__main__":
    main()
