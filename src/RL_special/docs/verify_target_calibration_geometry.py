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


def calibration_axis_roots(
    node_cls,
    x_local: float,
    y_local: float,
    yaw_error_rad: float,
    motion_sign: float,
    distance_m: float,
) -> list[tuple[float, tuple[float, float, float]]]:
    signed_distance_m = motion_sign * distance_m

    def axis_error(candidate_rad: float) -> float:
        moved_state = node_cls._target_state_after_calibration_motion(
            x_local,
            y_local,
            yaw_error_rad,
            candidate_rad,
            signed_distance_m,
        )
        _, robot_y_target_frame_m, _ = node_cls._robot_pose_in_target_frame(
            *moved_state
        )
        return robot_y_target_frame_m

    roots: list[tuple[float, tuple[float, float, float]]] = []
    previous_angle = -math.pi
    previous_error = axis_error(previous_angle)
    for sample_idx in range(1, 1441):
        angle = -math.pi + (2.0 * math.pi * sample_idx / 1440)
        error = axis_error(angle)
        if previous_error * error < 0.0:
            low = previous_angle
            high = angle
            low_error = previous_error
            for _ in range(48):
                mid = 0.5 * (low + high)
                mid_error = axis_error(mid)
                if low_error * mid_error <= 0.0:
                    high = mid
                else:
                    low = mid
                    low_error = mid_error
            root = 0.5 * (low + high)
            moved_state = node_cls._target_state_after_calibration_motion(
                x_local,
                y_local,
                yaw_error_rad,
                root,
                signed_distance_m,
            )
            roots.append((root, node_cls._robot_pose_in_target_frame(*moved_state)))
        previous_angle = angle
        previous_error = error
    return roots


def verify_calibration_cases(node_cls) -> tuple[int, int]:
    checked = 0
    skipped_unreachable = 0

    scenarios = [
        ("front", "target farther than robot", [-0.45, -0.25, -0.10]),
        ("front", "robot between target and cart", [0.03, 0.10, 0.20]),
        ("rear", "target farther than robot", [0.10, 0.25, 0.45]),
        ("rear", "robot between target and cart", [-0.03, -0.10, -0.20]),
    ]

    for robot_type, scenario_name, x_values in scenarios:
        motion_sign = 1.0 if robot_type == "front" else -1.0
        for x_local in x_values:
            for y_local in (-0.24, -0.12, -0.04, 0.04, 0.12, 0.24):
                for yaw_deg in (-35.0, -15.0, 0.0, 15.0, 35.0):
                    yaw = math.radians(yaw_deg)
                    axis_distance = abs(
                        y_local * math.cos(yaw) - x_local * math.sin(yaw)
                    )
                    if axis_distance > 0.3000001:
                        skipped_unreachable += 1
                        continue

                    rotate_out = node_cls._compute_calibration_rotate_out_target_rad(
                        x_local,
                        y_local,
                        yaw,
                        motion_sign,
                        0.30,
                    )
                    after_move = node_cls._target_state_after_calibration_motion(
                        x_local,
                        y_local,
                        yaw,
                        rotate_out,
                        motion_sign * 0.30,
                    )
                    robot_x_t, robot_y_t, _ = node_cls._robot_pose_in_target_frame(
                        *after_move
                    )
                    if abs(robot_y_t) > 1.0e-7:
                        raise AssertionError(
                            f"{robot_type} {scenario_name}: after move y_T is "
                            f"{robot_y_t:.12f}, state={(x_local, y_local, yaw_deg)}"
                        )

                    roots = calibration_axis_roots(
                        node_cls,
                        x_local,
                        y_local,
                        yaw,
                        motion_sign,
                        0.30,
                    )
                    preferred_roots = [
                        pose for _, pose in roots
                        if pose[0] * motion_sign >= -1.0e-7
                    ]
                    if preferred_roots and robot_x_t * motion_sign < -1.0e-7:
                        raise AssertionError(
                            f"{robot_type} {scenario_name}: wrong target axis side, "
                            f"x_T={robot_x_t:.12f}, motion_sign={motion_sign}, "
                            f"state={(x_local, y_local, yaw_deg)}"
                        )
                    if not preferred_roots:
                        farthest_available_x = max(abs(pose[0]) for _, pose in roots)
                        if abs(abs(robot_x_t) - farthest_available_x) > 1.0e-7:
                            raise AssertionError(
                                f"{robot_type} {scenario_name}: did not choose the "
                                f"farther available x-axis intersection, "
                                f"x_T={robot_x_t:.12f}, farthest={farthest_available_x:.12f}, "
                                f"state={(x_local, y_local, yaw_deg)}"
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
                            f"{robot_type} {scenario_name}: final state invalid, "
                            f"y_T={final_y_t:.12f}, heading_T={final_heading_t:.12f}, "
                            f"state={(x_local, y_local, yaw_deg)}"
                        )
                    checked += 1

    return checked, skipped_unreachable


def main() -> None:
    node_cls = load_node_class()
    canonical_count = verify_canonical_targets(node_cls)
    calibration_count, skipped_count = verify_calibration_cases(node_cls)
    print(
        "OK: "
        f"canonical_cases={canonical_count}, "
        f"calibration_cases={calibration_count}, "
        f"skipped_unreachable={skipped_count}"
    )


if __name__ == "__main__":
    main()
