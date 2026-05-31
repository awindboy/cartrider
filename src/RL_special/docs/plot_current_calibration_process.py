#!/usr/bin/env python3
import math
from pathlib import Path


PROFILES = {
    "rear": {
        "color": "#d62728",
        "cart_fill": "#f7d7d4",
        "target_x_offset_m": 0.50,
        "docking_axis_sign": -1.0,
    },
    "front": {
        "color": "#1f77b4",
        "cart_fill": "#d8e7fb",
        "target_x_offset_m": 0.55,
        "docking_axis_sign": 1.0,
    },
}

QUADRANTS = [
    ("Q1", 0.21, 0.16, math.radians(-18.0)),
    ("Q2", -0.21, 0.16, math.radians(-18.0)),
    ("Q3", -0.21, -0.16, math.radians(18.0)),
    ("Q4", 0.21, -0.16, math.radians(18.0)),
]

CART_LENGTH_M = 0.64
CART_WIDTH_M = 0.42
PANEL_W = 212
PANEL_H = 212
MARGIN = 24
GAP_X = 12
GAP_Y = 44
HEADER_H = 70
ROW_LABEL_W = 44
CANVAS_W = ROW_LABEL_W + PANEL_W * 8 + GAP_X * 9
CANVAS_H = HEADER_H + PANEL_H * 4 + GAP_Y * 5
X_MIN = -1.02
X_MAX = 1.02
# Keep the same metric scale on both axes so heading and motion vectors line up
# visually. Otherwise the straight move looks like sideways sliding.
Y_MIN = -1.02
Y_MAX = 1.02


def wrap_to_pi(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def robot_pose_in_target_frame(
    target_x_local: float,
    target_y_local: float,
    target_yaw_error_rad: float,
) -> tuple[float, float, float]:
    robot_heading_target_frame_rad = -target_yaw_error_rad
    cos_theta = math.cos(robot_heading_target_frame_rad)
    sin_theta = math.sin(robot_heading_target_frame_rad)
    robot_x_target_frame_m = -(
        cos_theta * target_x_local - sin_theta * target_y_local
    )
    robot_y_target_frame_m = -(
        sin_theta * target_x_local + cos_theta * target_y_local
    )
    return robot_x_target_frame_m, robot_y_target_frame_m, robot_heading_target_frame_rad


def target_state_from_robot_pose_in_target_frame(
    robot_x_target_frame_m: float,
    robot_y_target_frame_m: float,
    robot_heading_target_frame_rad: float,
) -> tuple[float, float, float]:
    cos_theta = math.cos(robot_heading_target_frame_rad)
    sin_theta = math.sin(robot_heading_target_frame_rad)
    target_x_local_m = -(
        cos_theta * robot_x_target_frame_m + sin_theta * robot_y_target_frame_m
    )
    target_y_local_m = (
        sin_theta * robot_x_target_frame_m
        - cos_theta * robot_y_target_frame_m
    )
    target_yaw_error_rad = wrap_to_pi(-robot_heading_target_frame_rad)
    return target_x_local_m, target_y_local_m, target_yaw_error_rad


def target_state_after_rotation(
    target_x_local: float,
    target_y_local: float,
    target_yaw_error_rad: float,
    rotate_rad: float,
) -> tuple[float, float, float]:
    cos_yaw = math.cos(rotate_rad)
    sin_yaw = math.sin(rotate_rad)
    return (
        cos_yaw * target_x_local + sin_yaw * target_y_local,
        -sin_yaw * target_x_local + cos_yaw * target_y_local,
        wrap_to_pi(target_yaw_error_rad - rotate_rad),
    )


def target_state_after_motion(
    target_x_local: float,
    target_y_local: float,
    target_yaw_error_rad: float,
    signed_motion_distance_m: float,
) -> tuple[float, float, float]:
    return (
        target_x_local - signed_motion_distance_m,
        target_y_local,
        target_yaw_error_rad,
    )


def compute_rotate_out_and_move_distance(
    target_x_local: float,
    target_y_local: float,
    target_yaw_error_rad: float,
    calibration_axis_sign: float,
) -> tuple[float, float, float]:
    (
        robot_x_target_frame_m,
        robot_y_target_frame_m,
        robot_heading_target_frame_rad,
    ) = robot_pose_in_target_frame(
        target_x_local,
        target_y_local,
        target_yaw_error_rad,
    )
    if calibration_axis_sign > 0.0:
        goal_x_target_frame_m = max(0.0, robot_x_target_frame_m)
    else:
        goal_x_target_frame_m = min(0.0, robot_x_target_frame_m)
    delta_x_target_frame_m = goal_x_target_frame_m - robot_x_target_frame_m
    delta_y_target_frame_m = -robot_y_target_frame_m
    move_distance_m = math.hypot(
        delta_x_target_frame_m,
        delta_y_target_frame_m,
    )
    if move_distance_m <= 1.0e-9:
        return 0.0, 0.0, 1.0
    move_direction_target_frame_rad = math.atan2(
        delta_y_target_frame_m,
        delta_x_target_frame_m,
    )
    forward_rotate_out_target_rad = wrap_to_pi(
        move_direction_target_frame_rad
        - robot_heading_target_frame_rad
    )
    reverse_rotate_out_target_rad = wrap_to_pi(
        move_direction_target_frame_rad
        + math.pi
        - robot_heading_target_frame_rad
    )
    if abs(forward_rotate_out_target_rad) <= abs(reverse_rotate_out_target_rad):
        return (
            forward_rotate_out_target_rad,
            move_distance_m,
            1.0,
        )
    return (
        reverse_rotate_out_target_rad,
        move_distance_m,
        -1.0,
    )


def calibration_states(
    robot_x_target_frame_m: float,
    robot_y_target_frame_m: float,
    robot_heading_target_frame_rad: float,
    cfg: dict,
) -> dict:
    start = target_state_from_robot_pose_in_target_frame(
        robot_x_target_frame_m,
        robot_y_target_frame_m,
        robot_heading_target_frame_rad,
    )
    rotate_out, move_distance, move_motion_sign = compute_rotate_out_and_move_distance(
        *start,
        cfg["docking_axis_sign"],
    )
    signed_distance = move_motion_sign * move_distance
    after_rotate = target_state_after_rotation(*start, rotate_out)
    after_move = target_state_after_motion(*after_rotate, signed_distance)
    rotate_back = after_move[2]
    after_rotate_back = target_state_after_rotation(*after_move, rotate_back)
    return {
        "start": start,
        "after_rotate": after_rotate,
        "after_move": after_move,
        "after_rotate_back": after_rotate_back,
        "rotate_out": rotate_out,
        "rotate_back": rotate_back,
        "signed_distance": signed_distance,
        "move_distance": move_distance,
        "move_motion_sign": move_motion_sign,
    }


def sx(panel_left: float, x_m: float) -> float:
    return panel_left + MARGIN + (x_m - X_MIN) / (X_MAX - X_MIN) * (
        PANEL_W - 2 * MARGIN
    )


def sy(panel_top: float, y_m: float) -> float:
    return panel_top + PANEL_H - MARGIN - (y_m - Y_MIN) / (Y_MAX - Y_MIN) * (
        PANEL_H - 2 * MARGIN
    )


def esc(text: str) -> str:
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def line(x1, y1, x2, y2, stroke="#000", width=2, dash=None):
    attrs = [
        f'x1="{x1:.1f}"',
        f'y1="{y1:.1f}"',
        f'x2="{x2:.1f}"',
        f'y2="{y2:.1f}"',
        f'stroke="{stroke}"',
        f'stroke-width="{width}"',
        'fill="none"',
    ]
    if dash:
        attrs.append(f'stroke-dasharray="{dash}"')
    return f'<line {" ".join(attrs)} />'


def rect(x, y, w, h, fill="#fff", stroke="#000", width=1, radius=6, opacity=1.0):
    return (
        f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" '
        f'rx="{radius}" ry="{radius}" fill="{fill}" stroke="{stroke}" '
        f'stroke-width="{width}" opacity="{opacity}" />'
    )


def circle(cx, cy, r, fill, stroke="#000", width=1, opacity=1.0):
    return (
        f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r}" fill="{fill}" '
        f'stroke="{stroke}" stroke-width="{width}" opacity="{opacity}" />'
    )


def polygon(points, fill="#000", stroke="#000", width=1, opacity=1.0):
    pts = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    return (
        f'<polygon points="{pts}" fill="{fill}" stroke="{stroke}" '
        f'stroke-width="{width}" opacity="{opacity}" />'
    )


def text(x, y, body, size=12, fill="#000", anchor="start", weight="normal"):
    return (
        f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{fill}" '
        f'text-anchor="{anchor}" font-weight="{weight}" font-family="Arial">{esc(body)}</text>'
    )


def multiline_text(x, y, lines, size=9, fill="#333", dy=12):
    parts = [
        f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{fill}" '
        'font-family="Courier New" xml:space="preserve">'
    ]
    for idx, item in enumerate(lines):
        parts.append(f'<tspan x="{x:.1f}" dy="{0 if idx == 0 else dy}">{esc(item)}</tspan>')
    parts.append("</text>")
    return "".join(parts)


def robot_triangle(
    panel_left: float,
    panel_top: float,
    x_m: float,
    y_m: float,
    heading_rad: float,
    color: str,
    opacity: float = 1.0,
) -> str:
    px = sx(panel_left, x_m)
    py = sy(panel_top, y_m)
    size = 10.0
    points = [
        (px + size * math.cos(heading_rad), py - size * math.sin(heading_rad)),
        (
            px - 0.76 * size * math.cos(heading_rad) - 0.56 * size * math.sin(heading_rad),
            py + 0.76 * size * math.sin(heading_rad) - 0.56 * size * math.cos(heading_rad),
        ),
        (
            px - 0.76 * size * math.cos(heading_rad) + 0.56 * size * math.sin(heading_rad),
            py + 0.76 * size * math.sin(heading_rad) + 0.56 * size * math.cos(heading_rad),
        ),
    ]
    return polygon(points, fill=color, stroke=color, width=1, opacity=opacity)


def draw_cart(panel_left: float, panel_top: float, cfg: dict) -> list[str]:
    axis_sign = cfg["docking_axis_sign"]
    face_x = -axis_sign * cfg["target_x_offset_m"]
    cart_center_x = face_x - axis_sign * CART_LENGTH_M / 2.0
    left = sx(panel_left, cart_center_x - CART_LENGTH_M / 2.0)
    right = sx(panel_left, cart_center_x + CART_LENGTH_M / 2.0)
    top = sy(panel_top, CART_WIDTH_M / 2.0)
    bottom = sy(panel_top, -CART_WIDTH_M / 2.0)
    face_px = sx(panel_left, face_x)
    target_px = sx(panel_left, 0.0)
    target_py = sy(panel_top, 0.0)
    return [
        rect(
            min(left, right),
            min(top, bottom),
            abs(right - left),
            abs(bottom - top),
            fill=cfg["cart_fill"],
            stroke="#777777",
            width=1,
            radius=2,
            opacity=0.58,
        ),
        line(face_px, target_py, target_px, target_py, stroke="#777777", width=1.2, dash="4 4"),
        circle(target_px, target_py, 5, "#ff7f0e", stroke="#ffffff", width=2),
    ]


def panel_frame(panel_left: float, panel_top: float, title: str, cfg: dict) -> list[str]:
    x_axis_y = sy(panel_top, 0.0)
    y_axis_x = sx(panel_left, 0.0)
    return [
        rect(panel_left, panel_top, PANEL_W, PANEL_H, fill="#fcfcfb", stroke="#dddddd", width=1),
        text(panel_left + PANEL_W / 2, panel_top + 18, title, size=11, anchor="middle", weight="bold"),
        line(panel_left + MARGIN, x_axis_y, panel_left + PANEL_W - MARGIN, x_axis_y, stroke="#d9d9d9", width=0.8),
        line(y_axis_x, panel_top + MARGIN, y_axis_x, panel_top + PANEL_H - MARGIN, stroke="#d9d9d9", width=0.8),
        *draw_cart(panel_left, panel_top, cfg),
    ]


def draw_panel(
    panel_left: float,
    panel_top: float,
    title: str,
    state: tuple[float, float, float],
    cfg: dict,
    notes: list[str],
    projection_state: tuple[float, float, float] | None = None,
) -> str:
    color = cfg["color"]
    robot_x, robot_y, robot_heading = robot_pose_in_target_frame(*state)
    parts = panel_frame(panel_left, panel_top, title, cfg)
    if projection_state is not None:
        proj_x, proj_y, proj_heading = robot_pose_in_target_frame(*projection_state)
        parts.append(
            line(
                sx(panel_left, robot_x),
                sy(panel_top, robot_y),
                sx(panel_left, proj_x),
                sy(panel_top, proj_y),
                stroke=color,
                width=1.8,
                dash="6 4",
            )
        )
        parts.append(robot_triangle(panel_left, panel_top, proj_x, proj_y, proj_heading, color, opacity=0.28))
    parts.append(robot_triangle(panel_left, panel_top, robot_x, robot_y, robot_heading, color))
    parts.append(circle(sx(panel_left, robot_x), sy(panel_top, robot_y), 2.5, "#111111", stroke="#111111"))
    parts.append(multiline_text(panel_left + 10, panel_top + PANEL_H - 36, notes, size=8.4, fill="#333333", dy=11))
    return "\n".join(parts)


def draw_quadrant_row(row_idx: int, quadrant: tuple[str, float, float, float]) -> str:
    q_name, robot_x, robot_y, robot_heading = quadrant
    top = HEADER_H + GAP_Y + row_idx * (PANEL_H + GAP_Y)
    left0 = ROW_LABEL_W + GAP_X
    parts = [
        text(22, top + PANEL_H / 2 - 10, q_name, size=18, anchor="middle", weight="bold"),
        text(22, top + PANEL_H / 2 + 12, f"x={robot_x:+.2f}", size=9, anchor="middle", fill="#555555"),
        text(22, top + PANEL_H / 2 + 26, f"y={robot_y:+.2f}", size=9, anchor="middle", fill="#555555"),
    ]
    for profile_idx, (name, cfg) in enumerate(PROFILES.items()):
        states = calibration_states(robot_x, robot_y, robot_heading, cfg)
        projection_from_rotate = target_state_after_motion(
            *states["after_rotate"],
            states["signed_distance"],
        )
        panel_base = profile_idx * 4
        lefts = [
            left0 + (panel_base + idx) * (PANEL_W + GAP_X)
            for idx in range(4)
        ]
        moved_pose = robot_pose_in_target_frame(*states["after_move"])
        final_pose = robot_pose_in_target_frame(*states["after_rotate_back"])
        if abs(moved_pose[1]) > 1.0e-6:
            raise RuntimeError(f"{name} {q_name} calibration move did not reach target x-axis")
        if abs(states["after_rotate_back"][2]) > 1.0e-6:
            raise RuntimeError(f"{name} {q_name} calibration did not clear yaw error")

        parts.extend(
            [
                draw_panel(
                    lefts[0],
                    top,
                    f"{name} 1 start",
                    states["start"],
                    cfg,
                    [
                        "target centered",
                        f"yaw={math.degrees(states['start'][2]):+.0f}deg",
                    ],
                ),
                draw_panel(
                    lefts[1],
                    top,
                    f"{name} 2 rotate",
                    states["after_rotate"],
                    cfg,
                    [
                        f"turn={math.degrees(states['rotate_out']):+.1f}deg",
                        f"faint=after {states['move_distance']:.2f}m",
                    ],
                    projection_state=projection_from_rotate,
                ),
                draw_panel(
                    lefts[2],
                    top,
                    f"{name} 3 move",
                    states["after_move"],
                    cfg,
                    [
                        "on target x-axis",
                        f"move={states['move_distance']:.2f}m",
                        f"y_T={moved_pose[1]:+.3f}m",
                    ],
                ),
                draw_panel(
                    lefts[3],
                    top,
                    f"{name} 4 yaw",
                    states["after_rotate_back"],
                    cfg,
                    [
                        f"turn={math.degrees(states['rotate_back']):+.1f}deg",
                        f"final yaw={math.degrees(states['after_rotate_back'][2]):+.0f}deg",
                    ],
                ),
            ]
        )
        parts.append(
            line(
                lefts[-1] + PANEL_W + GAP_X / 2,
                top + 10,
                lefts[-1] + PANEL_W + GAP_X / 2,
                top + PANEL_H - 10,
                stroke="#eeeeee",
                width=1,
            )
        )
    return "\n".join(parts)


def build_svg() -> str:
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{CANVAS_W}" height="{CANVAS_H}" viewBox="0 0 {CANVAS_W} {CANVAS_H}">',
        rect(0, 0, CANVAS_W, CANVAS_H, fill="#ffffff", stroke="#ffffff", width=0, radius=0),
        text(CANVAS_W / 2, 28, "RL_special Calibration Quadrant Sweep", size=23, anchor="middle", weight="bold"),
        text(
            CANVAS_W / 2,
            51,
            "Each panel is centered on the offset-applied target. Cart rectangle, target dot, and quadrant axes stay fixed.",
            size=13,
            fill="#555555",
            anchor="middle",
        ),
    ]
    for row_idx, quadrant in enumerate(QUADRANTS):
        parts.append(draw_quadrant_row(row_idx, quadrant))
    parts.append("</svg>")
    return "\n".join(parts)


def main() -> None:
    out_path = Path(__file__).with_name("current_calibration_process.svg")
    out_path.write_text(build_svg(), encoding="utf-8")
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()
