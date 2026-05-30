#!/usr/bin/env python3
import math
from pathlib import Path


PROFILES = {
    "rear": {
        "color": "#d62728",
        "target_x_local_m": 0.14,
        "target_y_local_m": 0.08,
        "target_theta_vision_deg": 8.0,
        "calibration_escape_motion_sign": -1.0,
        "calibration_escape_distance_m": 0.30,
    },
    "front": {
        "color": "#1f77b4",
        "target_x_local_m": -0.14,
        "target_y_local_m": 0.05,
        "target_theta_vision_deg": 8.0,
        "calibration_escape_motion_sign": 1.0,
        "calibration_escape_distance_m": 0.30,
    },
}

PANEL_W = 360
PANEL_H = 300
HEADER_H = 64
GAP_X = 24
GAP_Y = 48
MARGIN = 42
CANVAS_W = PANEL_W * 4 + GAP_X * 5
CANVAS_H = HEADER_H + PANEL_H * 2 + GAP_Y * 3
X_MIN = -1.20
X_MAX = 1.20
Y_MIN = -0.90
Y_MAX = 0.90


def sx(panel_left: float, x_m: float) -> float:
    usable_w = PANEL_W - 2 * MARGIN
    return panel_left + MARGIN + (x_m - X_MIN) / (X_MAX - X_MIN) * usable_w


def sy(panel_top: float, y_m: float) -> float:
    usable_h = PANEL_H - 2 * MARGIN
    return panel_top + PANEL_H - MARGIN - (y_m - Y_MIN) / (Y_MAX - Y_MIN) * usable_h


def esc(text: str) -> str:
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def line(x1, y1, x2, y2, stroke="#000", width=2, dash=None, marker_end=None):
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
    if marker_end:
        attrs.append(f'marker-end="url(#{marker_end})"')
    return f'<line {" ".join(attrs)} />'


def rect(x, y, w, h, fill="#fff", stroke="#000", width=1, radius=12):
    return (
        f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" '
        f'rx="{radius}" ry="{radius}" fill="{fill}" stroke="{stroke}" stroke-width="{width}" />'
    )


def circle(cx, cy, r, fill, stroke="#000", width=1):
    return (
        f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r}" fill="{fill}" '
        f'stroke="{stroke}" stroke-width="{width}" />'
    )


def polygon(points, fill="#000", stroke="#000", width=1):
    pts = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    return (
        f'<polygon points="{pts}" fill="{fill}" '
        f'stroke="{stroke}" stroke-width="{width}" />'
    )


def text(x, y, body, size=14, fill="#000", anchor="start", weight="normal"):
    return (
        f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{fill}" '
        f'text-anchor="{anchor}" font-weight="{weight}" font-family="Arial">{esc(body)}</text>'
    )


def multiline_text(x, y, lines, size=12, fill="#000", dy=16, family="Courier New"):
    parts = [
        f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{fill}" font-family="{family}">'
    ]
    for idx, item in enumerate(lines):
        parts.append(f'<tspan x="{x:.1f}" dy="{0 if idx == 0 else dy}">{esc(item)}</tspan>')
    parts.append("</text>")
    return "".join(parts)


def panel_frame(panel_left: float, panel_top: float, title: str, subtitle: str):
    return [
        rect(panel_left, panel_top, PANEL_W, PANEL_H, fill="#fcfcfc", stroke="#dddddd"),
        text(panel_left + PANEL_W / 2, panel_top + 24, title, size=17, anchor="middle", weight="bold"),
        text(panel_left + PANEL_W / 2, panel_top + 46, subtitle, size=11, anchor="middle", fill="#666666"),
    ]


def _triangle_points(
    panel_left: float,
    panel_top: float,
    x_m: float,
    y_m: float,
    heading_rad: float,
):
    ox = sx(panel_left, 0.0)
    oy = sy(panel_top, 0.0)
    tip = (
        sx(panel_left, x_m + 0.14 * math.cos(heading_rad)),
        sy(panel_top, y_m + 0.14 * math.sin(heading_rad)),
    )
    left = (
        sx(panel_left, x_m - 0.08 * math.cos(heading_rad) - 0.05 * math.sin(heading_rad)),
        sy(panel_top, y_m - 0.08 * math.sin(heading_rad) + 0.05 * math.cos(heading_rad)),
    )
    right = (
        sx(panel_left, x_m - 0.08 * math.cos(heading_rad) + 0.05 * math.sin(heading_rad)),
        sy(panel_top, y_m - 0.08 * math.sin(heading_rad) - 0.05 * math.cos(heading_rad)),
    )
    return [tip, left, right]


def draw_robot_frame(panel_left: float, panel_top: float, show_camera_triangle: bool = False):
    parts = [
        polygon(
            _triangle_points(panel_left, panel_top, 0.0, 0.0, 0.0),
            fill="#111111",
            stroke="#111111",
            width=1,
        )
    ]
    if show_camera_triangle:
        parts.append(
            polygon(
                _triangle_points(panel_left, panel_top, 0.0, 0.0, math.pi),
                fill="#ffffff",
                stroke="#555555",
                width=1,
            )
        )
    return parts


def draw_robot_pose(
    panel_left: float,
    panel_top: float,
    robot_x_local: float,
    robot_y_local: float,
    robot_heading_rad: float,
    fill: str,
    stroke: str,
    opacity: float = 1.0,
):
    triangle_points = _triangle_points(
        panel_left,
        panel_top,
        robot_x_local,
        robot_y_local,
        robot_heading_rad,
    )
    return [
        f'<polygon points="{triangle_points[0][0]:.1f},{triangle_points[0][1]:.1f} '
        f'{triangle_points[1][0]:.1f},{triangle_points[1][1]:.1f} '
        f'{triangle_points[2][0]:.1f},{triangle_points[2][1]:.1f}" '
        f'fill="{fill}" stroke="{stroke}" stroke-width="1" opacity="{opacity:.2f}" />'
    ]


def draw_reachable_circle(
    panel_left: float,
    panel_top: float,
    radius_m: float,
    stroke: str,
    opacity: float = 0.35,
):
    ox = sx(panel_left, 0.0)
    oy = sy(panel_top, 0.0)
    rx = abs(sx(panel_left, radius_m) - sx(panel_left, 0.0))
    return [
        f'<circle cx="{ox:.1f}" cy="{oy:.1f}" r="{rx:.1f}" '
        f'fill="none" stroke="{stroke}" stroke-width="1.5" opacity="{opacity:.2f}" stroke-dasharray="6 6" />'
    ]


def draw_target_pose(
    panel_left: float,
    panel_top: float,
    target_x_local: float,
    target_y_local: float,
    target_theta_vision_rad: float,
    color: str,
    show_negative_x_axis: bool = False,
    show_positive_x_axis: bool = False,
):
    px = sx(panel_left, target_x_local)
    py = sy(panel_top, target_y_local)

    tip_x = sx(
        panel_left,
        target_x_local + 0.14 * math.cos(target_theta_vision_rad),
    )
    tip_y = sy(
        panel_top,
        target_y_local + 0.14 * math.sin(target_theta_vision_rad),
    )
    left_x = sx(
        panel_left,
        target_x_local - 0.08 * math.cos(target_theta_vision_rad) - 0.05 * math.sin(target_theta_vision_rad),
    )
    left_y = sy(
        panel_top,
        target_y_local - 0.08 * math.sin(target_theta_vision_rad) + 0.05 * math.cos(target_theta_vision_rad),
    )
    right_x = sx(
        panel_left,
        target_x_local - 0.08 * math.cos(target_theta_vision_rad) + 0.05 * math.sin(target_theta_vision_rad),
    )
    right_y = sy(
        panel_top,
        target_y_local - 0.08 * math.sin(target_theta_vision_rad) - 0.05 * math.cos(target_theta_vision_rad),
    )

    parts = [
        polygon(
            [(tip_x, tip_y), (left_x, left_y), (right_x, right_y)],
            fill=color,
            stroke=color,
            width=1,
        ),
    ]

    if show_negative_x_axis:
        neg_x_axis_end_x = sx(
            panel_left,
            target_x_local - 1.40 * math.cos(target_theta_vision_rad),
        )
        neg_x_axis_end_y = sy(
            panel_top,
            target_y_local - 1.40 * math.sin(target_theta_vision_rad),
        )
        parts.append(
            line(
                px,
                py,
                neg_x_axis_end_x,
                neg_x_axis_end_y,
                stroke=color,
                width=2,
                dash="8 6",
            )
        )

    if show_positive_x_axis:
        pos_x_axis_end_x = sx(
            panel_left,
            target_x_local + 1.40 * math.cos(target_theta_vision_rad),
        )
        pos_x_axis_end_y = sy(
            panel_top,
            target_y_local + 1.40 * math.sin(target_theta_vision_rad),
        )
        parts.append(
            line(
                px,
                py,
                pos_x_axis_end_x,
                pos_x_axis_end_y,
                stroke=color,
                width=2,
                dash="8 6",
            )
        )

    return parts


def apply_robot_motion_to_target(
    target_x_local: float,
    target_y_local: float,
    target_theta_vision_rad: float,
    linear_distance_m: float,
    delta_yaw_rad: float,
) -> tuple[float, float, float]:
    cos_yaw = math.cos(delta_yaw_rad)
    sin_yaw = math.sin(delta_yaw_rad)
    rotated_x = cos_yaw * target_x_local + sin_yaw * target_y_local
    rotated_y = -sin_yaw * target_x_local + cos_yaw * target_y_local

    new_x = rotated_x - linear_distance_m
    new_y = rotated_y
    new_theta = wrap_to_pi(target_theta_vision_rad - delta_yaw_rad)
    return new_x, new_y, new_theta


def robot_in_target_frame(
    target_x_local: float,
    target_y_local: float,
    target_theta_vision_rad: float,
) -> tuple[float, float, float]:
    cos_theta = math.cos(target_theta_vision_rad)
    sin_theta = math.sin(target_theta_vision_rad)
    robot_x_t = -(cos_theta * target_x_local + sin_theta * target_y_local)
    robot_y_t = sin_theta * target_x_local - cos_theta * target_y_local
    robot_heading_t = -target_theta_vision_rad
    return robot_x_t, robot_y_t, robot_heading_t


def compute_rotate_out(
    target_x_local: float,
    target_y_local: float,
    target_theta_vision_rad: float,
    motion_sign: float,
    distance_m: float,
) -> tuple[float, float]:
    signed_motion_distance_m = motion_sign * distance_m
    if abs(signed_motion_distance_m) <= 1.0e-9:
        return 0.0, 0.0

    robot_x_t, robot_y_t, robot_heading_t = robot_in_target_frame(
        target_x_local,
        target_y_local,
        target_theta_vision_rad,
    )

    longitudinal_sq = (
        signed_motion_distance_m * signed_motion_distance_m
        - robot_y_t * robot_y_t
    )
    if longitudinal_sq < 0.0:
        longitudinal_sq = 0.0

    delta_x_t_mag = math.sqrt(longitudinal_sq)
    x_axis_sign = 1.0 if motion_sign > 0.0 else -1.0
    candidates = (
        robot_x_t - delta_x_t_mag,
        robot_x_t + delta_x_t_mag,
    )
    valid_candidates = [
        candidate
        for candidate in candidates
        if candidate * x_axis_sign >= -1.0e-9
    ]
    if valid_candidates:
        moved_robot_x_t = (
            max(valid_candidates)
            if x_axis_sign > 0.0
            else min(valid_candidates)
        )
    else:
        moved_robot_x_t = (
            max(candidates)
            if x_axis_sign > 0.0
            else min(candidates)
        )

    delta_x_t = moved_robot_x_t - robot_x_t
    delta_y_t = -robot_y_t
    move_heading_t = math.atan2(
        delta_y_t / signed_motion_distance_m,
        delta_x_t / signed_motion_distance_m,
    )
    return wrap_to_pi(move_heading_t - robot_heading_t), moved_robot_x_t


def profile_row(row_idx: int, name: str, cfg: dict):
    top = HEADER_H + GAP_Y + row_idx * (PANEL_H + GAP_Y)
    lefts = [GAP_X * (i + 1) + PANEL_W * i for i in range(4)]
    color = cfg["color"]
    x0 = cfg["target_x_local_m"]
    y0 = cfg["target_y_local_m"]
    theta0 = math.radians(cfg["target_theta_vision_deg"])
    motion_sign = cfg["calibration_escape_motion_sign"]
    distance = cfg["calibration_escape_distance_m"]
    use_positive_axis = name == "front"

    alpha, moved_robot_x_t = compute_rotate_out(x0, y0, theta0, motion_sign, distance)
    x1, y1, theta1 = apply_robot_motion_to_target(
        x0,
        y0,
        theta0,
        linear_distance_m=0.0,
        delta_yaw_rad=alpha,
    )
    x2, y2, theta2 = apply_robot_motion_to_target(
        x1,
        y1,
        theta1,
        linear_distance_m=motion_sign * distance,
        delta_yaw_rad=0.0,
    )
    beta = wrap_to_pi(theta2)
    x3, y3, theta3 = apply_robot_motion_to_target(
        x2,
        y2,
        theta2,
        linear_distance_m=0.0,
        delta_yaw_rad=beta,
    )

    parts = [text(14, top + PANEL_H / 2, name.upper(), size=20, fill=color, weight="bold")]
    robot_x_t0, robot_y_t0, _ = robot_in_target_frame(x0, y0, theta0)
    axis_endpoint_x_local = x1 + moved_robot_x_t * math.cos(theta1)
    axis_endpoint_y_local = y1 + moved_robot_x_t * math.sin(theta1)
    ghost_x1 = motion_sign * distance * math.cos(alpha)
    ghost_y1 = motion_sign * distance * math.sin(alpha)
    ghost_heading1 = alpha

    titles = [
        "1/4 Start",
        "2/4 After First Turn",
        "3/4 After 30cm Move",
        "4/4 After Final Turn",
    ]
    subtitles = [
        "Target axis must already pass through the 30cm circle",
        "Rotate to face the farther axis-circle intersection",
        "Robot reaches the target axis after the 30cm move",
        "Rotate by beta so yaw error becomes 0",
    ]

    for idx, left in enumerate(lefts):
        parts.extend(panel_frame(left, top, titles[idx], subtitles[idx]))
        parts.extend(draw_robot_frame(left, top, show_camera_triangle=use_positive_axis))

    parts.extend(
        draw_target_pose(
            lefts[0],
            top,
            x0,
            y0,
            theta0,
            color,
            show_negative_x_axis=not use_positive_axis,
            show_positive_x_axis=use_positive_axis,
        )
    )
    parts.extend(draw_reachable_circle(lefts[0], top, distance, color))
    parts.append(
        multiline_text(
            lefts[0] + 16,
            top + PANEL_H - 102,
            [
                f"x={x0:+.3f} m",
                f"y={y0:+.3f} m",
                f"theta_vision={math.degrees(theta0):+.1f} deg",
                f"robot in target frame: x_T={robot_x_t0:+.3f}, y_T={robot_y_t0:+.3f}",
            ],
            fill=color,
        )
    )

    parts.extend(
        draw_target_pose(
            lefts[1],
            top,
            x1,
            y1,
            theta1,
            color,
            show_negative_x_axis=not use_positive_axis,
            show_positive_x_axis=use_positive_axis,
        )
    )
    parts.extend(draw_reachable_circle(lefts[1], top, distance, color))
    parts.append(
        circle(
            sx(lefts[1], axis_endpoint_x_local),
            sy(top, axis_endpoint_y_local),
            4,
            color,
            stroke=color,
            width=1,
        )
    )
    parts.extend(
        draw_robot_pose(
            lefts[1],
            top,
            ghost_x1,
            ghost_y1,
            ghost_heading1,
            fill=color,
            stroke=color,
            opacity=0.25,
        )
    )
    parts.append(
        line(
            sx(lefts[1], 0.0),
            sy(top, 0.0),
            sx(lefts[1], ghost_x1),
            sy(top, ghost_y1),
            stroke=color,
            width=1.5,
            dash="4 4",
        )
    )
    parts.append(
        multiline_text(
            lefts[1] + 16,
            top + PANEL_H - 118,
            [
                f"theta={math.degrees(alpha):+.1f} deg",
                f"x={x1:+.3f} m",
                f"y={y1:+.3f} m",
                f"theta_vision={math.degrees(theta1):+.1f} deg",
                "ghost triangle = farther axis-circle intersection",
            ],
            fill=color,
        )
    )

    parts.extend(
        draw_target_pose(
            lefts[2],
            top,
            x2,
            y2,
            theta2,
            "#2ca02c",
            show_negative_x_axis=not use_positive_axis,
            show_positive_x_axis=use_positive_axis,
        )
    )
    parts.append(
        multiline_text(
            lefts[2] + 16,
            top + PANEL_H - 118,
            [
                f"move={motion_sign * distance:+.2f} m",
                f"x={x2:+.3f} m",
                f"y={y2:+.3f} m",
                f"theta_vision={math.degrees(theta2):+.1f} deg",
                "robot is now on the target axis, but yaw is not aligned yet",
            ],
            fill="#2ca02c",
        )
    )

    parts.extend(
        draw_target_pose(
            lefts[3],
            top,
            x3,
            y3,
            theta3,
            color,
            show_negative_x_axis=not use_positive_axis,
            show_positive_x_axis=use_positive_axis,
        )
    )
    parts.append(
        multiline_text(
            lefts[3] + 16,
            top + PANEL_H - 118,
            [
                f"beta={math.degrees(beta):+.1f} deg",
                f"x={x3:+.3f} m",
                f"y={y3:+.3f} m",
                f"theta_vision={math.degrees(theta3):+.1f} deg",
                "goal here: yaw error = 0",
            ],
            fill=color,
        )
    )

    return "\n".join(parts)


def build_svg() -> str:
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{CANVAS_W}" height="{CANVAS_H}" viewBox="0 0 {CANVAS_W} {CANVAS_H}">',
        "<defs>",
        '<marker id="arrow" markerWidth="12" markerHeight="12" refX="10" refY="6" orient="auto" markerUnits="strokeWidth">',
        '<path d="M0,0 L12,6 L0,12 z" fill="#333333" />',
        "</marker>",
        "</defs>",
        rect(0, 0, CANVAS_W, CANVAS_H, fill="#ffffff", stroke="none", width=0, radius=0),
        text(CANVAS_W / 2, 30, "RL_special Calibration In Robot-Centered Frame", size=24, anchor="middle", weight="bold"),
        text(
            CANVAS_W / 2,
            52,
            "Robot axle center stays fixed at the origin. The target pose shows what the robot sees after theta, after the 30 cm move, and after beta.",
            size=12,
            anchor="middle",
            fill="#555555",
        ),
    ]
    for idx, (name, cfg) in enumerate(PROFILES.items()):
        parts.append(profile_row(idx, name, cfg))
    parts.append("</svg>")
    return "\n".join(parts)


def main() -> None:
    out_path = Path(__file__).with_name("current_calibration_process.svg")
    out_path.write_text(build_svg(), encoding="utf-8")
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()
