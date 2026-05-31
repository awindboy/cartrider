#!/usr/bin/env python3
import math
from pathlib import Path


PROFILES = {
    "rear": {
        "color": "#d62728",
        "base_link_to_axle_center_x_m": 0.120,
        "target_x_offset_m": 0.50,
        "raw_target_x_m": 0.30,
        "raw_target_y_m": 0.35,
        "theta_deg": -25.0,
        "calibration_escape_motion_sign": -1.0,
    },
    "front": {
        "color": "#1f77b4",
        "base_link_to_axle_center_x_m": 0.095,
        "target_x_offset_m": 0.55,
        "raw_target_x_m": 0.30,
        "raw_target_y_m": 0.05,
        "theta_deg": 25.0,
        "calibration_escape_motion_sign": 1.0,
    },
}

ESCAPE_DISTANCE_M = 0.30
PANEL_W = 300
PANEL_H = 300
MARGIN = 42
GAP_X = 24
GAP_Y = 44
HEADER_H = 66
CANVAS_W = PANEL_W * 4 + GAP_X * 5
CANVAS_H = HEADER_H + PANEL_H * 2 + GAP_Y * 3
X_MIN = -0.62
X_MAX = 0.62
Y_MIN = -0.46
Y_MAX = 0.46


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
    return (
        robot_x_target_frame_m,
        robot_y_target_frame_m,
        robot_heading_target_frame_rad,
    )


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


def target_state_after_calibration_motion(
    target_x_local: float,
    target_y_local: float,
    target_yaw_error_rad: float,
    rotate_out_rad: float,
    signed_motion_distance_m: float,
) -> tuple[float, float, float]:
    rotated = target_state_after_rotation(
        target_x_local,
        target_y_local,
        target_yaw_error_rad,
        rotate_out_rad,
    )
    return target_state_after_motion(*rotated, signed_motion_distance_m)


def compute_rotate_out_target_rad(
    target_x_local: float,
    target_y_local: float,
    target_yaw_error_rad: float,
    calibration_escape_motion_sign: float,
    calibration_escape_distance_m: float,
) -> float:
    signed_motion_distance_m = (
        calibration_escape_motion_sign * calibration_escape_distance_m
    )
    if abs(signed_motion_distance_m) <= 1.0e-9:
        return 0.0
    x_axis_sign = 1.0 if calibration_escape_motion_sign > 0.0 else -1.0

    def axis_error(candidate_rad: float) -> float:
        moved_state = target_state_after_calibration_motion(
            target_x_local,
            target_y_local,
            target_yaw_error_rad,
            candidate_rad,
            signed_motion_distance_m,
        )
        _, robot_y_target_frame_m, _ = robot_pose_in_target_frame(*moved_state)
        return robot_y_target_frame_m

    roots: list[float] = []
    sample_count = 1440
    previous_angle = -math.pi
    previous_error = axis_error(previous_angle)
    if abs(previous_error) <= 1.0e-9:
        roots.append(previous_angle)

    for sample_idx in range(1, sample_count + 1):
        angle = -math.pi + (2.0 * math.pi * sample_idx / sample_count)
        error = axis_error(angle)
        if abs(error) <= 1.0e-9:
            roots.append(angle)
        elif previous_error * error < 0.0:
            low = previous_angle
            high = angle
            low_error = previous_error
            for _ in range(48):
                mid = 0.5 * (low + high)
                mid_error = axis_error(mid)
                if abs(mid_error) <= 1.0e-12:
                    low = high = mid
                    break
                if low_error * mid_error <= 0.0:
                    high = mid
                else:
                    low = mid
                    low_error = mid_error
            roots.append(0.5 * (low + high))
        previous_angle = angle
        previous_error = error

    if not roots:
        best_angle = 0.0
        best_error = float("inf")
        for sample_idx in range(sample_count + 1):
            angle = -math.pi + (2.0 * math.pi * sample_idx / sample_count)
            error = abs(axis_error(angle))
            if error < best_error:
                best_error = error
                best_angle = angle
        return wrap_to_pi(best_angle)

    def root_score(candidate_rad: float) -> tuple[int, float, float]:
        moved_state = target_state_after_calibration_motion(
            target_x_local,
            target_y_local,
            target_yaw_error_rad,
            candidate_rad,
            signed_motion_distance_m,
        )
        robot_x_target_frame_m, robot_y_target_frame_m, _ = (
            robot_pose_in_target_frame(*moved_state)
        )
        axis_distance = robot_x_target_frame_m * x_axis_sign
        axis_penalty = 0 if axis_distance >= -1.0e-9 else 1
        if axis_penalty == 0:
            axis_distance_score = -axis_distance
        else:
            axis_distance_score = axis_distance
        return axis_penalty, axis_distance_score, abs(robot_y_target_frame_m)

    return wrap_to_pi(min(roots, key=root_score))


def canonical_target_state(name: str, cfg: dict) -> tuple[float, float, float]:
    theta = math.radians(cfg["theta_deg"])
    base_x = cfg["raw_target_x_m"]
    base_y = cfg["raw_target_y_m"]
    if name == "front":
        base_x *= -1.0
        base_y *= -1.0

    axle_x = base_x - cfg["base_link_to_axle_center_x_m"]
    axle_y = base_y
    yaw_error = wrap_to_pi(-theta)
    cart_offset_sign = 1.0 if name == "front" else -1.0
    target_x = (
        axle_x
        + cart_offset_sign * cfg["target_x_offset_m"] * math.cos(yaw_error)
    )
    target_y = (
        axle_y
        + cart_offset_sign * cfg["target_x_offset_m"] * math.sin(yaw_error)
    )
    return target_x, target_y, yaw_error


def calibration_states(name: str, cfg: dict) -> dict:
    x0, y0, yaw0 = canonical_target_state(name, cfg)
    sign = cfg["calibration_escape_motion_sign"]
    signed_distance = sign * ESCAPE_DISTANCE_M
    rotate_out = compute_rotate_out_target_rad(
        x0,
        y0,
        yaw0,
        sign,
        ESCAPE_DISTANCE_M,
    )
    after_rotate = target_state_after_rotation(x0, y0, yaw0, rotate_out)
    after_move = target_state_after_motion(*after_rotate, signed_distance)
    rotate_back = after_move[2]
    after_rotate_back = target_state_after_rotation(*after_move, rotate_back)
    return {
        "start": (x0, y0, yaw0),
        "after_rotate": after_rotate,
        "after_move": after_move,
        "after_rotate_back": after_rotate_back,
        "rotate_out": rotate_out,
        "rotate_back": rotate_back,
        "signed_distance": signed_distance,
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


def rect(x, y, w, h, fill="#fff", stroke="#000", width=1, radius=8):
    return (
        f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" '
        f'rx="{radius}" ry="{radius}" fill="{fill}" stroke="{stroke}" stroke-width="{width}" />'
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


def text(x, y, body, size=14, fill="#000", anchor="start", weight="normal"):
    return (
        f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{fill}" '
        f'text-anchor="{anchor}" font-weight="{weight}" font-family="Arial">{esc(body)}</text>'
    )


def multiline_text(x, y, lines, size=11, fill="#000", dy=15):
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
    size = 14.0
    points = [
        (px + size * math.cos(heading_rad), py - size * math.sin(heading_rad)),
        (
            px - 0.75 * size * math.cos(heading_rad) - 0.55 * size * math.sin(heading_rad),
            py + 0.75 * size * math.sin(heading_rad) - 0.55 * size * math.cos(heading_rad),
        ),
        (
            px - 0.75 * size * math.cos(heading_rad) + 0.55 * size * math.sin(heading_rad),
            py + 0.75 * size * math.sin(heading_rad) + 0.55 * size * math.cos(heading_rad),
        ),
    ]
    return polygon(points, fill=color, stroke=color, width=1, opacity=opacity)


def panel_frame(panel_left: float, panel_top: float, title: str, axis_sign: float, color: str):
    x_axis_y = sy(panel_top, 0.0)
    y_axis_x = sx(panel_left, 0.0)
    return [
        rect(panel_left, panel_top, PANEL_W, PANEL_H, fill="#fcfcfc", stroke="#dddddd"),
        text(panel_left + PANEL_W / 2, panel_top + 24, title, size=15, anchor="middle", weight="bold"),
        line(panel_left + MARGIN, x_axis_y, panel_left + PANEL_W - MARGIN, x_axis_y, stroke="#d0d0d0", width=1),
        line(y_axis_x, panel_top + MARGIN, y_axis_x, panel_top + PANEL_H - MARGIN, stroke="#d0d0d0", width=1),
        line(y_axis_x, x_axis_y, sx(panel_left, axis_sign * 0.55), x_axis_y, stroke=color, width=1.5),
        circle(y_axis_x, x_axis_y, 7, "#ff7f0e", stroke="#ffffff", width=2),
    ]


def draw_panel(
    panel_left: float,
    panel_top: float,
    title: str,
    state: tuple[float, float, float],
    color: str,
    axis_sign: float,
    notes: list[str],
    projection_state: tuple[float, float, float] | None = None,
) -> str:
    robot_x, robot_y, robot_heading = robot_pose_in_target_frame(*state)
    parts = panel_frame(panel_left, panel_top, title, axis_sign, color)
    if projection_state is not None:
        proj_x, proj_y, proj_heading = robot_pose_in_target_frame(*projection_state)
        parts.append(line(sx(panel_left, robot_x), sy(panel_top, robot_y), sx(panel_left, proj_x), sy(panel_top, proj_y), stroke=color, width=2, dash="7 5"))
        parts.append(robot_triangle(panel_left, panel_top, proj_x, proj_y, proj_heading, color, opacity=0.28))
    parts.append(robot_triangle(panel_left, panel_top, robot_x, robot_y, robot_heading, color))
    parts.append(circle(sx(panel_left, robot_x), sy(panel_top, robot_y), 3, "#111111", stroke="#111111"))
    parts.append(
        multiline_text(
            panel_left + 14,
            panel_top + PANEL_H - 66,
            notes,
            size=10,
            fill="#333333",
            dy=14,
        )
    )
    return "\n".join(parts)


def profile_row(row_idx: int, name: str, cfg: dict) -> str:
    top = HEADER_H + GAP_Y + row_idx * (PANEL_H + GAP_Y)
    states = calibration_states(name, cfg)
    color = cfg["color"]
    axis_sign = 1.0 if cfg["calibration_escape_motion_sign"] > 0.0 else -1.0
    lefts = [GAP_X + idx * (PANEL_W + GAP_X) for idx in range(4)]
    projection_from_rotate = target_state_after_motion(
        *states["after_rotate"],
        states["signed_distance"],
    )

    start_pose = robot_pose_in_target_frame(*states["start"])
    moved_pose = robot_pose_in_target_frame(*states["after_move"])
    final_pose = robot_pose_in_target_frame(*states["after_rotate_back"])

    panels = [
        draw_panel(
            lefts[0],
            top,
            f"{name.upper()} 1. target lost",
            states["start"],
            color,
            axis_sign,
            [
                "canonical target",
                f"start y_T={start_pose[1]:+.3f} m",
                f"yaw_error={math.degrees(states['start'][2]):+.1f} deg",
            ],
        ),
        draw_panel(
            lefts[1],
            top,
            "2. rotate_out",
            states["after_rotate"],
            color,
            axis_sign,
            [
                f"rotate={math.degrees(states['rotate_out']):+.1f} deg",
                "faint robot = expected",
                "position after 30 cm",
            ],
            projection_state=projection_from_rotate,
        ),
        draw_panel(
            lefts[2],
            top,
            "3. move 30 cm",
            states["after_move"],
            color,
            axis_sign,
            [
                f"moved x_T={moved_pose[0]:+.3f} m",
                f"moved y_T={moved_pose[1]:+.6f} m",
                "must be on target x-axis",
            ],
        ),
        draw_panel(
            lefts[3],
            top,
            "4. rotate_back",
            states["after_rotate_back"],
            color,
            axis_sign,
            [
                f"rotate={math.degrees(states['rotate_back']):+.1f} deg",
                f"final y_T={final_pose[1]:+.6f} m",
                f"final yaw={math.degrees(states['after_rotate_back'][2]):+.1f} deg",
            ],
        ),
    ]

    if abs(moved_pose[1]) > 1.0e-6:
        raise RuntimeError(f"{name} calibration move did not reach target x-axis")
    if abs(states["after_rotate_back"][2]) > 1.0e-6:
        raise RuntimeError(f"{name} calibration did not clear yaw error")

    return "\n".join(panels)


def build_svg() -> str:
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{CANVAS_W}" height="{CANVAS_H}" viewBox="0 0 {CANVAS_W} {CANVAS_H}">',
        "<defs>",
        '<marker id="arrow" markerWidth="8" markerHeight="8" refX="4" refY="4" orient="auto" markerUnits="strokeWidth">',
        '<path d="M 0 0 L 8 4 L 0 8 z" fill="#555555" />',
        "</marker>",
        "</defs>",
        rect(0, 0, CANVAS_W, CANVAS_H, fill="#ffffff", stroke="#ffffff", width=0, radius=0),
        text(CANVAS_W / 2, 31, "RL_special Calibration Process", size=24, anchor="middle", weight="bold"),
        text(
            CANVAS_W / 2,
            53,
            "Panels are drawn in the target frame. The orange dot is the target; the colored axis is the docking side.",
            size=13,
            fill="#555555",
            anchor="middle",
        ),
    ]
    for row_idx, (name, cfg) in enumerate(PROFILES.items()):
        parts.append(profile_row(row_idx, name, cfg))
    parts.append("</svg>")
    return "\n".join(parts)


def main() -> None:
    out_path = Path(__file__).with_name("current_calibration_process.svg")
    out_path.write_text(build_svg(), encoding="utf-8")
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()
