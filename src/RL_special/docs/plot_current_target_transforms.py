#!/usr/bin/env python3
import math
from pathlib import Path


PROFILES = {
    "front": {
        "base_link_to_axle_center_x_m": 0.095,
        "target_x_offset_m": 0.55,
        "raw_target_x_m": 0.85,
        "raw_target_y_m": 0.22,
        "theta_deg": 25.0,
        "color": "#1f77b4",
    },
    "rear": {
        "base_link_to_axle_center_x_m": 0.120,
        "target_x_offset_m": 0.50,
        "raw_target_x_m": 0.85,
        "raw_target_y_m": 0.22,
        "theta_deg": -25.0,
        "color": "#d62728",
    },
}

PANEL_W = 420
PANEL_H = 360
MARGIN = 52
GAP_X = 30
GAP_Y = 50
HEADER_H = 60
CANVAS_W = PANEL_W * 3 + GAP_X * 4
CANVAS_H = HEADER_H + PANEL_H * 2 + GAP_Y * 3
X_MIN = -1.20
X_MAX = 1.05
Y_MIN = -0.65
Y_MAX = 0.65


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


def circle(cx, cy, r, fill, stroke="#000", width=1):
    return (
        f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r}" fill="{fill}" '
        f'stroke="{stroke}" stroke-width="{width}" />'
    )


def rect(x, y, w, h, fill="#fff", stroke="#000", width=1, radius=10):
    return (
        f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" '
        f'rx="{radius}" ry="{radius}" fill="{fill}" stroke="{stroke}" stroke-width="{width}" />'
    )


def text(x, y, body, size=14, fill="#000", anchor="start", weight="normal"):
    return (
        f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{fill}" '
        f'text-anchor="{anchor}" font-weight="{weight}" font-family="Arial">{esc(body)}</text>'
    )


def multiline_text(x, y, lines, size=12, fill="#000", dy=17):
    parts = [
        f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{fill}" '
        'font-family="Courier New" xml:space="preserve">'
    ]
    for idx, item in enumerate(lines):
        parts.append(f'<tspan x="{x:.1f}" dy="{0 if idx == 0 else dy}">{esc(item)}</tspan>')
    parts.append("</text>")
    return "".join(parts)


def panel_frame(panel_left: float, panel_top: float, title: str, subtitle: str) -> list[str]:
    x_axis_y = sy(panel_top, 0.0)
    y_axis_x = sx(panel_left, 0.0)
    return [
        rect(panel_left, panel_top, PANEL_W, PANEL_H, fill="#fcfcfc", stroke="#dddddd"),
        text(panel_left + PANEL_W / 2, panel_top + 26, title, size=18, anchor="middle", weight="bold"),
        text(panel_left + PANEL_W / 2, panel_top + 47, subtitle, size=12, anchor="middle", fill="#666666"),
        line(panel_left + MARGIN, x_axis_y, panel_left + PANEL_W - MARGIN, x_axis_y, stroke="#cfcfcf", width=1),
        line(y_axis_x, panel_top + MARGIN, y_axis_x, panel_top + PANEL_H - MARGIN, stroke="#cfcfcf", width=1),
        text(panel_left + PANEL_W - 28, x_axis_y - 8, "+x", size=12, anchor="end", fill="#666666"),
        text(y_axis_x + 8, panel_top + MARGIN + 12, "+y", size=12, fill="#666666"),
    ]


def target_marker(
    panel_left: float,
    panel_top: float,
    x_m: float,
    y_m: float,
    theta_rad: float,
    color: str,
    label: str,
) -> list[str]:
    px = sx(panel_left, x_m)
    py = sy(panel_top, y_m)
    hx = sx(panel_left, x_m + 0.16 * math.cos(theta_rad))
    hy = sy(panel_top, y_m + 0.16 * math.sin(theta_rad))
    return [
        circle(px, py, 7, color, stroke="#ffffff", width=2),
        text(px + 10, py - 9, label, size=12, fill=color),
        line(px, py, hx, hy, stroke=color, width=2.5, marker_end="arrow"),
    ]


def canonical_values(name: str, cfg: dict) -> dict:
    theta = math.radians(cfg["theta_deg"])
    raw_x = cfg["raw_target_x_m"]
    raw_y = cfg["raw_target_y_m"]

    if name == "front":
        base_x = -raw_x
        base_y = -raw_y
    else:
        base_x = raw_x
        base_y = raw_y

    axle_x = base_x - cfg["base_link_to_axle_center_x_m"]
    axle_y = base_y
    cart_offset_sign = 1.0 if name == "front" else -1.0
    target_x = (
        axle_x
        + cart_offset_sign * cfg["target_x_offset_m"] * math.cos(theta)
    )
    target_y = (
        axle_y
        + cart_offset_sign * cfg["target_x_offset_m"] * math.sin(theta)
    )
    yaw_error = math.atan2(math.sin(-theta), math.cos(-theta))

    return {
        "theta": theta,
        "raw_x": raw_x,
        "raw_y": raw_y,
        "base_x": base_x,
        "base_y": base_y,
        "axle_x": axle_x,
        "axle_y": axle_y,
        "target_x": target_x,
        "target_y": target_y,
        "yaw_error": yaw_error,
        "cart_offset_sign": cart_offset_sign,
    }


def profile_row(row_idx: int, name: str, cfg: dict) -> str:
    panel_top = HEADER_H + GAP_Y + row_idx * (PANEL_H + GAP_Y)
    values = canonical_values(name, cfg)
    color = cfg["color"]

    panel1_left = GAP_X
    panel2_left = panel1_left + PANEL_W + GAP_X
    panel3_left = panel2_left + PANEL_W + GAP_X

    parts: list[str] = []
    parts.append(text(18, panel_top + PANEL_H / 2, name.upper(), size=20, fill=color, weight="bold"))

    parts.extend(panel_frame(panel1_left, panel_top, f"{name.capitalize()} 1/3", "Vision target in camera/base_link frame"))
    parts.append(circle(sx(panel1_left, 0.0), sy(panel_top, 0.0), 6, "#000000"))
    parts.extend(target_marker(panel1_left, panel_top, values["raw_x"], values["raw_y"], values["theta"], color, "vision target"))
    parts.append(
        multiline_text(
            panel1_left + 18,
            panel_top + PANEL_H - 84,
            [
                "Pose2D from vision",
                f"x = {values['raw_x']:.3f} m",
                f"y = {values['raw_y']:.3f} m",
                f"theta = {math.degrees(values['theta']):.1f} deg",
                "front only: x,y signs are flipped",
            ],
        )
    )

    parts.extend(panel_frame(panel2_left, panel_top, f"{name.capitalize()} 2/3", "Canonical robot-local target before cart offset"))
    parts.append(circle(sx(panel2_left, 0.0), sy(panel_top, 0.0), 6, "#000000"))
    parts.append(text(sx(panel2_left, 0.0), sy(panel_top, 0.0) - 12, "axle origin", size=12, anchor="middle"))
    parts.extend(target_marker(panel2_left, panel_top, values["axle_x"], values["axle_y"], values["theta"], "#2ca02c", "axle target"))
    parts.append(
        multiline_text(
            panel2_left + 18,
            panel_top + PANEL_H - 98,
            [
                "Normalize to robot frame",
                f"x_base = {values['base_x']:.3f} m",
                f"y_base = {values['base_y']:.3f} m",
                f"x_axle = x_base - {cfg['base_link_to_axle_center_x_m']:.3f}",
                f"x_axle = {values['axle_x']:.3f} m",
                f"y_axle = {values['axle_y']:.3f} m",
            ],
        )
    )

    parts.extend(panel_frame(panel3_left, panel_top, f"{name.capitalize()} 3/3", "Canonical policy/align/calibration target"))
    parts.append(circle(sx(panel3_left, 0.0), sy(panel_top, 0.0), 6, "#000000"))
    parts.append(text(sx(panel3_left, 0.0), sy(panel_top, 0.0) - 12, "axle origin", size=12, anchor="middle"))
    parts.extend(target_marker(panel3_left, panel_top, values["axle_x"], values["axle_y"], values["theta"], "#aaaaaa", "axle target"))
    parts.extend(target_marker(panel3_left, panel_top, values["target_x"], values["target_y"], values["theta"], "#ff7f0e", "canonical target"))
    parts.append(
        line(
            sx(panel3_left, values["axle_x"]),
            sy(panel_top, values["axle_y"]),
            sx(panel3_left, values["target_x"]),
            sy(panel_top, values["target_y"]),
            stroke="#ff7f0e",
            width=2.5,
            dash="7 5",
            marker_end="arrow",
        )
    )
    parts.append(
        multiline_text(
            panel3_left + 18,
            panel_top + PANEL_H - 116,
            [
                "Shared canonical state",
                (
                    "x_target = x_axle + d cos(theta)"
                    if values["cart_offset_sign"] > 0.0
                    else "x_target = x_axle - d cos(theta)"
                ),
                (
                    "y_target = y_axle + d sin(theta)"
                    if values["cart_offset_sign"] > 0.0
                    else "y_target = y_axle - d sin(theta)"
                ),
                "yaw_error = wrap_to_pi(-theta)",
                f"x_target = {values['target_x']:.3f} m",
                f"y_target = {values['target_y']:.3f} m",
                f"yaw_error = {math.degrees(values['yaw_error']):.1f} deg",
            ],
        )
    )

    return "\n".join(parts)


def main() -> None:
    output_path = Path(__file__).with_name("current_target_transforms.svg")
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{CANVAS_W}" height="{CANVAS_H}" viewBox="0 0 {CANVAS_W} {CANVAS_H}">',
        "<defs>",
        '<marker id="arrow" markerWidth="8" markerHeight="8" refX="4" refY="4" orient="auto" markerUnits="strokeWidth">',
        '<path d="M 0 0 L 8 4 L 0 8 z" fill="#555555" />',
        "</marker>",
        "</defs>",
        rect(0, 0, CANVAS_W, CANVAS_H, fill="#ffffff", stroke="#ffffff", width=0, radius=0),
        text(CANVAS_W / 2, 30, "RL_special canonical target transform", size=24, anchor="middle", weight="bold"),
        text(CANVAS_W / 2, 51, "After this transform, policy, align, calibration, and odometry all use the same target state.", size=13, anchor="middle", fill="#555555"),
    ]
    for row_idx, (name, cfg) in enumerate(PROFILES.items()):
        svg.append(profile_row(row_idx, name, cfg))
    svg.append("</svg>")
    output_path.write_text("\n".join(svg), encoding="utf-8")
    print(output_path)


if __name__ == "__main__":
    main()
