#!/usr/bin/env python3
import math
from pathlib import Path


PROFILES = {
    "front": {
        "base_link_to_axle_center_x_m": 0.095,
        "target_x_offset_m": 0.0,
        "raw_target_x_m": 0.85,
        "raw_target_y_m": 0.22,
        "theta_deg": 25.0,
        "color": "#1f77b4",
    },
    "rear": {
        "base_link_to_axle_center_x_m": 0.120,
        "target_x_offset_m": 0.20,
        "raw_target_x_m": 0.85,
        "raw_target_y_m": 0.22,
        "theta_deg": 25.0,
        "color": "#d62728",
    },
}

PANEL_W = 420
PANEL_H = 360
MARGIN = 55
GAP_X = 24
GAP_Y = 48
HEADER_H = 56
CANVAS_W = PANEL_W * 3 + GAP_X * 4
CANVAS_H = HEADER_H + PANEL_H * 2 + GAP_Y * 3
X_MIN = -0.20
X_MAX = 1.15
Y_MIN = -0.30
Y_MAX = 0.55


def sx(panel_left: float, x_m: float) -> float:
    usable_w = PANEL_W - 2 * MARGIN
    return panel_left + MARGIN + (x_m - X_MIN) / (X_MAX - X_MIN) * usable_w


def sy(panel_top: float, y_m: float) -> float:
    usable_h = PANEL_H - 2 * MARGIN
    return panel_top + PANEL_H - MARGIN - (y_m - Y_MIN) / (Y_MAX - Y_MIN) * usable_h


def esc(text: str) -> str:
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def line(x1, y1, x2, y2, stroke="#000", width=2, dash=None, marker_end=None, marker_start=None):
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
    if marker_start:
        attrs.append(f'marker-start="url(#{marker_start})"')
    return f'<line {" ".join(attrs)} />'


def circle(cx, cy, r, fill, stroke="#000", width=1):
    return (
        f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r}" fill="{fill}" '
        f'stroke="{stroke}" stroke-width="{width}" />'
    )


def rect(x, y, w, h, fill="#fff", stroke="#000", width=1, radius=12):
    return (
        f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" '
        f'rx="{radius}" ry="{radius}" fill="{fill}" stroke="{stroke}" stroke-width="{width}" />'
    )


def text(x, y, body, size=14, fill="#000", anchor="start", weight="normal", family="Arial"):
    return (
        f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{fill}" '
        f'text-anchor="{anchor}" font-weight="{weight}" font-family="{family}">{esc(body)}</text>'
    )


def multiline_text(x, y, lines, size=13, fill="#000", family="Courier New", dy=18):
    parts = [
        f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{fill}" '
        f'font-family="{family}" xml:space="preserve">'
    ]
    for idx, line_text in enumerate(lines):
        parts.append(f'<tspan x="{x:.1f}" dy="{0 if idx == 0 else dy}">{esc(line_text)}</tspan>')
    parts.append("</text>")
    return "".join(parts)


def panel_frame(panel_left: float, panel_top: float, title: str, subtitle: str) -> list[str]:
    x_axis_y = sy(panel_top, 0.0)
    y_axis_x = sx(panel_left, 0.0)
    return [
        rect(panel_left, panel_top, PANEL_W, PANEL_H, fill="#fcfcfc", stroke="#dddddd"),
        text(panel_left + PANEL_W / 2, panel_top + 26, title, size=18, anchor="middle", weight="bold"),
        text(panel_left + PANEL_W / 2, panel_top + 48, subtitle, size=12, anchor="middle", fill="#666666"),
        line(panel_left + MARGIN, x_axis_y, panel_left + PANEL_W - MARGIN, x_axis_y, stroke="#cfcfcf", width=1),
        line(y_axis_x, panel_top + MARGIN, y_axis_x, panel_top + PANEL_H - MARGIN, stroke="#cfcfcf", width=1),
        text(panel_left + PANEL_W - 32, x_axis_y - 8, "+x", size=12, anchor="end", fill="#666666"),
        text(y_axis_x + 8, panel_top + MARGIN + 12, "+y", size=12, fill="#666666"),
    ]


def target_with_heading(panel_left: float, panel_top: float, x_m: float, y_m: float, theta_rad: float, color: str, label: str) -> list[str]:
    px = sx(panel_left, x_m)
    py = sy(panel_top, y_m)
    hx = sx(panel_left, x_m + 0.18 * math.cos(theta_rad))
    hy = sy(panel_top, y_m + 0.18 * math.sin(theta_rad))
    return [
        circle(px, py, 7, color),
        text(px + 10, py - 10, label, size=12, fill=color),
        line(px, py, hx, hy, stroke=color, width=2.5, marker_end="arrow"),
        text(hx + 10, hy + 4, f"theta_vision={math.degrees(theta_rad):.0f} deg", size=12, fill=color),
    ]


def profile_row(row_idx: int, name: str, cfg: dict) -> str:
    panel_top = HEADER_H + GAP_Y + row_idx * (PANEL_H + GAP_Y)
    theta = math.radians(cfg["theta_deg"])
    raw_x = cfg["raw_target_x_m"]
    raw_y = cfg["raw_target_y_m"]
    axle_shift = cfg["base_link_to_axle_center_x_m"]
    cart_offset = cfg["target_x_offset_m"]
    axle_x = raw_x - axle_shift
    axle_y = raw_y
    final_x = axle_x - cart_offset * math.cos(theta)
    final_y = axle_y - cart_offset * math.sin(theta)

    parts: list[str] = []

    row_label_y = panel_top + PANEL_H / 2
    parts.append(
        text(
            GAP_X,
            row_label_y,
            f"{name.upper()}",
            size=20,
            fill=cfg["color"],
            weight="bold",
        )
    )

    panel1_left = GAP_X * 2
    panel2_left = panel1_left + PANEL_W + GAP_X
    panel3_left = panel2_left + PANEL_W + GAP_X

    parts.extend(panel_frame(panel1_left, panel_top, f"{name.capitalize()} 1/3", "Vision raw target in base_link frame"))
    parts.append(circle(sx(panel1_left, 0.0), sy(panel_top, 0.0), 6, "#000000"))
    parts.append(text(sx(panel1_left, 0.0), sy(panel_top, 0.0) - 12, "base_link center", size=12, anchor="middle"))
    parts.extend(target_with_heading(panel1_left, panel_top, raw_x, raw_y, theta, cfg["color"], "raw target"))
    parts.append(
        multiline_text(
            panel1_left + 18,
            panel_top + PANEL_H - 78,
            [
                "Input from vision",
                f"x_base = {raw_x:.3f} m",
                f"y_base = {raw_y:.3f} m",
                f"theta_vision = {cfg['theta_deg']:.1f} deg",
            ],
            size=12,
            family="Courier New",
        )
    )

    parts.extend(panel_frame(panel2_left, panel_top, f"{name.capitalize()} 2/3", "Shift origin from base_link to axle center"))
    parts.append(circle(sx(panel2_left, 0.0), sy(panel_top, 0.0), 6, "#000000"))
    parts.append(text(sx(panel2_left, 0.0), sy(panel_top, 0.0) - 12, "base_link center", size=12, anchor="middle"))
    parts.append(circle(sx(panel2_left, axle_shift), sy(panel_top, 0.0), 6, "#2ca02c"))
    parts.append(text(sx(panel2_left, axle_shift), sy(panel_top, 0.0) - 12, "axle center", size=12, fill="#2ca02c", anchor="middle"))
    parts.append(
        line(
            sx(panel2_left, 0.0),
            sy(panel_top, 0.0) + 18,
            sx(panel2_left, axle_shift),
            sy(panel_top, 0.0) + 18,
            stroke="#2ca02c",
            width=2.5,
            marker_start="arrow",
            marker_end="arrow",
        )
    )
    parts.append(
        text(
            0.5 * (sx(panel2_left, 0.0) + sx(panel2_left, axle_shift)),
            sy(panel_top, 0.0) + 40,
            f"{axle_shift:.3f} m",
            size=12,
            fill="#2ca02c",
            anchor="middle",
        )
    )
    parts.extend(target_with_heading(panel2_left, panel_top, axle_x, axle_y, theta, "#2ca02c", "same target in axle frame"))
    parts.append(
        multiline_text(
            panel2_left + 18,
            panel_top + PANEL_H - 78,
            [
                "Origin shift",
                f"x_axle = x_base - {axle_shift:.3f}",
                "y_axle = y_base",
                f"x_axle = {axle_x:.3f} m",
                f"y_axle = {axle_y:.3f} m",
            ],
            size=12,
            family="Courier New",
        )
    )

    parts.extend(panel_frame(panel3_left, panel_top, f"{name.capitalize()} 3/3", "Apply cart longitudinal offset"))
    parts.append(circle(sx(panel3_left, 0.0), sy(panel_top, 0.0), 6, "#2ca02c"))
    parts.append(text(sx(panel3_left, 0.0), sy(panel_top, 0.0) - 12, "axle origin", size=12, fill="#2ca02c", anchor="middle"))
    parts.extend(target_with_heading(panel3_left, panel_top, axle_x, axle_y, theta, "#888888", "axle-frame target"))
    parts.extend(target_with_heading(panel3_left, panel_top, final_x, final_y, theta, "#ff7f0e", "final policy target"))
    if cart_offset > 0.0:
        parts.append(
            line(
                sx(panel3_left, axle_x),
                sy(panel_top, axle_y),
                sx(panel3_left, final_x),
                sy(panel_top, final_y),
                stroke="#ff7f0e",
                width=2.5,
                marker_end="arrow",
            )
        )
        parts.append(
            text(
                0.5 * (sx(panel3_left, axle_x) + sx(panel3_left, final_x)),
                0.5 * (sy(panel_top, axle_y) + sy(panel_top, final_y)) - 10,
                f"offset {cart_offset:.3f} m",
                size=12,
                fill="#ff7f0e",
                anchor="middle",
            )
        )
    parts.append(
        multiline_text(
            panel3_left + 18,
            panel_top + PANEL_H - 96,
            [
                "Final transform",
                "x_target = x_axle - d cos(theta_vision)",
                "y_target = y_axle - d sin(theta_vision)",
                "heading_error = wrap_to_pi(-theta_vision)",
                f"d = {cart_offset:.3f} m",
                f"x_target = {final_x:.3f} m",
                f"y_target = {final_y:.3f} m",
            ],
            size=12,
            family="Courier New",
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
        rect(0, 0, CANVAS_W, CANVAS_H, fill="#ffffff", stroke="#ffffff", width=0),
        text(CANVAS_W / 2, 28, "RL_special target transform pipeline", size=24, anchor="middle", weight="bold"),
        text(
            CANVAS_W / 2,
            48,
            "Each row shows: raw vision target in base_link frame -> shift to axle frame -> apply cart-surface offset",
            size=13,
            anchor="middle",
            fill="#555555",
        ),
    ]
    for row_idx, (name, cfg) in enumerate(PROFILES.items()):
        svg.append(profile_row(row_idx, name, cfg))
    svg.append("</svg>")
    output_path.write_text("\n".join(svg), encoding="utf-8")
    print(output_path)


if __name__ == "__main__":
    main()
