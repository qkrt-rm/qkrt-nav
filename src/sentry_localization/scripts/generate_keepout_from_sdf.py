#!/usr/bin/env python3
"""Generate a Nav2 keepout mask from collision geometry in an SDF world/model.

The script rasterizes SDF collision primitives (box/cylinder/sphere) into a
PGM image aligned to an existing occupancy map YAML's origin/resolution/size,
then writes a keepout mask YAML for nav2_map_server.
"""

import argparse
import math
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

FREE_VAL = 255
OCCUPIED_VAL = 0


@dataclass
class Pose2D:
    x: float
    y: float
    z: float
    yaw: float


def parse_yaml_map(path: Path) -> dict:
    """Parse a minimal ROS map YAML file without external deps."""
    data = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        k, v = line.split(":", 1)
        k = k.strip()
        v = v.strip()
        data[k] = v

    if "image" not in data or "resolution" not in data or "origin" not in data:
        raise ValueError(f"Map YAML missing required keys: {path}")

    image = strip_yaml_string(data["image"])
    resolution = float(strip_yaml_string(data["resolution"]))
    origin_vals = parse_bracket_list(data["origin"])  # [x, y, yaw]
    if len(origin_vals) < 2:
        raise ValueError(f"Invalid origin in map YAML: {data['origin']}")

    return {
        "image": image,
        "resolution": resolution,
        "origin_x": float(origin_vals[0]),
        "origin_y": float(origin_vals[1]),
    }


def strip_yaml_string(v: str) -> str:
    v = v.strip()
    if (v.startswith("\"") and v.endswith("\"")) or (v.startswith("'") and v.endswith("'")):
        return v[1:-1]
    return v


def parse_bracket_list(v: str) -> List[float]:
    text = strip_yaml_string(v)
    text = text.strip()
    if not (text.startswith("[") and text.endswith("]")):
        raise ValueError(f"Expected bracket list, got: {v}")
    inner = text[1:-1].strip()
    if not inner:
        return []
    out = []
    for p in inner.split(","):
        out.append(float(p.strip()))
    return out


def read_image_dimensions(path: Path) -> Tuple[int, int]:
    suffix = path.suffix.lower()
    if suffix == ".png":
        return read_png_dimensions(path)
    if suffix in (".pgm", ".ppm", ".pnm"):
        return read_pnm_dimensions(path)
    raise ValueError(f"Unsupported map image format: {path}")


def read_png_dimensions(path: Path) -> Tuple[int, int]:
    with path.open("rb") as f:
        sig = f.read(8)
        if sig != b"\x89PNG\r\n\x1a\n":
            raise ValueError(f"Not a PNG: {path}")
        chunk_len = struct.unpack(">I", f.read(4))[0]
        chunk_type = f.read(4)
        if chunk_type != b"IHDR" or chunk_len < 8:
            raise ValueError(f"Invalid PNG header: {path}")
        width = struct.unpack(">I", f.read(4))[0]
        height = struct.unpack(">I", f.read(4))[0]
    return width, height


def read_pnm_dimensions(path: Path) -> Tuple[int, int]:
    with path.open("rb") as f:
        magic = f.readline().strip()
        if magic not in (b"P2", b"P5", b"P3", b"P6"):
            raise ValueError(f"Unsupported PNM type in {path}: {magic!r}")

        tokens = []
        while len(tokens) < 3:
            line = f.readline()
            if not line:
                break
            line = line.split(b"#", 1)[0].strip()
            if not line:
                continue
            tokens.extend(line.split())

        if len(tokens) < 3:
            raise ValueError(f"Could not parse PNM header: {path}")

        width = int(tokens[0])
        height = int(tokens[1])
    return width, height


def parse_pose(text: Optional[str]) -> Pose2D:
    if not text:
        return Pose2D(0.0, 0.0, 0.0, 0.0)
    vals = [float(x) for x in text.split()]
    if len(vals) < 2:
        return Pose2D(0.0, 0.0, 0.0, 0.0)
    x = vals[0]
    y = vals[1]
    z = vals[2] if len(vals) >= 3 else 0.0
    yaw = vals[5] if len(vals) >= 6 else 0.0
    return Pose2D(x, y, z, yaw)


def compose_pose(base: Pose2D, child: Pose2D) -> Pose2D:
    c = math.cos(base.yaw)
    s = math.sin(base.yaw)
    x = base.x + (c * child.x - s * child.y)
    y = base.y + (s * child.x + c * child.y)
    z = base.z + child.z
    yaw = base.yaw + child.yaw
    return Pose2D(x, y, z, yaw)


def world_to_pixel(x: float, y: float, origin_x: float, origin_y: float, res: float, height: int) -> Tuple[float, float]:
    px = (x - origin_x) / res
    py_from_bottom = (y - origin_y) / res
    py = (height - 1) - py_from_bottom
    return px, py


def clamp_int(v: int, lo: int, hi: int) -> int:
    return lo if v < lo else hi if v > hi else v


def point_in_poly(x: float, y: float, poly: List[Tuple[float, float]]) -> bool:
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        intersects = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


def mark_rotated_box(
    img: bytearray,
    width: int,
    height: int,
    center: Pose2D,
    size_x: float,
    size_y: float,
    origin_x: float,
    origin_y: float,
    res: float,
    padding: float,
    draw_value: int,
):
    hx = 0.5 * size_x + padding
    hy = 0.5 * size_y + padding
    c = math.cos(center.yaw)
    s = math.sin(center.yaw)

    corners_world = []
    for lx, ly in [(-hx, -hy), (-hx, hy), (hx, hy), (hx, -hy)]:
        wx = center.x + (c * lx - s * ly)
        wy = center.y + (s * lx + c * ly)
        corners_world.append((wx, wy))

    corners_px = [world_to_pixel(wx, wy, origin_x, origin_y, res, height) for wx, wy in corners_world]

    min_x = clamp_int(math.floor(min(p[0] for p in corners_px)), 0, width - 1)
    max_x = clamp_int(math.ceil(max(p[0] for p in corners_px)), 0, width - 1)
    min_y = clamp_int(math.floor(min(p[1] for p in corners_px)), 0, height - 1)
    max_y = clamp_int(math.ceil(max(p[1] for p in corners_px)), 0, height - 1)

    if min_x > max_x or min_y > max_y:
        return

    for py in range(min_y, max_y + 1):
        for px in range(min_x, max_x + 1):
            if point_in_poly(px + 0.5, py + 0.5, corners_px):
                idx = py * width + px
                if draw_value < img[idx]:
                    img[idx] = draw_value


def mark_circle(
    img: bytearray,
    width: int,
    height: int,
    center_x: float,
    center_y: float,
    radius: float,
    origin_x: float,
    origin_y: float,
    res: float,
    padding: float,
    draw_value: int,
):
    r = radius + padding
    cx, cy = world_to_pixel(center_x, center_y, origin_x, origin_y, res, height)
    r_px = r / res

    min_x = clamp_int(math.floor(cx - r_px), 0, width - 1)
    max_x = clamp_int(math.ceil(cx + r_px), 0, width - 1)
    min_y = clamp_int(math.floor(cy - r_px), 0, height - 1)
    max_y = clamp_int(math.ceil(cy + r_px), 0, height - 1)

    r2 = r_px * r_px
    for py in range(min_y, max_y + 1):
        dy = (py + 0.5) - cy
        for px in range(min_x, max_x + 1):
            dx = (px + 0.5) - cx
            if dx * dx + dy * dy <= r2:
                idx = py * width + px
                if draw_value < img[idx]:
                    img[idx] = draw_value


def iter_collisions(root: ET.Element) -> Iterable[Tuple[str, ET.Element, Pose2D]]:
    """Yield (link_name, collision_element, collision_world_pose)."""
    for model in root.findall(".//model"):
        model_pose = parse_pose((model.find("pose").text if model.find("pose") is not None else None))
        for link in model.findall("link"):
            link_name = link.attrib.get("name", "link")
            link_pose = parse_pose((link.find("pose").text if link.find("pose") is not None else None))
            link_world = compose_pose(model_pose, link_pose)
            for collision in link.findall("collision"):
                coll_pose = parse_pose((collision.find("pose").text if collision.find("pose") is not None else None))
                coll_world = compose_pose(link_world, coll_pose)
                yield link_name, collision, coll_world


def extract_box_size(collision: ET.Element) -> Optional[Tuple[float, float, float]]:
    size_node = collision.find("./geometry/box/size")
    if size_node is None or not size_node.text:
        return None
    vals = [float(v) for v in size_node.text.split()]
    if len(vals) < 3:
        return None
    return vals[0], vals[1], vals[2]


def extract_cylinder(collision: ET.Element) -> Optional[Tuple[float, float]]:
    r_node = collision.find("./geometry/cylinder/radius")
    l_node = collision.find("./geometry/cylinder/length")
    if r_node is None or l_node is None or not r_node.text or not l_node.text:
        return None
    return float(r_node.text), float(l_node.text)


def extract_sphere(collision: ET.Element) -> Optional[float]:
    r_node = collision.find("./geometry/sphere/radius")
    if r_node is None or not r_node.text:
        return None
    return float(r_node.text)


def write_pgm(path: Path, width: int, height: int, data: bytearray):
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    with path.open("wb") as f:
        f.write(header)
        f.write(data)


def to_rel_or_abs(target: Path, base_dir: Path) -> str:
    try:
        rel = target.relative_to(base_dir)
        return str(rel)
    except ValueError:
        return str(target)


def parse_extra_padding_rules(raw_rules: List[str]) -> List[Tuple[re.Pattern, float, str]]:
    rules: List[Tuple[re.Pattern, float, str]] = []
    for raw in raw_rules:
        if ":" not in raw:
            raise ValueError(f"Invalid --extra-padding-rule '{raw}'. Expected REGEX:METERS")
        regex_text, pad_text = raw.rsplit(":", 1)
        regex_text = regex_text.strip()
        pad_text = pad_text.strip()
        if not regex_text:
            raise ValueError(f"Invalid --extra-padding-rule '{raw}'. Empty regex")
        pad_val = float(pad_text)
        if pad_val < 0.0:
            raise ValueError(f"Invalid --extra-padding-rule '{raw}'. Padding must be >= 0")
        rules.append((re.compile(regex_text), pad_val, regex_text))
    return rules


def parse_soft_padding_rules(raw_rules: List[str]) -> List[Tuple[re.Pattern, float, float, str]]:
    rules: List[Tuple[re.Pattern, float, float, str]] = []
    for raw in raw_rules:
        parts = raw.rsplit(":", 2)
        if len(parts) != 3:
            raise ValueError(
                f"Invalid --soft-padding-rule '{raw}'. Expected REGEX:METERS:OCCUPANCY_1_99"
            )
        regex_text, pad_text, occ_text = parts[0].strip(), parts[1].strip(), parts[2].strip()
        if not regex_text:
            raise ValueError(f"Invalid --soft-padding-rule '{raw}'. Empty regex")
        pad_val = float(pad_text)
        occ_val = float(occ_text)
        if pad_val <= 0.0:
            raise ValueError(f"Invalid --soft-padding-rule '{raw}'. Padding must be > 0")
        if occ_val <= 0.0 or occ_val >= 100.0:
            raise ValueError(f"Invalid --soft-padding-rule '{raw}'. Occupancy must be in (0, 100)")
        rules.append((re.compile(regex_text), pad_val, occ_val, regex_text))
    return rules


def parse_core_occupancy_rules(raw_rules: List[str]) -> List[Tuple[re.Pattern, float, str]]:
    rules: List[Tuple[re.Pattern, float, str]] = []
    for raw in raw_rules:
        parts = raw.rsplit(":", 1)
        if len(parts) != 2:
            raise ValueError(f"Invalid --core-occupancy-rule '{raw}'. Expected REGEX:OCCUPANCY_1_100")
        regex_text, occ_text = parts[0].strip(), parts[1].strip()
        if not regex_text:
            raise ValueError(f"Invalid --core-occupancy-rule '{raw}'. Empty regex")
        occ_val = float(occ_text)
        if occ_val <= 0.0 or occ_val > 100.0:
            raise ValueError(f"Invalid --core-occupancy-rule '{raw}'. Occupancy must be in (0, 100]")
        rules.append((re.compile(regex_text), occ_val, regex_text))
    return rules


def occupancy_to_pixel(occ: float) -> int:
    # For negate=0 masks: 0 -> white (255), 100 -> black (0)
    return max(0, min(255, int(round((100.0 - occ) * 255.0 / 100.0))))


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--sdf", required=True, help="Path to SDF world/model file")
    p.add_argument("--map-yaml", required=True, help="Path to map YAML used by localization")
    p.add_argument("--output-image", required=True, help="Output keepout mask image (.pgm)")
    p.add_argument("--output-yaml", required=True, help="Output keepout mask YAML")
    p.add_argument("--padding", type=float, default=0.02, help="Extra keepout padding in meters")
    p.add_argument(
        "--core-occupancy",
        type=float,
        default=100.0,
        help="Occupancy value for core obstacle region (1-100). Use <100 for non-lethal cost layers.",
    )
    p.add_argument("--z-min", type=float, default=0.05, help="Ignore collisions whose top is below this z (m)")
    p.add_argument(
        "--include-link-regex",
        default=".*",
        help="Only include collisions from link names matching this regex",
    )
    p.add_argument(
        "--exclude-link-regex",
        default="",
        help="Exclude collisions from link names matching this regex",
    )
    p.add_argument(
        "--include-below-z-link-regex",
        default="^slow_zone_.*",
        help="Include matching links even when collision top is below --z-min",
    )
    p.add_argument(
        "--extra-padding-rule",
        action="append",
        default=[],
        help="Additional padding per link name, format REGEX:METERS (repeatable)",
    )
    p.add_argument(
        "--soft-padding-rule",
        action="append",
        default=[],
        help="Non-lethal halo per link: REGEX:METERS:OCCUPANCY_1_99 (repeatable)",
    )
    p.add_argument(
        "--core-occupancy-rule",
        action="append",
        default=[],
        help="Override core occupancy per link: REGEX:OCCUPANCY_1_100 (repeatable)",
    )
    p.add_argument("--mode", default="trinary", choices=["trinary", "scale", "raw"], help="Map mode for output YAML")
    return p


def main() -> int:
    args = build_arg_parser().parse_args()

    sdf_path = Path(args.sdf).resolve()
    map_yaml_path = Path(args.map_yaml).resolve()
    out_img_path = Path(args.output_image).resolve()
    out_yaml_path = Path(args.output_yaml).resolve()

    map_data = parse_yaml_map(map_yaml_path)
    map_image_path = (map_yaml_path.parent / map_data["image"]).resolve()
    width, height = read_image_dimensions(map_image_path)

    origin_x = map_data["origin_x"]
    origin_y = map_data["origin_y"]
    res = map_data["resolution"]

    include_re = re.compile(args.include_link_regex)
    exclude_re = re.compile(args.exclude_link_regex) if args.exclude_link_regex else None
    include_below_z_re = re.compile(args.include_below_z_link_regex) if args.include_below_z_link_regex else None
    extra_padding_rules = parse_extra_padding_rules(args.extra_padding_rule)
    soft_padding_rules = parse_soft_padding_rules(args.soft_padding_rule)
    core_occupancy_rules = parse_core_occupancy_rules(args.core_occupancy_rule)
    if args.core_occupancy <= 0.0 or args.core_occupancy > 100.0:
        raise ValueError("--core-occupancy must be in (0, 100]")

    tree = ET.parse(sdf_path)
    root = tree.getroot()

    img = bytearray([FREE_VAL] * (width * height))

    marked = 0
    skipped_height = 0
    skipped_regex = 0
    override_hits = 0
    soft_rule_hits = 0

    for link_name, collision, coll_pose in iter_collisions(root):
        if not include_re.search(link_name):
            skipped_regex += 1
            continue
        if exclude_re and exclude_re.search(link_name):
            skipped_regex += 1
            continue

        force_include_below_z = bool(include_below_z_re and include_below_z_re.search(link_name))
        extra_padding = 0.0
        for rule_re, rule_pad, _rule_text in extra_padding_rules:
            if rule_re.search(link_name):
                extra_padding = max(extra_padding, rule_pad)
        if extra_padding > 0.0:
            override_hits += 1
        core_padding = args.padding + extra_padding

        soft_padding = 0.0
        soft_occ = 0.0
        for rule_re, rule_pad, rule_occ, _rule_text in soft_padding_rules:
            if rule_re.search(link_name):
                if rule_pad > soft_padding:
                    soft_padding = rule_pad
                    soft_occ = rule_occ
        if soft_padding > 0.0:
            soft_rule_hits += 1

        core_occupancy = args.core_occupancy
        for rule_re, rule_occ, _rule_text in core_occupancy_rules:
            if rule_re.search(link_name):
                core_occupancy = rule_occ
        core_draw_value = occupancy_to_pixel(core_occupancy)

        box = extract_box_size(collision)
        if box is not None:
            sx, sy, sz = box
            top_z = coll_pose.z + 0.5 * sz
            if (not force_include_below_z) and top_z < args.z_min:
                skipped_height += 1
                continue
            if soft_padding > 0.0:
                mark_rotated_box(
                    img, width, height, coll_pose, sx, sy, origin_x, origin_y, res, core_padding + soft_padding, occupancy_to_pixel(soft_occ)
                )
            mark_rotated_box(img, width, height, coll_pose, sx, sy, origin_x, origin_y, res, core_padding, core_draw_value)
            marked += 1
            continue

        cyl = extract_cylinder(collision)
        if cyl is not None:
            r, l = cyl
            top_z = coll_pose.z + 0.5 * l
            if (not force_include_below_z) and top_z < args.z_min:
                skipped_height += 1
                continue
            if soft_padding > 0.0:
                mark_circle(
                    img, width, height, coll_pose.x, coll_pose.y, r, origin_x, origin_y, res, core_padding + soft_padding, occupancy_to_pixel(soft_occ)
                )
            mark_circle(img, width, height, coll_pose.x, coll_pose.y, r, origin_x, origin_y, res, core_padding, core_draw_value)
            marked += 1
            continue

        sph = extract_sphere(collision)
        if sph is not None:
            top_z = coll_pose.z + sph
            if (not force_include_below_z) and top_z < args.z_min:
                skipped_height += 1
                continue
            if soft_padding > 0.0:
                mark_circle(
                    img, width, height, coll_pose.x, coll_pose.y, sph, origin_x, origin_y, res, core_padding + soft_padding, occupancy_to_pixel(soft_occ)
                )
            mark_circle(img, width, height, coll_pose.x, coll_pose.y, sph, origin_x, origin_y, res, core_padding, core_draw_value)
            marked += 1
            continue

    out_img_path.parent.mkdir(parents=True, exist_ok=True)
    out_yaml_path.parent.mkdir(parents=True, exist_ok=True)

    if out_img_path.suffix.lower() != ".pgm":
        raise ValueError("This script currently writes .pgm only. Use --output-image ...pgm")

    write_pgm(out_img_path, width, height, img)

    yaml_image_value = to_rel_or_abs(out_img_path, out_yaml_path.parent)
    yaml_text = (
        f"image: {yaml_image_value}\n"
        f"resolution: {res}\n"
        f"origin: [{origin_x}, {origin_y}, 0.0]\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
        "negate: 0\n"
        f"mode: {args.mode}\n"
    )
    out_yaml_path.write_text(yaml_text, encoding="utf-8")

    print(f"Map image: {map_image_path}")
    print(f"Map size: {width}x{height}, resolution: {res}, origin: ({origin_x}, {origin_y})")
    print(f"Marked collisions: {marked}")
    print(f"Skipped by z-min ({args.z_min} m): {skipped_height}")
    print(f"Skipped by include/exclude regex: {skipped_regex}")
    print(f"Links with extra padding override: {override_hits}")
    print(f"Links with soft padding rule: {soft_rule_hits}")
    print(f"Wrote keepout image: {out_img_path}")
    print(f"Wrote keepout yaml: {out_yaml_path}")

    if marked == 0:
        print("Warning: no collisions were rasterized. Check link regex and SDF geometry types.", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
