#!/usr/bin/env python3
"""
obj_topdown.py
--------------
Load a 3D OBJ file and render a top-down (bird's-eye) view as a PNG image.

The image size is computed automatically from the model's aspect ratio so the
model fills the canvas with minimal wasted space.  The longest side is capped
at --max-res pixels (default 2048).  You can still override both dimensions
explicitly with --width and --height if you prefer a fixed canvas.

Usage:
    python obj_topdown.py <input.obj> [output.png] [options]

Options:
    --max-res INT    Cap on the longest image side in pixels (default: 2048)
    --width   INT    Override image width  in pixels  (disables auto-sizing)
    --height  INT    Override image height in pixels  (disables auto-sizing)
    --padding FLOAT  Fractional padding around the model (default: 0.05)
    --bg      COLOR  Background color as R,G,B 0-255  (default: 240,240,240)
    --fill    COLOR  Face fill color as R,G,B 0-255   (default: 80,140,200)
    --edge    COLOR  Edge color as R,G,B 0-255        (default: 30,30,30)
    --alpha   INT    Face fill alpha 0-255            (default: 180)
    --no-edges       Disable edge drawing
    --shadows        Draw soft drop-shadows under faces

Dependencies (install with pip):
    numpy
    Pillow

Example:
    python obj_topdown.py scene.obj top_view.png --max-res 4096
    python obj_topdown.py scene.obj top_view.png --width 1920 --height 1080
"""

import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter


# ---------------------------------------------------------------------------
# OBJ loader
# ---------------------------------------------------------------------------

def load_obj(path: str):
    """
    Parse a Wavefront OBJ file.

    Returns
    -------
    vertices : np.ndarray, shape (N, 3)  – X Y Z positions
    faces    : list[list[int]]           – 0-based vertex indices per face
    """
    vertices = []
    faces = []

    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            parts = line.split()
            token = parts[0].lower()

            if token == "v":
                # vertex position  (ignore optional w)
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])

            elif token == "f":
                # face  – each element may be  v, v/vt, v/vt/vn, v//vn
                indices = []
                for p in parts[1:]:
                    idx = int(p.split("/")[0])
                    # OBJ indices are 1-based; negative means relative
                    if idx < 0:
                        idx = len(vertices) + idx
                    else:
                        idx -= 1
                    indices.append(idx)
                # Triangulate simple polygon (fan triangulation)
                for i in range(1, len(indices) - 1):
                    faces.append([indices[0], indices[i], indices[i + 1]])

    if not vertices:
        raise ValueError("No vertices found in OBJ file.")

    return np.array(vertices, dtype=np.float64), faces


# ---------------------------------------------------------------------------
# Rendering helpers
# ---------------------------------------------------------------------------

def parse_color(s: str) -> tuple:
    parts = [int(x) for x in s.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("Color must be R,G,B  e.g.  255,0,0")
    return tuple(parts)


def project_topdown(vertices: np.ndarray):
    """
    Return the X and Z coordinates (top-down projection ignoring Y / height).
    Most OBJ environments use Y-up convention, so XZ is the ground plane.
    We also try XY if the model is flat in Z.
    """
    x = vertices[:, 0]
    y = vertices[:, 1]
    z = vertices[:, 2]

    range_z = z.max() - z.min()
    range_y = y.max() - y.min()

    if range_z < range_y * 0.01:
        # Model is nearly flat in Z – probably Z-up; use XY plane
        return x, y
    else:
        # Y-up convention – use XZ plane (flip Z so the image feels natural)
        return x, -z


def compute_image_size(
    x_range: float,
    z_range: float,
    max_res: int = 2048,
    min_res: int = 64,
) -> tuple[int, int]:
    """
    Derive (width, height) so the image aspect ratio exactly matches the
    model's footprint aspect ratio, with the longer side equal to max_res.

    Parameters
    ----------
    x_range  : world-space width  of the projected bounding box
    z_range  : world-space height of the projected bounding box
    max_res  : pixel budget for the longest side
    min_res  : minimum pixels on the shorter side (guards against degenerate
               models that are almost a line)

    Returns
    -------
    (width, height) in pixels, both values are even numbers for codec compat.
    """
    if x_range <= 0 or z_range <= 0:
        return max_res, max_res

    aspect = x_range / z_range          # > 1 means wider than tall

    if aspect >= 1.0:                   # landscape / square
        w = max_res
        h = max(min_res, int(round(max_res / aspect)))
    else:                               # portrait
        h = max_res
        w = max(min_res, int(round(max_res * aspect)))

    # Make both dimensions even (friendly for video codecs / texture atlases)
    w = w if w % 2 == 0 else w + 1
    h = h if h % 2 == 0 else h + 1

    return w, h


def world_to_pixel(px, pz, x_min, z_min, scale, pad_px, img_height):
    """Map world (px, pz) → pixel (col, row)."""
    col = ((px - x_min) * scale + pad_px).astype(int)
    row = (img_height - ((pz - z_min) * scale + pad_px)).astype(int)
    return col, row


def face_normal_y(v0, v1, v2):
    """Y-component of the face normal (used for back-face culling in top view)."""
    e1 = v1 - v0
    e2 = v2 - v0
    n = np.cross(e1, e2)
    return n[1]  # y-component


# ---------------------------------------------------------------------------
# Main renderer
# ---------------------------------------------------------------------------

def render_topdown(
    obj_path: str,
    out_path: str,
    width: int | None = None,
    height: int | None = None,
    max_res: int = 2048,
    padding: float = 0.05,
    bg_color=(240, 240, 240),
    fill_color=(80, 140, 200),
    edge_color=(30, 30, 30),
    fill_alpha: int = 180,
    draw_edges: bool = True,
    draw_shadows: bool = False,
):
    print(f"[1/4] Loading '{obj_path}' …")
    vertices, faces = load_obj(obj_path)
    print(f"      {len(vertices):,} vertices, {len(faces):,} faces (after triangulation)")

    # --- project to 2-D ---
    print("[2/4] Projecting to top-down view …")
    px, pz = project_topdown(vertices)

    x_min, x_max = px.min(), px.max()
    z_min, z_max = pz.min(), pz.max()
    x_range = x_max - x_min or 1.0
    z_range = z_max - z_min or 1.0

    # --- determine canvas size ---
    if width is None or height is None:
        auto_w, auto_h = compute_image_size(x_range, z_range, max_res=max_res)
        width  = width  or auto_w
        height = height or auto_h
        print(f"      Auto-sized canvas: {width}×{height} px  "
              f"(aspect {x_range:.2f}×{z_range:.2f}, max-res {max_res})")
    else:
        print(f"      Fixed canvas: {width}×{height} px")

    pad_px = int(min(width, height) * padding)
    draw_w = width  - 2 * pad_px
    draw_h = height - 2 * pad_px
    scale  = min(draw_w / x_range, draw_h / z_range)

    # Center the model
    cx = (x_min + x_max) / 2
    cz = (z_min + z_max) / 2
    x_min_adj = cx - draw_w / (2 * scale)
    z_min_adj = cz - draw_h / (2 * scale)

    col_arr, row_arr = world_to_pixel(px, pz, x_min_adj, z_min_adj, scale, pad_px, height)

    # --- build image ---
    print("[3/4] Rasterising …")
    img = Image.new("RGBA", (width, height), (*bg_color, 255))

    # Optional shadow layer
    if draw_shadows:
        shadow_layer = Image.new("RGBA", (width, height), (0, 0, 0, 0))
        shadow_draw  = ImageDraw.Draw(shadow_layer)

    face_layer = Image.new("RGBA", (width, height), (0, 0, 0, 0))
    face_draw  = ImageDraw.Draw(face_layer)

    fill_rgba = (*fill_color, fill_alpha)
    edge_rgba = (*edge_color, 255)
    shadow_offset = max(2, int(scale * 0.02))

    skipped = 0
    for tri in faces:
        try:
            p0 = (col_arr[tri[0]], row_arr[tri[0]])
            p1 = (col_arr[tri[1]], row_arr[tri[1]])
            p2 = (col_arr[tri[2]], row_arr[tri[2]])
        except IndexError:
            skipped += 1
            continue

        poly = [p0, p1, p2]

        if draw_shadows:
            so = shadow_offset
            shadow_poly = [(c + so, r + so) for c, r in poly]
            shadow_draw.polygon(shadow_poly, fill=(0, 0, 0, 60))

        face_draw.polygon(poly, fill=fill_rgba)
        if draw_edges:
            face_draw.line([p0, p1, p2, p0], fill=edge_rgba, width=1)

    if skipped:
        print(f"      Warning: {skipped} faces skipped (index out of range).")

    if draw_shadows:
        blurred_shadow = shadow_layer.filter(ImageFilter.GaussianBlur(radius=shadow_offset * 2))
        img = Image.alpha_composite(img, blurred_shadow)

    img = Image.alpha_composite(img, face_layer)

    # Convert to RGB for PNG output
    final = img.convert("RGB")

    print(f"[4/4] Saving '{out_path}' ({width}×{height} px) …")
    final.save(out_path, "PNG", optimize=True)
    print("      Done ✓")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Render a top-down PNG from a Wavefront OBJ file.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("input",  help="Path to input .obj file")
    parser.add_argument("output", nargs="?", default=None,
                        help="Path to output .png file (default: <input_stem>_topdown.png)")
    parser.add_argument("--max-res",  type=int,   default=2048,
                        help="Longest side in pixels when auto-sizing (default: 2048)")
    parser.add_argument("--width",    type=int,   default=None,
                        help="Override image width  in pixels (disables auto-sizing)")
    parser.add_argument("--height",   type=int,   default=None,
                        help="Override image height in pixels (disables auto-sizing)")
    parser.add_argument("--padding",  type=float, default=0.05,  help="Padding fraction")
    parser.add_argument("--bg",       type=str,   default="240,240,240",
                        help="Background color R,G,B")
    parser.add_argument("--fill",     type=str,   default="80,140,200",
                        help="Face fill color R,G,B")
    parser.add_argument("--edge",     type=str,   default="30,30,30",
                        help="Edge color R,G,B")
    parser.add_argument("--alpha",    type=int,   default=180,
                        help="Face fill alpha (0-255)")
    parser.add_argument("--no-edges", action="store_true",
                        help="Disable edge drawing")
    parser.add_argument("--shadows",  action="store_true",
                        help="Draw soft drop-shadows under faces")

    args = parser.parse_args()

    if not Path(args.input).is_file():
        print(f"Error: file not found – '{args.input}'", file=sys.stderr)
        sys.exit(1)

    out = args.output or (Path(args.input).stem + "_topdown.png")

    render_topdown(
        obj_path=args.input,
        out_path=out,
        width=args.width,
        height=args.height,
        max_res=args.max_res,
        padding=args.padding,
        bg_color=parse_color(args.bg),
        fill_color=parse_color(args.fill),
        edge_color=parse_color(args.edge),
        fill_alpha=args.alpha,
        draw_edges=not args.no_edges,
        draw_shadows=args.shadows,
    )


if __name__ == "__main__":
    main()
