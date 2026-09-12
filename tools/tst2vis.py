#!/usr/bin/env python3
"""Standalone Vision3D CAD Text File (.vis) writer.

Clones File → Export → CAD Text File (*.vis), which is:

    CDocCompose::GenereVisFile     Vision3D.exe  0x1401a47c0
      (menu id 0xB1E2, thunk 0x1401a7aa0)
    CVisReader::SaveFile           AVWizard.dll  0x180022ae0
    CVisReader::Write_BOARD        AVWizard.dll  0x180020eb0

Vision3D does not import SaveFile. Compose writes the same keyword
language itself from the loaded TST (CDataCao / CTest).

This script does **not** parse the binary .tst CArchive. Geometry comes
from the sibling SVG overlay Vision3D writes next to a TST (open the
program in Process mode once if the .svg is missing). Optional .tst
bytes are only scanned for header strings (panel name, revision, author,
ID-code model).

Coordinate convention, matched to an official export of 200950AB_2nd:
  SVG units are micrometres, origin top-left, Y down.
  VIS millimetres, origin at the panel bottom-right:
      X_vis = svg_x/1000 - panel_width_mm
      Y_vis = panel_height_mm - svg_y/1000

Keyword grammar (from GenereVisFile / Write_BOARD format strings):

    PANEL_NAME %s
    CLEAR
    UNIT mm
    SIDE_NAME %s
    CUSTOMER %s
    REVISION %s
    AUTHOR %s
    SIDE_NUMBER %d
    CAD_IMP XYAPTJEMS[F]
    PANEL_DIM %lf %lf %lf %lf
    USED_DIM  %lf %lf %lf %lf
    G_OFFSET 0. 0.
    NB_BOARD %d
    BOARD_POLYGON %d %d {x y}...
    BOARD_ELLIPSE %d %lf %lf %lf %lf %lf
    1D_2D_CODE_PANEL %s %lf %lf %lf %s
    1D_2D_CODE %s %d %lf %lf %lf %s
    FM  %d %lf %lf              panel fiducial
    FMB %d %d %lf %lf           board fiducial (index, board, x, y)
    SKIP %d 1 %lf %lf           board, then literal 1, then x y
    COMP %d
    %lf %lf %lf %s %s %s %s %s %s
        x y angle  PN  topo  JEDEC  tested  absent  feeder/stat
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable

PATH_CMD = re.compile(r"([MLZmlz])|([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)")
ROTATE_RE = re.compile(
    r"rotate\(\s*([+-]?(?:\d+\.?\d*|\.\d+))"
    r"(?:[\s,]+([+-]?(?:\d+\.?\d*|\.\d+))[\s,]+([+-]?(?:\d+\.?\d*|\.\d+)))?\s*\)",
    re.I,
)
PASCAL_ASCII = re.compile(rb"[\x20-\x7e]{2,80}")
HEADER_KEYS = {
    "PANEL_NAME",
    "CLEAR",
    "UNIT",
    "SIDE_NAME",
    "CUSTOMER",
    "REVISION",
    "AUTHOR",
    "SIDE_NUMBER",
    "CAD_IMP",
    "PANEL_DIM",
    "USED_DIM",
    "G_OFFSET",
    "NB_BOARD",
}


def fmt6(value: float) -> str:
    return f"{value:.6f}"


def parse_path_points(d: str) -> list[tuple[float, float]]:
    tokens = PATH_CMD.findall(d or "")
    points: list[tuple[float, float]] = []
    cmd = "M"
    nums: list[float] = []

    def flush() -> None:
        nonlocal nums
        if cmd in "MmLl" and len(nums) >= 2:
            for i in range(0, len(nums) - 1, 2):
                points.append((nums[i], nums[i + 1]))
        nums = []

    for op, num in tokens:
        if op:
            flush()
            cmd = op
            if cmd in "Zz":
                nums = []
        else:
            nums.append(float(num))
    flush()
    return points


def bbox_center(points: Iterable[tuple[float, float]]) -> tuple[float, float]:
    pts = list(points)
    if not pts:
        raise ValueError("no points")
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0


def child_paths(el: ET.Element) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for child in el.iter():
        d = child.attrib.get("d")
        if d:
            pts.extend(parse_path_points(d))
    return pts


def class_tokens(el: ET.Element) -> set[str]:
    return set((el.attrib.get("class") or "").split())


def first_path(el: ET.Element) -> ET.Element | None:
    for child in el:
        tag = child.tag.rsplit("}", 1)[-1]
        if tag == "path":
            return child
    return el.find("./{*}path")


def svg_to_vis(x_um: float, y_um: float, width_um: float, height_um: float) -> tuple[float, float]:
    return x_um / 1000.0 - width_um / 1000.0, height_um / 1000.0 - y_um / 1000.0


def infer_side(stem: str) -> tuple[int, str]:
    lower = stem.lower()
    number = 2 if any(lower.endswith(s) for s in ("_2nd", "_bot", "_bottom", "_2")) else 1
    name = "BOTTOM" if any(lower.endswith(s) for s in ("_bot", "_bottom")) else "TOP"
    return number, name


def scrape_tst_metadata(tst_path: Path, stem: str) -> dict[str, str]:
    """Pull length-prefixed ASCII fields that GenereVisFile also writes into the VIS header."""
    data = tst_path.read_bytes()
    meta: dict[str, str] = {}
    needle = stem.encode("ascii", errors="ignore")
    idx = data.find(needle)
    if idx >= 0:
        window = data[idx : idx + 400]
        strings = [m.group().decode("ascii") for m in PASCAL_ASCII.finditer(window)]
        uniq: list[str] = []
        for s in strings:
            if s not in uniq:
                uniq.append(s)
        if len(uniq) >= 2:
            meta["panel_name"] = uniq[1]
        if len(uniq) > 2 and 1 <= len(uniq[2]) <= 8 and uniq[2] not in {"TOP", "BOTTOM"}:
            meta["revision"] = uniq[2]
        skip_author = ("CAMERA", "UART", "COM PORT", "LIN/", ":\\", "/")
        for s in uniq:
            if s in {"TOP", "BOTTOM", "BOT"}:
                meta["side_name"] = "BOTTOM" if s.startswith("BOT") else "TOP"
            elif (
                " " in s
                and s not in set(uniq[:2])
                and not any(k in s.upper() for k in skip_author)
            ):
                meta["author"] = s
            elif s.upper() not in {"TOP", "BOTTOM", "BOT"} and "customer" in s.lower():
                meta["customer"] = s

    for match in re.finditer(
        rb"[\x01-\x50]([A-Za-z0-9_]{3,40})[\x01-\x50]([A-Za-z0-9_]{3,40})",
        data[-200000:],
    ):
        a, b = match.group(1).decode("ascii"), match.group(2).decode("ascii")
        if "DATAMATRIX" in b.upper() or "BARCODE" in b.upper() or "CODE" in b.upper():
            meta.setdefault(f"id_model:{a}", b)
    return meta


def parse_svg(svg_path: Path) -> dict[str, Any]:
    root = ET.fromstring(svg_path.read_text(encoding="latin-1"))
    panel_el = root.find("./{*}g[@id='panel']")
    if panel_el is None:
        panel_el = root.find("./g[@id='panel']")
    if panel_el is None:
        raise SystemExit(f"no #panel group in {svg_path}")
    panel_pts = child_paths(panel_el)
    xs = [p[0] for p in panel_pts]
    ys = [p[1] for p in panel_pts]
    width_um = max(xs) - min(xs)
    height_um = max(ys) - min(ys)

    boards: dict[int, list[tuple[float, float]]] = {}
    sub = root.find(".//{*}g[@id='sub-panels']")
    if sub is None:
        sub = root.find(".//g[@id='sub-panels']")
    if sub is not None:
        for board in list(sub):
            tokens = class_tokens(board)
            if "sub-panel" not in tokens:
                continue
            path_el = first_path(board)
            if path_el is None:
                continue
            idx = int(board.attrib["index"])
            boards[idx] = parse_path_points(path_el.attrib["d"])

    components: dict[int, list[dict]] = defaultdict(list)
    skips: list[dict] = []
    id_codes: list[dict] = []
    fiducials: list[dict] = []

    for el in root.iter():
        tokens = class_tokens(el)
        if "sub-panel" in tokens or "polarity" in tokens:
            continue
        if "skip" in tokens:
            sp = int(el.attrib.get("sub-panel-index", "0"))
            cx, cy = bbox_center(child_paths(el))
            skips.append({"board": sp, "x_um": cx, "y_um": cy, "ref": el.attrib.get("reference", "X")})
            continue
        if "internal-id-code" in tokens:
            cx, cy = bbox_center(child_paths(el))
            rot = ROTATE_RE.search(el.attrib.get("transform") or "")
            angle = float(rot.group(1)) if rot else 0.0
            id_codes.append(
                {
                    "ref": el.attrib.get("reference", "PANEL_ID"),
                    "x_um": cx,
                    "y_um": cy,
                    "angle": angle,
                    "board": int(el.attrib.get("sub-panel-index", "0")),
                    "model": el.attrib.get("jedec") or el.attrib.get("model") or "",
                }
            )
            continue
        if "panel-fiducial" in tokens or "sub-panel-fiducial" in tokens:
            cx, cy = bbox_center(child_paths(el))
            fiducials.append(
                {
                    "kind": "panel" if "panel-fiducial" in tokens else "board",
                    "index": int(el.attrib.get("index", "0") or 0),
                    "board": int(el.attrib.get("panel-index", el.attrib.get("sub-panel-index", "0"))),
                    "x_um": cx,
                    "y_um": cy,
                    "ref": el.attrib.get("reference", "P"),
                }
            )
            continue
        if "component" not in tokens:
            continue
        rot = ROTATE_RE.search(el.attrib.get("transform") or "")
        if rot and rot.group(2) is not None:
            x_um, y_um = float(rot.group(2)), float(rot.group(3))
            angle = float(rot.group(1))
        else:
            x_um, y_um = bbox_center(child_paths(el))
            angle = float(rot.group(1)) if rot else 0.0
        executable = "executable" in tokens and "not-executable" not in tokens
        absent = "missing" in tokens and "not-missing" not in tokens
        in_stat = "in-stat" in tokens and "not-in-stat" not in tokens
        feeder = el.attrib.get("feeder") or ""
        components[int(el.attrib.get("sub-panel-index", "0"))].append(
            {
                "x_um": x_um,
                "y_um": y_um,
                "angle": angle,
                "part": el.attrib.get("part-number") or "",
                "topo": el.attrib.get("topo") or el.attrib.get("reference") or "",
                "jedec": el.attrib.get("jedec") or "",
                "executable": executable,
                "absent": absent,
                "in_stat": in_stat,
                "feeder": feeder,
            }
        )

    return {
        "width_um": width_um,
        "height_um": height_um,
        "boards": {str(k): v for k, v in boards.items()},
        "components": {str(k): v for k, v in components.items()},
        "skips": skips,
        "id_codes": id_codes,
        "fiducials": fiducials,
    }


def cad_imp_token(parsed: dict, meta: dict) -> str:
    explicit = meta.get("cad_imp")
    if explicit:
        return explicit
    has_feeder = any(
        c.get("feeder")
        for comps in parsed.get("components", {}).values()
        for c in comps
    )
    return "XYAPTJEMSF" if has_feeder else "XYAPTJEMS"


def write_vis(parsed: dict, meta: dict, out_path: Path) -> None:
    """Emit the GenereVisFile / Write_BOARD keyword stream."""
    w, h = float(parsed["width_um"]), float(parsed["height_um"])
    boards_raw = parsed["boards"]
    board_ids = sorted(int(k) for k in boards_raw)
    lines: list[str] = []
    lines.append(f"PANEL_NAME {meta.get('panel_name', out_path.stem)}")
    lines.append("CLEAR")
    lines.append(f"UNIT {meta.get('unit', 'mm')}")
    if meta.get("side_name"):
        lines.append(f"SIDE_NAME {meta['side_name']}")
    if meta.get("customer"):
        lines.append(f"CUSTOMER {meta['customer']}")
    if meta.get("revision"):
        lines.append(f"REVISION {meta['revision']}")
    if meta.get("author"):
        lines.append(f"AUTHOR {meta['author']}")
    lines.append(f"SIDE_NUMBER {meta.get('side_number', '1')}")
    lines.append(f"CAD_IMP {cad_imp_token(parsed, meta)}")
    x1, y1 = svg_to_vis(0.0, h, w, h)
    x2, y2 = svg_to_vis(w, 0.0, w, h)
    blx, bly = min(x1, x2), min(y1, y2)
    trx, try_ = max(x1, x2), max(y1, y2)
    lines.append(f"PANEL_DIM {fmt6(blx)} {fmt6(bly)} {fmt6(trx)} {fmt6(try_)}")
    lines.append(f"USED_DIM {fmt6(blx)} {fmt6(bly)} {fmt6(trx)} {fmt6(try_)}")
    lines.append("G_OFFSET 0. 0.")
    lines.append(f"NB_BOARD {len(board_ids)}")
    for idx in board_ids:
        pts = boards_raw[str(idx)] if str(idx) in boards_raw else boards_raw[idx]
        vis_pts = [svg_to_vis(x, y, w, h) for x, y in pts]
        if len(vis_pts) > 1 and vis_pts[0] == vis_pts[-1]:
            vis_pts = vis_pts[:-1]
        coord = " ".join(f"{fmt6(x)} {fmt6(y)}" for x, y in vis_pts)
        lines.append(f"BOARD_POLYGON {idx} {len(vis_pts)} {coord}")

    for code in parsed.get("id_codes", []):
        x, y = svg_to_vis(code["x_um"], code["y_um"], w, h)
        model = (
            code.get("model")
            or meta.get(f"id_model:{code['ref']}")
            or "5X5_DATAMATRIX"
        )
        if int(code.get("board", 0)) == 0:
            lines.append(
                f"1D_2D_CODE_PANEL {code['ref']} {fmt6(x)} {fmt6(y)} {fmt6(code['angle'])} {model}"
            )
        else:
            lines.append(
                f"1D_2D_CODE {code['ref']} {int(code['board'])} {fmt6(x)} {fmt6(y)} {fmt6(code['angle'])} {model}"
            )

    panel_fids = [f for f in parsed.get("fiducials", []) if f.get("kind") == "panel"]
    panel_fids.sort(key=lambda f: f.get("index") or 0)
    for i, fid in enumerate(panel_fids, start=1):
        x, y = svg_to_vis(fid["x_um"], fid["y_um"], w, h)
        lines.append(f"FM {fid.get('index') or i} {fmt6(x)} {fmt6(y)}")

    board_fids = [f for f in parsed.get("fiducials", []) if f.get("kind") == "board"]
    board_fids.sort(key=lambda f: (int(f.get("board", 0)), f.get("index") or 0))
    for i, fid in enumerate(board_fids, start=1):
        x, y = svg_to_vis(fid["x_um"], fid["y_um"], w, h)
        lines.append(
            f"FMB {fid.get('index') or i} {int(fid.get('board', 0))} {fmt6(x)} {fmt6(y)}"
        )

    # GenereVisFile / Write_BOARD: "SKIP %d 1 %lf %lf" — second field is always 1.
    skips = sorted(parsed.get("skips", []), key=lambda s: (int(s["board"]), s.get("ref", "")))
    for skip in skips:
        x, y = svg_to_vis(skip["x_um"], skip["y_um"], w, h)
        lines.append(f"SKIP {int(skip['board'])} 1 {fmt6(x)} {fmt6(y)}")

    comps_by_board = parsed.get("components", {})
    for board in board_ids:
        lines.append(f"COMP {board}")
        comps = comps_by_board.get(str(board), comps_by_board.get(board, []))
        comps = sorted(comps, key=lambda c: c["topo"])
        for c in comps:
            x, y = svg_to_vis(c["x_um"], c["y_um"], w, h)
            tested = 1 if c.get("executable", True) else 0
            absent = 1 if c.get("absent") else 0
            extra = c.get("feeder") or (1 if c.get("in_stat") else 0)
            lines.append(
                f"{fmt6(x)} {fmt6(y)} {fmt6(c['angle'])} {c['part']} {c['topo']} {c['jedec']} {tested} {absent} {extra}"
            )

    out_path.write_text("\n".join(lines) + "\n", encoding="ascii", newline="\n")


def parse_vis(path: Path) -> dict[str, Any]:
    """Read a VIS keyword file back into the same structure write_vis emits."""
    header: dict[str, str] = {}
    boards: dict[str, list[tuple[float, float]]] = {}
    fms: list[dict] = []
    fmbs: list[dict] = []
    skips: list[dict] = []
    codes: list[dict] = []
    comps: dict[str, list[dict]] = defaultdict(list)
    board: int | None = None
    panel_dim = None
    for raw in path.read_text(encoding="latin-1").splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith("COMP "):
            board = int(line.split()[1])
            continue
        if board is not None and line[0] in "+-0123456789":
            p = line.split()
            comps[str(board)].append(
                {
                    "x": float(p[0]),
                    "y": float(p[1]),
                    "angle": float(p[2]),
                    "part": p[3],
                    "topo": p[4],
                    "jedec": p[5],
                    "tested": p[6] if len(p) > 6 else "1",
                    "absent": p[7] if len(p) > 7 else "0",
                    "extra": p[8] if len(p) > 8 else "0",
                }
            )
            continue
        board = None
        key = line.split()[0]
        if key == "BOARD_POLYGON":
            p = line.split()
            n = int(p[2])
            pts = [(float(p[3 + i * 2]), float(p[4 + i * 2])) for i in range(n)]
            boards[p[1]] = ("polygon", pts)
        elif key == "BOARD":
            p = line.split()
            boards[p[1]] = (
                "board",
                float(p[2]),
                float(p[3]),
                float(p[4]),
                float(p[5]),
                float(p[6]) if len(p) > 6 else 0.0,
            )
        elif key == "BOARD_ELLIPSE":
            p = line.split()
            boards[p[1]] = (
                "ellipse",
                float(p[2]),
                float(p[3]),
                float(p[4]),
                float(p[5]),
                float(p[6]) if len(p) > 6 else 0.0,
            )
        elif key == "FM":
            p = line.split()
            fms.append({"index": int(p[1]), "x": float(p[2]), "y": float(p[3])})
        elif key == "FMB":
            p = line.split()
            fmbs.append(
                {"index": int(p[1]), "board": int(p[2]), "x": float(p[3]), "y": float(p[4])}
            )
        elif key == "SKIP":
            p = line.split()
            skips.append({"board": int(p[1]), "x": float(p[3]), "y": float(p[4])})
        elif key == "1D_2D_CODE_PANEL":
            p = line.split()
            codes.append(
                {
                    "kind": "panel",
                    "ref": p[1],
                    "x": float(p[2]),
                    "y": float(p[3]),
                    "angle": float(p[4]),
                    "model": p[5] if len(p) > 5 else "",
                }
            )
        elif key == "1D_2D_CODE":
            p = line.split()
            codes.append(
                {
                    "kind": "board",
                    "ref": p[1],
                    "board": int(p[2]),
                    "x": float(p[3]),
                    "y": float(p[4]),
                    "angle": float(p[5]),
                    "model": p[6] if len(p) > 6 else "",
                }
            )
        elif key == "PANEL_DIM":
            p = line.split()
            panel_dim = [float(v) for v in p[1:5]]
            header[key] = line
        elif key in HEADER_KEYS:
            header[key] = line
    width_mm = abs(panel_dim[2] - panel_dim[0]) if panel_dim else 0.0
    height_mm = abs(panel_dim[3] - panel_dim[1]) if panel_dim else 0.0
    return {
        "header": header,
        "width_um": width_mm * 1000.0,
        "height_um": height_mm * 1000.0,
        "boards": boards,
        "fms": fms,
        "fmbs": fmbs,
        "skips": skips,
        "codes": codes,
        "comps": dict(comps),
    }


def compare_vis(generated: Path, reference: Path) -> int:
    """Compare CAD objects, ignoring component order within a board."""

    def objects(path: Path) -> dict:
        parsed = parse_vis(path)
        comps = {
            k: sorted(
                (
                    c["topo"],
                    c["part"],
                    c["jedec"],
                    f"{c['x']:.6f}",
                    f"{c['y']:.6f}",
                    f"{c['angle']:.6f}",
                    c["tested"],
                    c["absent"],
                    c["extra"],
                )
                for c in v
            )
            for k, v in parsed["comps"].items()
        }
        boards = {}
        for k, v in parsed["boards"].items():
            if v[0] == "polygon":
                boards[k] = ("polygon", tuple(f"{x:.6f}" for pt in v[1] for x in pt))
            else:
                boards[k] = (v[0], *(f"{x:.6f}" for x in v[1:]))
        fms = [(str(f["index"]), f"{f['x']:.6f}", f"{f['y']:.6f}") for f in parsed["fms"]]
        fmbs = [
            (str(f["index"]), str(f["board"]), f"{f['x']:.6f}", f"{f['y']:.6f}")
            for f in parsed["fmbs"]
        ]
        skips = sorted(
            (str(s["board"]), f"{s['x']:.6f}", f"{s['y']:.6f}") for s in parsed["skips"]
        )
        codes = sorted(
            (
                c["kind"],
                c["ref"],
                str(c.get("board", "")),
                f"{c['x']:.6f}",
                f"{c['y']:.6f}",
                f"{c['angle']:.6f}",
                c.get("model", ""),
            )
            for c in parsed["codes"]
        )
        header = [parsed["header"][k] for k in sorted(parsed["header"]) if k in HEADER_KEYS]
        return {
            "header": header,
            "boards": boards,
            "fms": fms,
            "fmbs": fmbs,
            "skips": skips,
            "codes": codes,
            "comps": comps,
        }

    a, b = objects(generated), objects(reference)
    mismatches = 0

    def check(name: str, left: Any, right: Any) -> None:
        nonlocal mismatches
        if left != right:
            mismatches += 1
            print(f"DIFF {name}: generated {left!r} vs reference {right!r}")

    check("boards", a["boards"], b["boards"])
    check("fiducials", a["fms"], b["fms"])
    check("board-fiducials", a["fmbs"], b["fmbs"])
    check("skips", a["skips"], b["skips"])
    check("id-codes", a["codes"], b["codes"])
    check("comp-keys", sorted(a["comps"]), sorted(b["comps"]))
    for board in sorted(set(a["comps"]) | set(b["comps"])):
        ga = a["comps"].get(board, [])
        gb = b["comps"].get(board, [])
        if ga != gb:
            only_g = [x for x in ga if x not in gb]
            only_r = [x for x in gb if x not in ga]
            print(
                f"DIFF COMP {board}: extra={only_g[:4]} missing={only_r[:4]} "
                f"n_extra={len(only_g)} n_missing={len(only_r)}"
            )
            mismatches += 1
    print(f"compare mismatches: {mismatches}")
    return mismatches


def resolve_inputs(src: Path) -> tuple[Path, Path | None]:
    if src.suffix.lower() == ".svg":
        tst = src.with_suffix(".tst")
        return src, tst if tst.exists() else None
    if src.suffix.lower() == ".tst":
        matches = list(src.parent.glob(src.stem + ".svg")) + list(src.parent.glob(src.stem + ".SVG"))
        if not matches:
            raise SystemExit(
                f"No SVG overlay for {src}. Open the TST in Vision3D Process mode "
                "once so it writes the sibling .svg, then re-run."
            )
        return matches[0], src
    raise SystemExit("input must be a .tst, .svg, or use --from-json / --parse-vis")


def build_meta(args: argparse.Namespace, stem: str, tst_path: Path | None) -> dict[str, str]:
    side_number, side_name = infer_side(stem)
    meta = {
        "panel_name": stem,
        "side_number": str(side_number),
        "side_name": side_name,
    }
    if tst_path:
        meta.update({k: v for k, v in scrape_tst_metadata(tst_path, stem).items() if v})
    for key in ("panel_name", "revision", "author", "side_name", "side_number", "customer"):
        val = getattr(args, key.replace("-", "_"), None)
        if val:
            meta[key] = val
    return meta


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Standalone clone of Vision3D CDocCompose::GenereVisFile (.tst/.svg → .vis)"
    )
    ap.add_argument("input", nargs="?", type=Path, help=".tst (uses sibling .svg) or .svg")
    ap.add_argument("-o", "--output", type=Path, help="output .vis path")
    ap.add_argument("--panel-name")
    ap.add_argument("--revision")
    ap.add_argument("--author")
    ap.add_argument("--customer")
    ap.add_argument("--side-name")
    ap.add_argument("--side-number")
    ap.add_argument("--compare", type=Path, help="official .vis to check against")
    ap.add_argument("--dump-json", type=Path, help="write parsed SVG/overlay as JSON")
    ap.add_argument("--from-json", type=Path, help="build .vis from a JSON dump instead of SVG")
    ap.add_argument("--parse-vis", type=Path, help="parse a .vis and print a short summary")
    args = ap.parse_args(argv)

    if args.parse_vis:
        parsed = parse_vis(args.parse_vis)
        ncomp = sum(len(v) for v in parsed["comps"].values())
        print(
            f"{args.parse_vis}  boards={len(parsed['boards'])} "
            f"comps={ncomp} fm={len(parsed['fms'])} fmb={len(parsed['fmbs'])} "
            f"skip={len(parsed['skips'])} id={len(parsed['codes'])}"
        )
        for k in ("PANEL_NAME", "SIDE_NAME", "REVISION", "AUTHOR", "CAD_IMP", "NB_BOARD"):
            if k in parsed["header"]:
                print(" ", parsed["header"][k])
        return 0

    if args.from_json:
        blob = json.loads(args.from_json.read_text(encoding="utf-8"))
        parsed, meta = blob["overlay"], blob.get("meta", {})
        out = args.output or args.from_json.with_suffix(".vis")
        write_vis(parsed, meta, out)
        print(f"wrote {out} from {args.from_json}")
        if args.compare:
            return 1 if compare_vis(out, args.compare) else 0
        return 0

    if args.input is None:
        ap.error("input .tst/.svg is required unless --from-json or --parse-vis")

    svg_path, tst_path = resolve_inputs(args.input)
    parsed = parse_svg(svg_path)
    stem = (tst_path or svg_path).stem
    meta = build_meta(args, stem, tst_path)

    if args.dump_json:
        args.dump_json.write_text(
            json.dumps({"meta": meta, "overlay": parsed}, indent=2),
            encoding="utf-8",
        )
        print(f"wrote {args.dump_json}")

    out = args.output
    if out is None:
        out = (tst_path or svg_path).with_name((tst_path or svg_path).stem + ".vis")
    write_vis(parsed, meta, out)
    ncomp = sum(len(v) for v in parsed["components"].values())
    print(f"wrote {out}  boards={len(parsed['boards'])} comps={ncomp}")
    if args.compare:
        return 1 if compare_vis(out, args.compare) else 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
