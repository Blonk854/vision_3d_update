#!/usr/bin/env python3
"""Convert a Vision3D binary .tst to a .vis CAD text file. No SVG required.

Reads the MFC CArchive CAD payload that CDocCompose::GenereVisFile
(Vision3D.exe 0x1401a47c0) walks via CDataCao / CTest:

    CTest::Serialize          VitDataCAD 0x18003df50
    CCarte::Serialize         0x180015670   board polygons as int32 µm CPoint
    CMire::Serialize          0x1800241e0   panel/board fiducials
    CSkip_bloc::Serialize     0x1800340b0
    CComposant::Serialize     0x180018890   x,y,w,h,angle doubles + topo/JEDEC/PN
    CDataMatrixInt::Serialize 0x18001f560   1D/2D codes

TST coordinates are already in VIS space (origin panel bottom-right, X left
negative), stored in micrometres. VIS text is millimetres: mm = µm / 1000.

The file begins with a short header and an embedded PNG thumbnail; CAD
objects start after IEND. This parser locates the CAD blob by class names
and by the CRect / CPoint / double layouts above — it does not replay
CArchive class maps.
"""

from __future__ import annotations

import argparse
import struct
import sys
import tempfile
from collections import defaultdict
from pathlib import Path

# Reuse the VIS compare helper from the SVG-based exporter.
try:
    from tst2vis import compare_vis, fmt6
except ImportError:  # script may be copied alone
    def fmt6(value: float) -> str:
        return f"{value:.6f}"

    def compare_vis(generated: Path, reference: Path) -> int:
        print("tst2vis.compare_vis not available; skip compare")
        return 0


def read_cstr(data: bytes, off: int) -> tuple[str, int] | tuple[None, int]:
    if off >= len(data):
        return None, off
    n = data[off]
    if n == 0 or n == 0xFF or n > 96:
        return None, off
    raw = data[off + 1 : off + 1 + n]
    if len(raw) != n or not raw.isascii():
        return None, off
    text = raw.decode("ascii")
    if not text.isprintable():
        return None, off
    return text, off + 1 + n


def um_to_mm(value: float) -> float:
    return value / 1000.0


def skip_png(data: bytes) -> int:
    png = data.find(b"\x89PNG\r\n\x1a\n")
    if png < 0:
        return 0
    iend = data.find(b"IEND", png)
    if iend < 0:
        return png
    return iend + 8


def cad_anchor(data: bytes) -> int:
    marks = [data.find(n) for n in (b"CComposant", b"CMire", b"CSkip_bloc")]
    marks = [m for m in marks if m >= 0]
    return min(marks) if marks else skip_png(data)


def next_class(data: bytes, start: int, names: tuple[bytes, ...]) -> int:
    hits = [data.find(n, start + 1) for n in names]
    hits = [h for h in hits if h > start]
    return min(hits) if hits else min(len(data), start + 80000)


def parse_header(data: bytes, start: int, end: int | None = None) -> tuple[dict[str, str], list[str]]:
    meta: dict[str, str] = {}
    strings: list[str] = []
    off = start
    stop = min(len(data), end if end is not None else start + 200000)
    while off < stop:
        text, nxt = read_cstr(data, off)
        if text is None:
            off += 1
            continue
        if len(text) >= 2:
            strings.append(text)
        off = nxt
    # Packed after the images directory: stem, panel, [revision], [customer], side, author
    cluster: list[str] = []
    for i, s in enumerate(strings):
        if "Images" in s and (s.startswith("C:\\") or s.startswith("D:\\") or s.startswith("\\\\")):
            cluster = strings[i + 1 : i + 8]
            break
    if cluster:
        if cluster and not cluster[0].startswith(("C:\\", "D:\\")):
            meta["panel_name"] = cluster[1] if len(cluster) > 1 else cluster[0]
        for s in cluster:
            su = s.upper()
            if su in {"TOP", "BOTTOM", "BOT"}:
                meta["side_name"] = "BOTTOM" if su.startswith("BOT") else "TOP"
            elif len(s) == 2 and s.isalnum() and s not in {"mm"}:
                meta.setdefault("revision", s)
            elif s.startswith(("C:\\", "D:\\", "\\\\")):
                continue
            elif s[:1].isupper() and "CAMERA" not in su and s not in meta.get("panel_name", ""):
                if su not in {"TOP", "BOTTOM", "BOT"} and len(s) >= 3:
                    if "author" not in meta and s != meta.get("panel_name") and s != cluster[0]:
                        meta["author"] = s
    for s in strings:
        su = s.upper()
        if su in {"TOP", "BOTTOM", "BOT"}:
            meta.setdefault("side_name", "BOTTOM" if su.startswith("BOT") else "TOP")
        elif len(s) == 2 and s.isalnum() and s not in {"mm"}:
            meta.setdefault("revision", s)
        elif s.count(" ") == 1 and s[:1].isupper():
            if "CAMERA" not in su and ":\\" not in s and "/" not in s:
                meta.setdefault("author", s)
    if "panel_name" not in meta:
        for s in strings:
            if s.endswith("_TOP") or s.endswith("_BOTTOM"):
                meta["panel_name"] = s
                break
    if "panel_name" not in meta:
        for s in strings:
            if 6 <= len(s) <= 40 and s.replace("_", "").replace("-", "").isalnum():
                if not s.startswith(("C:\\", "D:\\", "SKIP", "FIDUCIAL")):
                    meta["panel_name"] = s
                    break
    return meta, strings


def parse_panel_rect(data: bytes, start: int, end: int | None = None) -> tuple[int, int, int, int] | None:
    """CRect in µm: left, top, right, bottom. Official: -W, 0, 0, H."""
    stop = min(len(data) - 16, end if end is not None else start + 32768)
    hits = []
    i = start
    while i < stop:
        left, top, right, bottom = struct.unpack_from("<iiii", data, i)
        if right == 0 and top == 0 and -2_000_000 < left <= -10_000 and 10_000 <= bottom <= 2_000_000:
            hits.append((i, left, top, right, bottom))
            i += 16
            continue
        i += 1
    if not hits:
        return None
    return hits[-1][1:]


def axis_aligned_rect(pts: list[tuple[int, int]]) -> tuple[int, int, float, float] | None:
    """Return (w, h, cx, cy) in µm if pts are an axis-aligned rectangle."""
    if len(pts) != 4:
        return None
    xs = sorted({p[0] for p in pts})
    ys = sorted({p[1] for p in pts})
    if len(xs) != 2 or len(ys) != 2:
        return None
    return xs[1] - xs[0], ys[1] - ys[0], (xs[0] + xs[1]) / 2.0, (ys[0] + ys[1]) / 2.0


def classify_board(z: int, pts: list[tuple[int, int]]) -> str:
    """Match GenereVisFile: BOARD / BOARD_ELLIPSE / BOARD_POLYGON."""
    rect = axis_aligned_rect(pts)
    if rect is None or z == 0:
        return "polygon"
    w, h, _cx, _cy = rect
    # Tooling-hole ellipses are stored as a tiny 4-point box (typically 2 mm).
    if w <= 8000 and h <= 8000:
        return "ellipse"
    return "board"


def parse_boards(
    data: bytes, start: int, end: int | None = None
) -> tuple[list[dict], int]:
    boards: list[dict] = []
    first_off = -1
    sig = b"\x05\x00\x00\x00"
    i = data.find(sig, start)
    if i < 0:
        return boards, first_off
    stop = len(data) - 24 if end is None else min(end, len(data) - 24)
    while i < stop:
        tag, z, idx, one, npts = struct.unpack_from("<iiiii", data, i)
        if tag == 5 and abs(z) <= 16 and one == 1 and 3 <= npts <= 256 and 1 <= idx <= 256:
            pts = []
            p = i + 20
            ok = True
            for _ in range(npts):
                if p + 8 > len(data):
                    ok = False
                    break
                x, y = struct.unpack_from("<ii", data, p)
                if abs(x) > 5_000_000 or abs(y) > 5_000_000:
                    ok = False
                    break
                pts.append((x, y))
                p += 8
            if ok and pts:
                if first_off < 0:
                    first_off = i
                rec: dict = {"idx": idx, "pts": pts, "kind": classify_board(z, pts), "z": z}
                rect = axis_aligned_rect(pts)
                if rect:
                    rec["w_um"], rec["h_um"], rec["cx_um"], rec["cy_um"] = rect
                    rec["angle"] = 0.0
                boards.append(rec)
                i = p
                zeros = 0
                while i + 4 <= len(data) and struct.unpack_from("<i", data, i)[0] == 0 and zeros < 8:
                    i += 4
                    zeros += 1
                continue
        nxt = data.find(sig, i + 1)
        if nxt < 0 or nxt >= stop:
            break
        i = nxt
    return boards, first_off


def parse_components(data: bytes) -> dict[int, list[dict]]:
    mark = data.find(b"CComposant")
    start = mark if mark >= 0 else skip_png(data)
    end = len(data)
    comps: dict[int, list[dict]] = defaultdict(list)
    i = start
    while i < end - 80:
        topo, o1 = read_cstr(data, i)
        if not topo:
            i += 1
            continue
        jedec, o2 = read_cstr(data, o1)
        if not jedec:
            i += 1
            continue
        pn, o3 = read_cstr(data, o2)
        if not pn:
            if o2 < len(data) and data[o2] == 0:
                pn, o3 = "*", o2 + 1
            else:
                i += 1
                continue
        if i < 52:
            i += 1
            continue
        x, y, w, h, ang = struct.unpack_from("<5d", data, i - 40)
        if not (abs(x) < 2_000_000 and abs(y) < 2_000_000 and 5 < abs(w) < 80_000 and 5 < abs(h) < 80_000):
            i += 1
            continue
        if abs(ang) > 720:
            i += 1
            continue
        board = struct.unpack_from("<i", data, i - 52)[0]
        if not (1 <= board <= 256):
            i += 1
            continue
        tested, extra, absent = 1, 0, 0
        if o3 + 20 <= len(data):
            flags = struct.unpack_from("<5i", data, o3)
            tested = 1 if flags[0] else 0
            extra = 1 if flags[1] else 0
            absent = 1 if flags[4] else 0
        elif o3 + 12 <= len(data):
            tested, extra, absent = struct.unpack_from("<iii", data, o3)
            tested = 1 if tested else 0
            extra = 1 if extra else 0
            absent = 1 if absent else 0
        if tested and (pn.upper().startswith("DNP") or jedec.upper() == "DNP"):
            absent = 1
        comps[board].append(
            {
                "x_um": x,
                "y_um": y,
                "angle": ang,
                "part": pn,
                "topo": topo,
                "jedec": jedec,
                "tested": tested,
                "absent": absent,
                "extra": extra if extra in (0, 1) else 0,
            }
        )
        i = o3
    return comps


def parse_skips(data: bytes) -> list[dict]:
    start = data.find(b"CSkip_bloc")
    if start < 0:
        return []
    end = next_class(data, start, (b"CComposant", b"CMire"))
    if end <= start:
        end = min(len(data), start + 200000)
    skips = []
    i = start
    while i + 32 <= end:
        x, y = struct.unpack_from("<2d", data, i)
        if -2_000_000 < x < 50_000 and 0 < y < 2_000_000 and abs(x) > 50 and abs(y) > 50:
            board = struct.unpack_from("<i", data, i + 24)[0]
            if 1 <= board <= 256:
                skips.append({"board": board, "x_um": x, "y_um": y})
                i += 16
                continue
        i += 1
    return skips


def parse_fiducials(data: bytes) -> tuple[list[dict], list[dict]]:
    start = data.find(b"CMire")
    if start < 0:
        return [], []
    end = next_class(data, start, (b"CSkip_bloc", b"CComposant"))
    panel: list[dict] = []
    board_fids: list[dict] = []
    i = start
    idx = 0
    while i + 32 <= end:
        x, y = struct.unpack_from("<2d", data, i)
        if -2_000_000 < x < 50_000 and 0 < y < 2_000_000 and abs(x) > 20:
            idx += 1
            board = struct.unpack_from("<i", data, i + 24)[0] if i + 28 <= len(data) else 0
            rec = {"index": idx, "x_um": x, "y_um": y, "board": board}
            if 1 <= board <= 256:
                board_fids.append(rec)
            else:
                panel.append(rec)
            i += 16
            continue
        i += 1
    return panel, board_fids


def parse_id_codes(data: bytes) -> list[dict]:
    codes = []
    start = cad_anchor(data)
    while True:
        hits = [h for h in (data.find(b"DATAMATRIX", start), data.find(b"BARCODE", start)) if h >= 0]
        if not hits:
            break
        hit = min(hits)
        model = None
        model_off = 0
        after_model = hit
        for back in range(1, 48):
            cand, nxt = read_cstr(data, hit - back)
            # Length byte must start the model string; reject coincidental
            # ASCII runs that swallow DATAMATRIX inside a huge payload.
            if cand and hit < nxt <= hit + 16 and (
                "DATAMATRIX" in cand.upper() or "BARCODE" in cand.upper()
            ):
                model, model_off, after_model = cand, hit - back, nxt
                break
        if not model:
            start = hit + 1
            continue
        name = None
        name_off = model_off
        for back in range(2, 64):
            cand, nxt = read_cstr(data, model_off - back)
            if cand and nxt == model_off:
                name, name_off = cand, model_off - back
                break
        if not name:
            start = hit + 1
            continue
        x = y = ang = 0.0
        board = 0
        for pad in range(0, 16):
            off = after_model + pad
            if off + 24 > len(data):
                break
            dx, dy, dang = struct.unpack_from("<3d", data, off)
            if -2_000_000 < dx < 50_000 and 0 < dy < 2_000_000 and abs(dx) > 50:
                x, y, ang = dx, dy, dang if abs(dang) <= 720 else 0.0
                break
        if "PANEL" not in name.upper():
            for back in (8, 12, 16, 20, 24, 28, 32):
                off = name_off - back
                if off < 0:
                    continue
                b = struct.unpack_from("<i", data, off)[0]
                if 1 <= b <= 256:
                    board = b
                    break
        codes.append(
            {
                "ref": name,
                "model": model,
                "x_um": x,
                "y_um": y,
                "angle": ang,
                "board": board,
            }
        )
        start = after_model
    uniq = {}
    for c in codes:
        uniq[c["ref"]] = c
    return list(uniq.values())


def write_vis_from_tst(parsed: dict, meta: dict, out_path: Path) -> None:
    lines: list[str] = []
    lines.append(f"PANEL_NAME {meta.get('panel_name', out_path.stem)}")
    lines.append("CLEAR")
    lines.append("UNIT mm")
    if meta.get("side_name"):
        lines.append(f"SIDE_NAME {meta['side_name']}")
    if meta.get("revision"):
        lines.append(f"REVISION {meta['revision']}")
    if meta.get("author"):
        lines.append(f"AUTHOR {meta['author']}")
    lines.append(f"SIDE_NUMBER {meta.get('side_number', '1')}")
    lines.append(f"CAD_IMP {meta.get('cad_imp', 'XYAPTJEMS')}")
    left, top, right, bottom = parsed["panel"]
    lines.append(
        f"PANEL_DIM {fmt6(um_to_mm(left))} {fmt6(um_to_mm(top))} "
        f"{fmt6(um_to_mm(right))} {fmt6(um_to_mm(bottom))}"
    )
    lines.append(
        f"USED_DIM {fmt6(um_to_mm(left))} {fmt6(um_to_mm(top))} "
        f"{fmt6(um_to_mm(right))} {fmt6(um_to_mm(bottom))}"
    )
    lines.append("G_OFFSET 0. 0.")
    boards = parsed["boards"]
    lines.append(f"NB_BOARD {len(boards)}")
    for rec in boards:
        idx = rec["idx"]
        kind = rec.get("kind", "polygon")
        if kind == "board":
            lines.append(
                f"BOARD {idx} {fmt6(um_to_mm(rec['w_um']))} {fmt6(um_to_mm(rec['h_um']))} "
                f"{fmt6(um_to_mm(rec['cx_um']))} {fmt6(um_to_mm(rec['cy_um']))} "
                f"{fmt6(rec.get('angle', 0.0))}"
            )
        elif kind == "ellipse":
            lines.append(
                f"BOARD_ELLIPSE {idx} {fmt6(um_to_mm(rec['w_um']))} {fmt6(um_to_mm(rec['h_um']))} "
                f"{fmt6(um_to_mm(rec['cx_um']))} {fmt6(um_to_mm(rec['cy_um']))} "
                f"{fmt6(rec.get('angle', 0.0))}"
            )
        else:
            coord = " ".join(f"{fmt6(um_to_mm(x))} {fmt6(um_to_mm(y))}" for x, y in rec["pts"])
            lines.append(f"BOARD_POLYGON {idx} {len(rec['pts'])} {coord}")

    for code in parsed["id_codes"]:
        x, y = um_to_mm(round(code["x_um"])), um_to_mm(round(code["y_um"]))
        a = code["angle"]
        if int(code.get("board") or 0) == 0:
            lines.append(
                f"1D_2D_CODE_PANEL {code['ref']} {fmt6(x)} {fmt6(y)} {fmt6(a)} {code['model']}"
            )
        else:
            lines.append(
                f"1D_2D_CODE {code['ref']} {int(code['board'])} {fmt6(x)} {fmt6(y)} {fmt6(a)} {code['model']}"
            )

    for i, fid in enumerate(parsed["panel_fids"], start=1):
        lines.append(
            f"FM {fid.get('index') or i} {fmt6(um_to_mm(fid['x_um']))} {fmt6(um_to_mm(fid['y_um']))}"
        )
    for i, fid in enumerate(parsed["board_fids"], start=1):
        lines.append(
            f"FMB {fid.get('index') or i} {int(fid['board'])} "
            f"{fmt6(um_to_mm(fid['x_um']))} {fmt6(um_to_mm(fid['y_um']))}"
        )

    for skip in sorted(parsed["skips"], key=lambda s: s["board"]):
        lines.append(
            f"SKIP {int(skip['board'])} 1 {fmt6(um_to_mm(skip['x_um']))} {fmt6(um_to_mm(skip['y_um']))}"
        )

    for board in sorted(parsed["components"]):
        lines.append(f"COMP {board}")
        comps = sorted(parsed["components"][board], key=lambda c: c["topo"])
        for c in comps:
            lines.append(
                f"{fmt6(um_to_mm(c['x_um']))} {fmt6(um_to_mm(c['y_um']))} {fmt6(c['angle'])} "
                f"{c['part']} {c['topo']} {c['jedec']} {c['tested']} {c['absent']} {c['extra']}"
            )

    out_path.write_text("\n".join(lines) + "\n", encoding="ascii", newline="\n")


def parse_tst(path: Path) -> tuple[dict, dict]:
    data = path.read_bytes()
    cad0 = skip_png(data)
    anchor = cad_anchor(data)
    board_from = max(cad0, anchor - 400000)
    boards, board0 = parse_boards(data, board_from, end=anchor)
    if not boards:
        raise SystemExit(f"no board polygons in {path}")
    hdr_end = board0 if board0 > 0 else anchor
    hdr_from = max(cad0, hdr_end - 32768)
    meta, _strings = parse_header(data, hdr_from, hdr_end)
    rect = parse_panel_rect(data, hdr_from, hdr_end)
    if rect is None:
        raise SystemExit(f"no panel CRect in {path}")
    comps = parse_components(data)
    skips = parse_skips(data)
    panel_fids, board_fids = parse_fiducials(data)
    codes = parse_id_codes(data)
    code_xy = {(round(c["x_um"], 1), round(c["y_um"], 1)) for c in codes}
    skips = [s for s in skips if (round(s["x_um"], 1), round(s["y_um"], 1)) not in code_xy]
    uniq: dict[int, dict] = {}
    for s in skips:
        uniq.setdefault(int(s["board"]), s)
    skips = list(uniq.values())
    panel_fids = [
        f for f in panel_fids if (round(f["x_um"], 1), round(f["y_um"], 1)) not in code_xy
    ]
    board_fids = [
        f for f in board_fids if (round(f["x_um"], 1), round(f["y_um"], 1)) not in code_xy
    ]
    parsed = {
        "panel": rect,
        "boards": boards,
        "components": comps,
        "skips": skips,
        "panel_fids": panel_fids,
        "board_fids": board_fids,
        "id_codes": codes,
    }
    return parsed, meta


def infer_side_number(stem: str, meta: dict) -> str:
    if meta.get("side_number"):
        return str(meta["side_number"])
    lower = stem.lower()
    if any(lower.endswith(s) for s in ("_2nd", "_bot", "_bottom", "_2")):
        return "2"
    return "1"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Convert Vision3D .tst → .vis without an SVG overlay")
    ap.add_argument("input", type=Path, help=".tst file")
    ap.add_argument("-o", "--output", type=Path, help="output .vis path")
    ap.add_argument("--panel-name")
    ap.add_argument("--revision")
    ap.add_argument("--author")
    ap.add_argument("--side-name")
    ap.add_argument("--side-number")
    ap.add_argument("--compare", type=Path, help="official .vis to check against")
    args = ap.parse_args(argv)

    src = args.input
    if src.suffix.lower() != ".tst":
        raise SystemExit("input must be a .tst")
    parsed, meta = parse_tst(src)
    meta["side_number"] = infer_side_number(src.stem, meta)
    if args.panel_name:
        meta["panel_name"] = args.panel_name
    if args.revision:
        meta["revision"] = args.revision
    if args.author:
        meta["author"] = args.author
    if args.side_name:
        meta["side_name"] = args.side_name
    if args.side_number:
        meta["side_number"] = args.side_number
    if "panel_name" not in meta:
        meta["panel_name"] = src.stem

    out = args.output
    if out is None:
        src_s = str(src.resolve()).lower()
        if src_s.startswith("c:\\vit") or src_s.startswith("d:\\vit"):
            out = Path(tempfile.gettempdir()) / (src.stem + ".vis")
        else:
            out = src.with_suffix(".vis")
    if out.resolve() == src.resolve():
        out = src.with_name(src.stem + "_export.vis")
    write_vis_from_tst(parsed, meta, out)
    ncomp = sum(len(v) for v in parsed["components"].values())
    print(
        f"wrote {out}  boards={len(parsed['boards'])} comps={ncomp} "
        f"skip={len(parsed['skips'])} fm={len(parsed['panel_fids'])} "
        f"fmb={len(parsed['board_fids'])} id={len(parsed['id_codes'])}"
    )
    if args.compare:
        return 1 if compare_vis(out, args.compare) else 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
