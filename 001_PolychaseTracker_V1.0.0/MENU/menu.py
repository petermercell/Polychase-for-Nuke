"""
SPDX-License-Identifier: GPL-3.0-or-later
Copyright (C) 2026 Peter Mercell

Developed with assistance from Claude (Anthropic).

polychase_nuke_ndk / plugin / menu.py

Nuke auto-runs this AFTER the GUI is up. Adds the Polychase menu and the
PolychaseTracker node creator, the viewer hotkeys, and a helper the C++ plugin
calls back into via Op::script_command():

  _polychase_ensure_transform_geo(tracker_name) -> str
      Idempotent: auto-inserts a TransformGeo named "<tracker>_xform" between
      the upstream geo and the PolychaseTracker. Returns the TransformGeo name
      on success, "" on failure.

The C++ plugin handles all keyframe writes directly via DD::Image::Knob::
setValueAt — Python is only used for the node-graph mutation step, since node
creation/rewiring from inside an Op callback is safest via Python.

Hotkeys (Viewer):
  shift+P        toggle Pin Edit
  ctrl+Z         Undo Move  (falls back to nuke.undo when not on a tracker)
  ctrl+shift+Z   Redo Move  (falls back to nuke.redo)

The move history is independent of Nuke's undo stack, so routing ctrl+Z to it
lets the two coexist instead of fighting. While a PolychaseTracker is the active
node ctrl+Z means "undo move" — and that history now includes gizmo moves, pin
edits AND wireframe-colour changes, so they all undo/redo through this one stack.
Change the shortcut strings below (e.g. 'ctrl+alt+z') if you'd rather leave
Nuke's ctrl+Z untouched.
"""
import nuke
import re


# Menu wiring -----------------------------------------------------------------
_polychase_menu = nuke.menu('Nuke').addMenu('Polychase')
_polychase_menu.addCommand('PolychaseTracker (NDK)',
                           "nuke.createNode('PolychaseTracker')")


# C++ -> Python callbacks -----------------------------------------------------
def _polychase_ensure_transform_geo(tracker_name):
    """Idempotent: insert a TransformGeo named "<tracker_name>_xform" between the
    upstream geo and a PolychaseTracker named `tracker_name`. Returns the
    TransformGeo node name on success, "" otherwise.

      Before:  upstream_geo --geo--> PolychaseTracker
      After:   upstream_geo --geo--> TransformGeo --geo--> PolychaseTracker

    The TransformGeo defaults to identity, so the mesh doesn't move at insertion;
    the C++ side then writes solved keys to it. If the upstream is already the
    named TransformGeo, we leave it alone and return its name.
    """
    tracker = nuke.toNode(tracker_name)
    if tracker is None:
        return ""

    xform_name = tracker_name + "_xform"

    # PolychaseTracker geo is input 2 (img=0, cam=1, geo=2).
    geo_input = tracker.input(2)
    if geo_input is None:
        return ""

    if geo_input.Class() == "TransformGeo" and geo_input.name() == xform_name:
        return xform_name

    xform = nuke.nodes.TransformGeo(name=xform_name)
    xform.setInput(0, geo_input)
    tracker.setInput(2, xform)

    # Cosmetic placement between the two nodes — never fail the insert over it.
    try:
        ux, uy = geo_input.xpos(), geo_input.ypos()
        tx, ty = tracker.xpos(), tracker.ypos()
        xform.setXYpos(int((ux + tx) * 0.5) + 80, int((uy + ty) * 0.5))
    except Exception:
        pass

    return xform_name


# --- Tracker4 'tracks' parsing -----------------------------------------------
# The Tracker4 'tracks' knob is a Table_Knob, NOT an Array_Knob: it has no
# .animations(), .animation(col) or .getValueAt(frame, col). The ONLY reliable
# Python read path is to serialize it with .toScript() and brace-parse the
# result. This mirrors the proven parser in TrackerReducer_PM.
#
# toScript() layout:
#     { ver ncols ntracks } { coldefs... } { {track}{track}... }
# Inside a track block the 'name' column is a QUOTED string (not braced), so the
# curve sub-blocks land at: 0=enable, 1=track_x, 2=track_y, 3=offset_x,
# 4=offset_y, ... (verified against the live toScript and TrackerReducer_PM).

def _pcn_match_brace(text, open_pos):
    """Index of the '}' matching the '{' at open_pos (brace-depth aware)."""
    depth = 0
    for i in range(open_pos, len(text)):
        c = text[i]
        if c == '{':
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0:
                return i
    return -1


def _pcn_parse_curve(curve_str):
    """`{curve [flags] xN v0 v1 ... [xM ...]}` -> {frame: value}.

    Dense tracker curves list one value per consecutive frame after `xN`; an
    explicit `xK` token re-seats the frame cursor (sparse keys); single
    non-numeric flag tokens (e.g. 'K') are skipped without advancing the frame.
    """
    s = curve_str.strip()
    if s.startswith('{'):
        s = s[1:]
    if s.endswith('}'):
        s = s[:-1]
    toks = s.split()
    if not toks or toks[0] != 'curve':
        return {}
    out = {}
    frame = None
    for t in toks[1:]:
        m = re.match(r'^x(-?\d+(?:\.\d+)?)$', t)
        if m:
            frame = int(round(float(m.group(1))))
            continue
        try:
            v = float(t)
        except ValueError:
            continue   # interpolation / flag token (e.g. 'K')
        if frame is None:
            frame = 1
        out[frame] = v
        frame += 1
    return out


def _pcn_curve_val(curve, frame, default=0.0):
    """Value of a parsed curve at `frame`; a single-key curve is constant."""
    if not curve:
        return default
    if frame in curve:
        return curve[frame]
    if len(curve) == 1:
        return next(iter(curve.values()))
    return default


def _pcn_sub_blocks(block):
    """Brace-delimited sub-blocks inside a track block, in column order."""
    subs = []
    search = 1
    end = len(block) - 1
    while search < end:
        ob = block.find('{', search)
        if ob == -1 or ob >= end:
            break
        cb = _pcn_match_brace(block, ob)
        if cb == -1:
            break
        subs.append(block[ob:cb + 1])
        search = cb + 1
    return subs


def _pcn_parse_tracker_tracks(script):
    """Parse a Tracker4 'tracks' toScript() into a list of per-track dicts
    {enable, x, y, ox, oy}, each a {frame: value} map. Empty list on any
    structural surprise (caller then reports 'no tracks found')."""
    # skip header { ver ncols ntracks }
    h_open = script.find('{')
    if h_open == -1:
        return []
    h_close = _pcn_match_brace(script, h_open)
    if h_close == -1:
        return []

    # Sanity-check the header before trusting the fixed column layout below
    # (0=enable, 1=track_x, 2=track_y, 3=offset_x, 4=offset_y). The header is
    # "{ ver ncols ntracks }". If a future Tracker4 changes the serialization so
    # there are fewer columns than we index, brace-parsing would silently produce
    # plausible-but-wrong curves; warn loudly and bail instead. Best-effort: if
    # the header doesn't parse as three ints we proceed (the layout is unchanged
    # in every Nuke we target), but a definitive ncols < 5 is a hard stop.
    try:
        hdr = script[h_open + 1:h_close].split()
        if len(hdr) >= 2:
            ncols = int(hdr[1])
            if ncols < 5:
                nuke.tprint(
                    "PolychaseTracker: Tracker4 'tracks' has %d columns; the "
                    "parser expects >=5 (enable,x,y,ox,oy). Serialization format "
                    "may have changed -- aborting track export to avoid wrong "
                    "data." % ncols)
                return []
    except (ValueError, IndexError):
        pass   # non-numeric header: assume the long-stable layout and continue

    # skip coldefs block
    c_open = script.find('{', h_close + 1)
    if c_open == -1:
        return []
    c_close = _pcn_match_brace(script, c_open)
    if c_close == -1:
        return []
    # data block holds the per-track sub-blocks
    d_open = script.find('{', c_close + 1)
    if d_open == -1:
        return []
    d_close = _pcn_match_brace(script, d_open)
    if d_close == -1:
        return []

    tracks = []
    search = d_open + 1
    while search < d_close:
        tb_open = script.find('{', search)
        if tb_open == -1 or tb_open >= d_close:
            break
        tb_close = _pcn_match_brace(script, tb_open)
        if tb_close == -1 or tb_close >= d_close:
            break
        block = script[tb_open:tb_close + 1]
        if 'curve' in block:
            subs = _pcn_sub_blocks(block)
            # The track NAME is the first quoted string in the block (the 'name'
            # column, which precedes the brace-delimited curve sub-blocks).
            name = ""
            qa = block.find('"')
            if qa != -1:
                qb = block.find('"', qa + 1)
                if qb != -1:
                    name = block[qa + 1:qb]
            tracks.append({
                'name':   name,
                'enable': _pcn_parse_curve(subs[0]) if len(subs) > 0 else {},
                'x':      _pcn_parse_curve(subs[1]) if len(subs) > 1 else {},
                'y':      _pcn_parse_curve(subs[2]) if len(subs) > 2 else {},
                'ox':     _pcn_parse_curve(subs[3]) if len(subs) > 3 else {},
                'oy':     _pcn_parse_curve(subs[4]) if len(subs) > 4 else {},
            })
        search = tb_close + 1
    return tracks


def _polychase_collect_user_tracks(polychase_name, tracker_name):
    """Read every track from a Tracker4 node and serialize the raw 2D positions
    onto the PolychaseTracker's hidden 'user_tracks_blob' knob.

      Blob format (raw, Nuke y-up pixels): tracks separated by ';', observations
      within a track separated by ' ', each observation 'frame,x,y'. The C++ side
      (uts_parse_blob) anchors each track to the mesh (ray-cast on the Reference
      Frame) and applies the convention flip (y_cv = h - y_nuke); here we only
      export the raw, untransformed 2D positions.

    Returns a short status string (the number of tracks exported).

    READ PATH: Tracker4.tracks is a Table_Knob with NO animation()/getValueAt()
    API (earlier attempts via arraySize()/animations() all failed). The working
    approach — proven in TrackerReducer_PM — is to brace-parse tracks.toScript().
    Column order inside a track block (name is a quoted string, hence skipped):
    0=enable, 1=track_x, 2=track_y, 3=offset_x, 4=offset_y. Effective anchor =
    track + offset (offset usually 0). Y-up px, no flip.
    """
    pc = nuke.toNode(polychase_name)
    tr = nuke.toNode(tracker_name)
    if pc is None:
        return "PolychaseTracker node not found"
    if tr is None:
        return "Tracker node '%s' not found" % tracker_name
    k = tr.knob('tracks')
    if k is None:
        return "'%s' has no 'tracks' knob (not a Tracker?)" % tracker_name

    try:
        script = k.toScript()
    except Exception as exc:
        return "could not serialize tracks on '%s' (%s)" % (tracker_name, exc)

    tracks = _pcn_parse_tracker_tracks(script)
    if not tracks:
        return "no tracks found on '%s'" % tracker_name

    out = []
    for idx, t in enumerate(tracks):
        x, y, en = t['x'], t['y'], t['enable']
        if not x or not y:
            continue

        # Frames present on BOTH the x and y curves (the tracked range).
        frames = sorted(fr for fr in x.keys() if fr in y)
        if not frames:
            continue

        # Skip a wholly-disabled track (constant enable == 0).
        if en and len(en) == 1 and _pcn_curve_val(en, frames[0], 1.0) < 0.5:
            continue

        obs = []
        for fr in frames:
            # Per-frame enable when the enable column is animated.
            if en and len(en) > 1 and _pcn_curve_val(en, fr, 1.0) < 0.5:
                continue
            # Effective anchor = track + offset (offset usually 0). Y-up px,
            # no flip -- the C++ consumer flips.
            px = x[fr] + _pcn_curve_val(t['ox'], fr, 0.0)
            py = y[fr] + _pcn_curve_val(t['oy'], fr, 0.0)
            obs.append("%d,%.4f,%.4f" % (fr, px, py))
        if obs:
            # Prefix each track with its STABLE name "name|obs obs ...". The name
            # is the pin<->track link's durable id (survives reorder/reload). Strip
            # the blob separators ';' '|' (and stray whitespace runs collapse to _)
            # so they can't corrupt the format; fall back to "track{index}".
            nm = (t.get('name') or ("track%d" % (idx + 1)))
            nm = nm.replace('|', '_').replace(';', '_').strip()
            if not nm:
                nm = "track%d" % (idx + 1)
            out.append(nm + "|" + " ".join(obs))

    blob = ";".join(out)
    tb = pc.knob('user_tracks_blob')
    if tb is not None:
        tb.setValue(blob)
    return "exported %d track(s) from %s" % (len(out), tracker_name)


# Viewer hotkeys --------------------------------------------------------------
def _find_polychase():
    """PolychaseTrackers to act on: selected ones, else the one feeding the
    active viewer (walking upstream). Returns a list (possibly empty)."""
    sel = [n for n in nuke.selectedNodes() if n.Class() == 'PolychaseTracker']
    if sel:
        return sel

    v = nuke.activeViewer()
    if v:
        idx = v.activeInput()
        start = v.node().input(0 if idx is None else idx)
        seen, stack = set(), [start]
        while stack:
            n = stack.pop()
            if not n or n in seen:
                continue
            seen.add(n)
            if n.Class() == 'PolychaseTracker':
                return [n]
            for i in range(n.inputs()):
                stack.append(n.input(i))
    return []


def toggle_polychase_pin_edit():
    nodes = _find_polychase()
    if not nodes:
        nuke.tprint('PolychaseTracker: none found (select it or view it)')
        return
    for n in nodes:
        k = n.knob('pin_input_active')
        if k:
            k.setValue(not bool(k.value()))
            nuke.tprint('pin_input_active -> %s on %s' % (bool(k.value()), n.name()))


def polychase_undo():
    nodes = _find_polychase()
    if nodes:
        nodes[0]['move_nav'].setValue(-1)   # plugin acts on -1, then resets to 0
    else:
        nuke.undo()


def polychase_redo():
    nodes = _find_polychase()
    if nodes:
        nodes[0]['move_nav'].setValue(1)
    else:
        nuke.redo()


_viewer = nuke.menu('Viewer')
_viewer.addCommand('PolychaseTracker/Toggle Pin Edit [P]', toggle_polychase_pin_edit, 'shift+p')
_viewer.addCommand('PolychaseTracker/Undo Move', polychase_undo, 'ctrl+z')
_viewer.addCommand('PolychaseTracker/Redo Move', polychase_redo, 'ctrl+shift+z')
