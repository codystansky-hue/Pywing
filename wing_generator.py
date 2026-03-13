import cadquery as cq
import math
import numpy as np
from scipy.interpolate import CubicSpline


def load_airfoil(filepath):
    """Loads a Selig format .dat file."""
    with open(filepath, 'r') as f:
        lines = f.readlines()

    coords = []
    for line in lines[1:]:
        parts = line.split()
        if len(parts) == 2:
            coords.append([float(parts[0]), float(parts[1])])

    return np.array(coords)


def _make_spline_bc():
    """Zero first derivative at root, not-a-knot at tip."""
    return ((1, 0.0), "not-a-knot")


def _dedup_pts(pts, tol=1e-6):
    """
    Remove consecutive duplicate 3-D points (within tol distance).
    Also removes the last point if it duplicates the first, since
    makeSpline(periodic=True) closes the curve implicitly.
    BSplCLib::Interpolate fails on duplicate parametrization values.
    """
    if not pts:
        return pts
    out = [pts[0]]
    for p in pts[1:]:
        dx = p[0] - out[-1][0]
        dy = p[1] - out[-1][1]
        dz = p[2] - out[-1][2]
        if math.sqrt(dx*dx + dy*dy + dz*dz) > tol:
            out.append(p)
    # Drop last point if it equals the first (periodic spline closes itself)
    if len(out) > 1:
        dx = out[-1][0] - out[0][0]
        dy = out[-1][1] - out[0][1]
        dz = out[-1][2] - out[0][2]
        if math.sqrt(dx*dx + dy*dy + dz*dz) <= tol:
            out = out[:-1]
    return out


def _build_sections(airfoil_coords, le_points, te_points, num_sections):
    """
    Shared loft-section builder.
    airfoil_coords: np.array (N, 2) normalized [x, z] coords.
    le_points / te_points: [(y_span, x, z), ...] multi-station control points.
    """
    le_y = [p[0] for p in le_points]
    le_x = [p[1] for p in le_points]
    le_z = [p[2] for p in le_points]

    te_y = [p[0] for p in te_points]
    te_x = [p[1] for p in te_points]
    te_z = [p[2] for p in te_points]

    bc = _make_spline_bc()
    cs_le_x = CubicSpline(le_y, le_x, bc_type=bc)
    cs_le_z = CubicSpline(le_y, le_z, bc_type=bc)
    cs_te_x = CubicSpline(te_y, te_x, bc_type=bc)
    cs_te_z = CubicSpline(te_y, te_z, bc_type=bc)

    y_vals = np.linspace(min(le_y), max(le_y), num_sections)

    sections = []
    for y in y_vals:
        lx = float(cs_le_x(y))
        lz = float(cs_le_z(y))
        tx = float(cs_te_x(y))

        chord = tx - lx
        if chord < 1e-6:
            chord = 1e-6

        pts = [(lx + ax * chord, float(y), lz + az * chord)
               for ax, az in airfoil_coords]
        pts = _dedup_pts(pts)

        section_pts = [cq.Vector(p) for p in pts]
        wire = cq.Wire.assembleEdges([cq.Edge.makeSpline(section_pts, periodic=True)])
        sections.append(wire)

    return sections


def create_wing(airfoil_path, le_points, te_points, num_sections=20):
    """Creates a wing from a .dat file path by lofting airfoil sections."""
    airfoil_coords = load_airfoil(airfoil_path)

    if len(airfoil_coords) > 100:
        indices = np.linspace(0, len(airfoil_coords) - 1, 100, dtype=int)
        airfoil_coords = airfoil_coords[indices]

    sections = _build_sections(airfoil_coords, le_points, te_points, num_sections)
    wing_half = cq.Solid.makeLoft(sections, ruled=False)
    return wing_half.fuse(wing_half.mirror("XZ"))


def create_wing_from_coords(coords, le_points, te_points, num_sections=20):
    """Same as create_wing but accepts a list of [x, z] coord pairs."""
    airfoil_coords = np.array(coords, dtype=float)

    if len(airfoil_coords) > 100:
        indices = np.linspace(0, len(airfoil_coords) - 1, 100, dtype=int)
        airfoil_coords = airfoil_coords[indices]

    sections = _build_sections(airfoil_coords, le_points, te_points, num_sections)
    wing_half = cq.Solid.makeLoft(sections, ruled=False)
    return wing_half.fuse(wing_half.mirror("XZ"))


def create_wing_with_root_tip(
    root_coords,
    tip_coords,
    le_points,
    te_points,
    num_sections=20,
    twist_deg_list=None,
    te_thickness_mm=None,
):
    """
    Lofts a wing with per-section airfoil interpolation between root and tip shapes.

    root_coords / tip_coords: [[x, z], ...] normalized closed-loop airfoil coords
        generated from CST (TE→LE upper surface, then LE→TE lower surface).
    le_points / te_points: [(y_span, x_le, z_le), ...] multi-station control points.
    twist_deg_list: twist angles (degrees) at each le_point station — positive
        twists the TE upward (washout). Must be the same length as le_points.
    te_thickness_mm: reserved for future use.
    """
    root_arr = np.array(root_coords, dtype=float)
    tip_arr  = np.array(tip_coords,  dtype=float)

    # Resample to the same point count
    N = min(len(root_arr), len(tip_arr))
    if len(root_arr) != N:
        idx = np.linspace(0, len(root_arr) - 1, N, dtype=int)
        root_arr = root_arr[idx]
    if len(tip_arr) != N:
        idx = np.linspace(0, len(tip_arr) - 1, N, dtype=int)
        tip_arr = tip_arr[idx]

    le_y = [p[0] for p in le_points]
    le_x = [p[1] for p in le_points]
    le_z = [p[2] for p in le_points]

    te_y = [p[0] for p in te_points]
    te_x = [p[1] for p in te_points]
    te_z = [p[2] for p in te_points]

    y_min, y_max = min(le_y), max(le_y)

    bc = _make_spline_bc()
    cs_le_x = CubicSpline(le_y, le_x, bc_type=bc)
    cs_le_z = CubicSpline(le_y, le_z, bc_type=bc)
    cs_te_x = CubicSpline(te_y, te_x, bc_type=bc)
    cs_te_z = CubicSpline(te_y, te_z, bc_type=bc)

    cs_twist = None
    if twist_deg_list is not None and len(twist_deg_list) == len(le_y):
        cs_twist = CubicSpline(le_y, twist_deg_list, bc_type=bc)

    y_vals = np.linspace(y_min, y_max, num_sections)

    sections = []
    for y in y_vals:
        t = float((y - y_min) / (y_max - y_min)) if y_max > y_min else 0.0

        # Interpolate airfoil shape spanwise
        airfoil = (1.0 - t) * root_arr + t * tip_arr

        lx = float(cs_le_x(y))
        lz = float(cs_le_z(y))
        tx = float(cs_te_x(y))

        chord = tx - lx
        if chord < 1e-6:
            chord = 1e-6

        twist_rad = math.radians(float(cs_twist(y))) if cs_twist is not None else 0.0
        cos_t = math.cos(twist_rad)
        sin_t = math.sin(twist_rad)

        pts = []
        for ax, az in airfoil:
            r = ax * chord
            s = az * chord
            if twist_rad != 0.0:
                r, s = r * cos_t - s * sin_t, r * sin_t + s * cos_t
            pts.append((lx + r, float(y), lz + s))
        pts = _dedup_pts(pts)

        section_pts = [cq.Vector(p) for p in pts]
        wire = cq.Wire.assembleEdges([cq.Edge.makeSpline(section_pts, periodic=True)])
        sections.append(wire)

    wing_half = cq.Solid.makeLoft(sections, ruled=False)
    return wing_half.fuse(wing_half.mirror("XZ"))


def compute_le_te_points(span, root_chord, tip_chord, sweep_deg, dihedral_deg, washout_deg):
    """
    Converts planform parameters to LE/TE point tuples.
    Returns (le_points, te_points) as lists of (y, x, z) tuples.
    """
    sweep_rad    = math.radians(sweep_deg)
    dihedral_rad = math.radians(dihedral_deg)
    washout_rad  = math.radians(washout_deg)

    root_le = (0, 0, 0)
    root_te = (0, root_chord, 0)

    tip_le_y = span
    tip_le_x = span * math.tan(sweep_rad)
    tip_le_z = span * math.tan(dihedral_rad)
    tip_te_x = tip_le_x + tip_chord
    tip_te_z = tip_le_z + tip_chord * math.sin(washout_rad)

    tip_le = (tip_le_y, tip_le_x, tip_le_z)
    tip_te = (tip_le_y, tip_te_x, tip_te_z)

    return [root_le, tip_le], [root_te, tip_te]


def export_wing(solid, fmt):
    """Exports a CadQuery solid to bytes. Supports 'step' and 'stl'."""
    import tempfile
    import os

    fmt = fmt.lower()
    if fmt not in ('step', 'stl'):
        raise ValueError(f"Unsupported format: {fmt}")

    with tempfile.NamedTemporaryFile(suffix=f'.{fmt}', delete=False) as tmp:
        tmp_path = tmp.name

    try:
        cq.exporters.export(solid, tmp_path)
        with open(tmp_path, 'rb') as f:
            return f.read()
    finally:
        os.unlink(tmp_path)


if __name__ == "__main__":
    le = [(0, 0, 0), (5, 0.5, 0.5), (10, 2, 0)]
    te = [(0, 1, 0), (5, 1.5, 0.5), (10, 3, 0)]

    result = create_wing("clarkk.dat", le, te, num_sections=20)
    cq.exporters.export(result, "wing.stl", tolerance=0.1, angularTolerance=0.1)
    cq.exporters.export(result, "wing.step")
    print("Wing exported to wing.stl and wing.step")
