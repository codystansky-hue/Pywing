import cadquery as cq
import numpy as np
from scipy.interpolate import CubicSpline

def load_airfoil(filepath):
    """Loads a Selig format .dat file."""
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    # Skip the first line (name)
    coords = []
    for line in lines[1:]:
        parts = line.split()
        if len(parts) == 2:
            coords.append([float(parts[0]), float(parts[1])])
    
    return np.array(coords)

def create_wing(
    airfoil_path,
    le_points, # [(y, x, z), ...] - y is spanwise, x is chordwise offset, z is rondure
    te_points, # [(y, x, z), ...]
    num_sections=20
):
    """
    Creates a wing by lofting airfoil sections.
    The sections are interpolated based on the leading and trailing edge splines.
    """
    airfoil_coords = load_airfoil(airfoil_path)
    
    # Downsample airfoil for performance
    if len(airfoil_coords) > 50:
        indices = np.linspace(0, len(airfoil_coords)-1, 50, dtype=int)
        airfoil_coords = airfoil_coords[indices]

    # Create splines for LE and TE
    # We use 'y' (spanwise) as the parameter
    le_y = [p[0] for p in le_points]
    le_x = [p[1] for p in le_points]
    le_z = [p[2] for p in le_points]
    
    te_y = [p[0] for p in te_points]
    te_x = [p[1] for p in te_points]
    te_z = [p[2] for p in te_points]
    
    cs_le_x = CubicSpline(le_y, le_x)
    cs_le_z = CubicSpline(le_y, le_z)
    
    cs_te_x = CubicSpline(te_y, te_x)
    cs_te_z = CubicSpline(te_y, te_z)
    
    # We'll loft through multiple sections to capture the spline curvature
    y_vals = np.linspace(min(le_y), max(le_y), num_sections)
    
    sections = []
    for y in y_vals:
        # Interpolate LE and TE at this y
        lx = cs_le_x(y)
        lz = cs_le_z(y)
        tx = cs_te_x(y)
        tz = cs_te_z(y)
        
        chord = tx - lx
        if chord < 1e-6:
             chord = 1e-6 # Avoid zero chord
        
        pts = []
        for ax, az in airfoil_coords:
            px = lx + ax * chord
            pz = lz + az * chord
            pts.append((px, y, pz))
            
        section_pts = [cq.Vector(p) for p in pts]
        
        # Ensure the section is closed for a better loft
        # makeSpline with periodic=True is good if points allow.
        # But airfoil points are TE-top-LE-bottom-TE.
        wire = cq.Wire.assembleEdges([cq.Edge.makeSpline(section_pts, periodic=True)])
        sections.append(wire)

    # Loft the sections
    wing = cq.Solid.makeLoft(sections)
    return wing

def create_wing_from_coords(coords, le_points, te_points, num_sections=20):
    """
    Same as create_wing but accepts a list of [x, y] coordinate pairs
    instead of a .dat file path.
    """
    airfoil_coords = np.array(coords)

    # Downsample airfoil for performance
    if len(airfoil_coords) > 50:
        indices = np.linspace(0, len(airfoil_coords) - 1, 50, dtype=int)
        airfoil_coords = airfoil_coords[indices]

    # Create splines for LE and TE
    le_y = [p[0] for p in le_points]
    le_x = [p[1] for p in le_points]
    le_z = [p[2] for p in le_points]

    te_y = [p[0] for p in te_points]
    te_x = [p[1] for p in te_points]
    te_z = [p[2] for p in te_points]

    cs_le_x = CubicSpline(le_y, le_x)
    cs_le_z = CubicSpline(le_y, le_z)

    cs_te_x = CubicSpline(te_y, te_x)
    cs_te_z = CubicSpline(te_y, te_z)

    y_vals = np.linspace(min(le_y), max(le_y), num_sections)

    sections = []
    for y in y_vals:
        lx = cs_le_x(y)
        lz = cs_le_z(y)
        tx = cs_te_x(y)
        tz = cs_te_z(y)

        chord = tx - lx
        if chord < 1e-6:
            chord = 1e-6

        pts = []
        for ax, az in airfoil_coords:
            px = lx + ax * chord
            pz = lz + az * chord
            pts.append((px, y, pz))

        section_pts = [cq.Vector(p) for p in pts]
        wire = cq.Wire.assembleEdges([cq.Edge.makeSpline(section_pts, periodic=True)])
        sections.append(wire)

    wing = cq.Solid.makeLoft(sections)
    return wing


def compute_le_te_points(span, root_chord, tip_chord, sweep_deg, dihedral_deg, washout_deg):
    """
    Converts human planform parameters to LE/TE point tuples pywing expects.
    Returns (le_points, te_points) as lists of (y, x, z) tuples.
    """
    import math
    sweep_rad = math.radians(sweep_deg)
    dihedral_rad = math.radians(dihedral_deg)
    washout_rad = math.radians(washout_deg)

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
    """
    Exports a CadQuery solid to bytes. Supports 'step' and 'stl'.
    """
    import tempfile
    import os

    fmt = fmt.lower()
    if fmt not in ('step', 'stl'):
        raise ValueError(f"Unsupported format: {fmt}")

    suffix = f'.{fmt}'
    with tempfile.NamedTemporaryFile(suffix=suffix, delete=False) as tmp:
        tmp_path = tmp.name

    try:
        cq.exporters.export(solid, tmp_path)
        with open(tmp_path, 'rb') as f:
            return f.read()
    finally:
        os.unlink(tmp_path)


if __name__ == "__main__":
    # Test generation
    le = [(0, 0, 0), (5, 0.5, 0.5), (10, 2, 0)]
    te = [(0, 1, 0), (5, 1.5, 0.5), (10, 3, 0)]

    result = create_wing("clarkk.dat", le, te, num_sections=5)
    cq.exporters.export(result, "wing.stl")
    print("Wing exported to wing.stl")
