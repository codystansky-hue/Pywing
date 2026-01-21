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

if __name__ == "__main__":
    # Test generation
    le = [(0, 0, 0), (5, 0.5, 0.5), (10, 2, 0)]
    te = [(0, 1, 0), (5, 1.5, 0.5), (10, 3, 0)]
    
    result = create_wing("clarkk.dat", le, te, num_sections=5)
    cq.exporters.export(result, "wing.stl")
    print("Wing exported to wing.stl")
