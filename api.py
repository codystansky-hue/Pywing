from flask import Flask, request, send_file, jsonify
from flask_cors import CORS
import io
import os
import tempfile
from wing_generator import create_wing, create_wing_from_coords, create_wing_with_root_tip, compute_le_te_points, export_wing
import cadquery as cq

app = Flask(__name__)
CORS(app)

AIRFOIL_PATH = "clarkk.dat"

@app.route('/generate', methods=['POST'])
def generate_wing_api():
    data = request.json
    try:
        le_points = data.get('le_points', [])
        te_points = data.get('te_points', [])
        num_sections = data.get('num_sections', 20)
        export_format = data.get('format', 'stl').lower()

        le_pts = [tuple(p) for p in le_points]
        te_pts = [tuple(p) for p in te_points]

        wing = create_wing(AIRFOIL_PATH, le_pts, te_pts, num_sections)

        temp_dir = tempfile.gettempdir()

        if export_format == 'step':
            out_path = os.path.join(temp_dir, "wing.step")
            cq.exporters.export(wing, out_path)
            mime = "application/step"
            download_name = "wing.step"
        else:
            out_path = os.path.join(temp_dir, "wing.stl")
            cq.exporters.export(wing, out_path, tolerance=0.1, angularTolerance=0.1)
            mime = "application/sla"
            download_name = "wing.stl"

        return send_file(out_path, as_attachment=True, download_name=download_name, mimetype=mime)
    except Exception as e:
        return jsonify({"error": str(e)}), 400

@app.route('/generate-wing', methods=['POST'])
def generate_wing_from_coords_api():
    data = request.json
    try:
        num_sections = int(data.get('num_sections', 30))
        fmt = data.get('format', 'step').lower()
        twist_deg = data.get('twist_deg')
        te_thickness_mm = data.get('te_thickness_mm')
        tip_rondure = data.get('tip_rondure')

        # Accept le_points/te_points directly (multi-station mode)
        if 'le_points' in data and 'te_points' in data:
            le_points = [tuple(p) for p in data['le_points']]
            te_points = [tuple(p) for p in data['te_points']]
        else:
            # Fall back to planform params for backward compatibility
            span = float(data.get('span', 1500))
            root_chord = float(data.get('root_chord', 250))
            tip_chord = float(data.get('tip_chord', 120))
            sweep_angle = float(data.get('sweep_angle', 0))
            dihedral_angle = float(data.get('dihedral_angle', 0))
            washout = float(data.get('washout', 0))
            le_points, te_points = compute_le_te_points(
                span, root_chord, tip_chord, sweep_angle, dihedral_angle, washout
            )

        # Root/tip airfoil interpolation mode
        if 'root_airfoil_coords' in data and 'tip_airfoil_coords' in data:
            root_coords = data['root_airfoil_coords']
            tip_coords = data['tip_airfoil_coords']
            wing = create_wing_with_root_tip(
                root_coords, tip_coords,
                le_points, te_points,
                num_sections=num_sections,
                twist_deg_list=twist_deg,
                te_thickness_mm=te_thickness_mm,
            )
        else:
            # Single airfoil mode (backward compat)
            airfoil_coords = data.get('airfoil_coords', [])
            wing = create_wing_from_coords(airfoil_coords, le_points, te_points, num_sections)

        wing_bytes = export_wing(wing, fmt)

        filename = f'wing.{fmt}'
        return send_file(
            io.BytesIO(wing_bytes),
            mimetype='application/octet-stream',
            as_attachment=True,
            download_name=filename
        )
    except Exception as e:
        return jsonify({"error": str(e)}), 400


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
