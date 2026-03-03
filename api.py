from flask import Flask, request, send_file, jsonify
from flask_cors import CORS
import io
import os
import tempfile
from wing_generator import create_wing, create_wing_from_coords, compute_le_te_points, export_wing
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
        num_sections = data.get('num_sections', 10)
        
        # Convert points to tuples
        le_pts = [tuple(p) for p in le_points]
        te_pts = [tuple(p) for p in te_points]
        
        wing = create_wing(AIRFOIL_PATH, le_pts, te_pts, num_sections)
        
        # Export to a temporary file
        temp_dir = tempfile.gettempdir()
        stl_path = os.path.join(temp_dir, "wing.stl")
        cq.exporters.export(wing, stl_path)
        
        return send_file(stl_path, as_attachment=True, download_name="wing.stl")
    except Exception as e:
        return jsonify({"error": str(e)}), 400

@app.route('/generate-wing', methods=['POST'])
def generate_wing_from_coords_api():
    data = request.json
    try:
        airfoil_coords = data.get('airfoil_coords', [])
        span = float(data.get('span', 1500))
        root_chord = float(data.get('root_chord', 250))
        tip_chord = float(data.get('tip_chord', 120))
        sweep_angle = float(data.get('sweep_angle', 0))
        dihedral_angle = float(data.get('dihedral_angle', 0))
        washout = float(data.get('washout', 0))
        num_sections = int(data.get('num_sections', 30))
        fmt = data.get('format', 'step').lower()

        le_points, te_points = compute_le_te_points(
            span, root_chord, tip_chord, sweep_angle, dihedral_angle, washout
        )

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
    app.run(port=5000, debug=True)
