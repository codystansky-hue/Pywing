from flask import Flask, request, send_file, jsonify
from flask_cors import CORS
import os
import tempfile
from wing_generator import create_wing
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

if __name__ == '__main__':
    app.run(port=5000, debug=True)
