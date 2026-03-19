# File: debug_model_load.py
import yaml
import torch
import sys
import importlib.util

# --- Configuration ---
# This must be the path to the model's YAML config file.
HYPES_PATH = "ecav/BM2CP/opencood/logs/opv2v_bm2cp_det_2025_07_03_23_44_53/config.yaml"

# This must be the absolute path to the model's Python file you have been editing.
# We confirmed this path in the previous step.
MODEL_FILE_PATH = "/home/atlas/TrafficSimulator_eCloud/ecloudsim/ecav/BM2CP/opencood/models/point_pillar_bm2cp.py"

# --- Main Test Logic ---
print("=" * 80)
print("--- STANDALONE MODEL DIAGNOSTIC SCRIPT ---")
print("=" * 80)

try:
    # 1. Load the model's configuration file
    print(f"\n[1] Loading model configuration from: {HYPES_PATH}")
    with open(HYPES_PATH, 'r') as f:
        hypes = yaml.load(f, Loader=yaml.UnsafeLoader)
    print("    YAML configuration loaded successfully.")

    # 2. Dynamically import the PointPillarBM2CP class from the specified file
    print(f"\n[2] Dynamically importing model class from: {MODEL_FILE_PATH}")
    spec = importlib.util.spec_from_file_location("point_pillar_bm2cp", MODEL_FILE_PATH)
    point_pillar_bm2cp_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(point_pillar_bm2cp_module)
    PointPillarBM2CP = point_pillar_bm2cp_module.PointPillarBM2CP
    print(f"    Model class '{PointPillarBM2CP.__name__}' imported successfully.")

    # 3. Try to create an instance of the model
    print("\n[3] Attempting to create an instance of the model (running its __init__ method)...")
    model_args = hypes['model']['args']
    model = PointPillarBM2CP(model_args)
    print("\n" + "*"*80)
    print("--- ✅✅✅ SUCCESS! ✅✅✅ ---")
    print("--- The model file and its __init__ method are working correctly. ---")
    print("*"*80)


except Exception as e:
    print("\n" + "!"*80)
    print("--- ❌❌❌ FAILURE! ❌❌❌ ---")
    print("--- The script failed. The error is inside your model's __init__ method or a file it imports. ---")
    print("!"*80)
    # Print the full error traceback
    import traceback
    traceback.print_exc()
