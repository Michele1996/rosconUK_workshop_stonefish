#!/usr/bin/env python3
"""Read documented offline force-balance estimate from ROSCon survey scene.

Not a substitute for a 60-120 s unpowered dynamic Stonefish test.
"""
from pathlib import Path
import json
root=Path(__file__).resolve().parents[1]
d=json.loads((root/'data/photogrammetry/featured_scene_manifest.json').read_text())
b=d['buoyancy_check']
print(f"Water density: {d['ocean_water_density_kg_m3']} kg/m^3")
print(f"Original buoyant displacement: {b['internal_buoyant_volume_m3']:.6f} m^3")
print(f"Estimated HeavyFit shell mass: {b['heavyfit_shell_mass_est_kg']:.3f} kg")
print(f"Estimated upward-minus-downward balance: {b['net_buoyancy_equivalent_kg']:+.4f} kg-equivalent")
print('NOTE: Shell-mass assumption only. Test drift in Stonefish with controller:=false, zero thrusters.')
