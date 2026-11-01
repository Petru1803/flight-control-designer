"""Designs flight controllers for PX4's SIH test airplane (10041_sihsim_airplane) with the application's
core and exports them for a SITL flight test.

    python tools/px4_sitl/make_sih_test.py <output folder> [pitch structure] [roll structure] [airspeed]

The SIH aircraft is linearised by tests/sih_airplane.py (a Python replica of PX4's simulator)."""
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from core import fccode, fcexport as fx  # noqa: E402
from core.models import LatModel, LongModel  # noqa: E402
from tests import sih_airplane as sih  # noqa: E402

out = sys.argv[1]
s_pitch = sys.argv[2] if len(sys.argv) > 2 else "LQG"
s_roll = sys.argv[3] if len(sys.argv) > 3 else "Hinf"
V = float(sys.argv[4]) if len(sys.argv) > 4 else 12.0
RATE = 250.0
specs = (6.0, 35.0, 0.1)

r = sih.linearise(V)
lm = LongModel(r["A_long"], r["B_long"], "Vaqt", V0=V, input_names=["de", "dT"])
latm = LatModel(r["A_lat"], r["B_lat"], V0=V, theta0=r["alpha0"], input_names=["da", "dr"])
fm = sih.FLAP_MAX
dmax = {"de": fm, "da": fm, "dr": fm, "dT": np.inf}
designs = {}
for axis, st in (("pitch", s_pitch), ("roll", s_roll)):
    D = fx.design(axis, st, lm, latm, dmax, "PX4", None, True, 1 / RATE, specs=specs)
    res = D.result
    print(f"{axis}: {st} feasible={all(res.feasible.values())} GM={res.margins['gm_db']:.1f} "
          f"PM={res.margins['pm']:.1f} DM={res.margins['dm']:.3f} digital stable={D.digital['stable']}")
    designs[axis] = D

theta0 = r["alpha0"]                         # level flight: theta = alpha
trim = {"de": r["pitch_cmd0"] * fm}          # trim command as an equivalent deflection
dcs = [fx.to_discrete(designs["pitch"], theta0, trim), fx.to_discrete(designs["roll"], 0.0)]
meta = {"name": f"PX4 SIH airplane ({V:g} m/s)", "V0": V, "dmax": dmax,
        "structures": {a: fx.STRUCTURES[d.structure] for a, d in designs.items()}}
files = fccode.write_files(os.path.join(out, "fw_ss_control"), fccode.px4_module(dcs, meta))
# airspeed setpoints for TECS around the design point
with open(os.path.join(out, "sih_params.txt"), "w", newline="\n") as f:
    f.write(f"param set FW_AIRSPD_TRIM {V:g}\nparam set FW_AIRSPD_MIN {V - 2:g}\nparam set FW_AIRSPD_MAX {V + 4:g}\n")
print("\n".join(files))
