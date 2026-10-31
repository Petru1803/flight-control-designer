"""Second MATLAB cross-check: actuator model (lag + Pade delay), pitch bandwidth frequency
response, Tustin discretisation and discrete loop margins with a one-sample delay."""
import os
import sys

import numpy as np
import scipy.io

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tests.smoke import lm, rl_user  # noqa: E402
from core import autopilot as ap, digital, hq1797, lti  # noqa: E402

out = sys.argv[1]
d = {}
act = ap.ActuatorSpec(tau=0.05, delay=0.02)
d["As"], d["Bs"], d["Ksas"] = lm.Ad, lm.Bd, rl_user.K
d["tau"], d["delay"] = act.tau, act.delay
plant_a = ap.build_plant("theta", lm, rl_user, actuator=act)
d["poles_act_py"] = np.sort_complex(np.linalg.eigvals(plant_a.A))
A, B, _ = hq1797.closed_loop_axis("long", lm, rl_user, act)
w = np.array([0.5, 1.0, 3.0, 10.0])
c = np.zeros(A.shape[0]); c[3] = 1
d["w"] = w
d["G_py"] = lti.freqresp(A, B[:, [0]], c, 0.0, w)
# digital: LQI (ideal actuators) with fixed weights, Ts = 0.02, Tustin, one-sample delay
plant = ap.build_plant("theta", lm, rl_user)
ctrl = ap.make_lqi(plant, np.diag([0.01, 0.05, 1.0, 400, 2500]), 120.0)
st = ap.APSettings(ref=5, T=20)
Ts = 0.02
dg = digital.digital_check(plant, ctrl, st, Ts, "bilinear", True)
d["Ts"] = Ts
d["Aol"], d["B"], d["Kext"] = plant.Aol, plant.B, plant.Ksas
d["Ac"], d["Bc"], d["Cc"], d["Dc"] = ctrl.Ac, ctrl.Bc, ctrl.Cc, ctrl.Dc
d["Acd_py"], d["Bcd_py"], d["Ccd_py"], d["Dcd_py"] = dg["Ad"], dg["Bd"], dg["Cd"], dg["Dd"]
d["dig_margins_py"] = np.array([dg["margins"]["gm_db"], dg["margins"]["pm"], dg["margins"]["dm"]])
d["dig_rho_py"] = dg["rho"]
scipy.io.savemat(out, d)
print("written", out)
