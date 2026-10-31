"""Write Python results + inputs to a .mat file for cross-checking in MATLAB (validate_matlab.m).
Data: the Course Jet (tests/test_data.py). Usage: validate_make.py <out.mat>"""
import os
import sys

import numpy as np
import scipy.io

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tests.smoke import latm, lm, rl_user, rlat, user  # noqa: E402
from core import autopilot as ap, sas  # noqa: E402

out = sys.argv[1]
d = {}
# 1. SAS acker with the user's targets
d["As"], d["Bs"] = lm.Ad, lm.Bd
d["poles_long"] = np.array(sas.second_order_poles(user.wn_sp, user.zeta_sp)
                           + sas.second_order_poles(user.wn_ph, user.zeta_ph))
d["K_long_py"] = rl_user.K[0]
# 2. LQI with fixed weights on the theta plant
plant = ap.build_plant("theta", lm, rl_user, latm, rlat)
st = ap.APSettings(ref=5, T=20)
Q = np.diag([0.01, 0.05, 1.0, 400, 2500]); R = 120.0
c = ap.make_lqi(plant, Q, R)
d["A_sas"], d["b"], d["c"] = plant.A, plant.b, plant.c
d["Q"], d["R"] = Q, R
d["K_lqi_py"] = c.info["K"]
mg = ap.loop_margins(plant, c)
d["lqi_margins_py"] = np.array([mg["gm_db"], mg["pm"], mg["dm"]])
# 3. Kalman (lqe) Qn=0.1 I, Rn=0.01
d["Qn"], d["Rn"] = 0.1, 0.01
cg = ap.make_lqg(plant, c, d["Qn"] * np.eye(4), d["Rn"])
d["L_py"] = cg.info["L"]
mg = ap.loop_margins(plant, cg)
d["lqg_margins_py"] = np.array([mg["gm_db"], mg["pm"], mg["dm"]])
# 4. PIDF (auto-tuned gains) in this app's loop (no prefilter inside the loop)
pp = ap.tune("PIDF", plant, st).params
d["pid"] = np.array([pp["Kp"], pp["Ki"], pp["Kd"], pp["Tf"]], dtype=float)
cp = ap.make_pidf(plant, *d["pid"])
mg = ap.loop_margins(plant, cp)
d["pid_margins_py"] = np.array([mg["gm_db"], mg["pm"], mg["dm"]])
# 5. hinfsyn: same generalized plant, weights of the auto-tuned design
hp = ap.tune("Hinf", plant, st).params
W = [float(np.ravel(hp[k])[0]) for k in ("W1", "W2", "W3", "W4", "nu")]
ch = ap.make_hinf(plant, *W)
d["hinf_w"] = np.array(W)
d["gamma_hinf_py"] = ch.info["gamma"]
# 6. loop shaping on the theta plant, crossover and zero of the auto-tuned design
lp = ap.tune("LoopShaping", plant, st).params
cl = ap.make_loopshaping(plant, lp["wc"], lp["rho"], lp["gamma_factor"])
d["ls_gamma_py"] = cl.info["gamma"]; d["ls_gmin_py"] = cl.info["gamma_min"]
d["ls_kw"] = cl.gains["W1 gain kw"]; d["ls_a"] = cl.gains["W1 zero a"]
# 7. lateral place: compare closed-loop poles
d["Alat"], d["Blat"] = latm.Ad, latm.Bd
d["poles_lat"] = np.array(sas.second_order_poles(2.0, 0.6) + [-4.0, -0.02])
d["K_lat_py"] = sas.place(latm.Ad, latm.Bd, d["poles_lat"])
scipy.io.savemat(out, d)
print("written", out)
