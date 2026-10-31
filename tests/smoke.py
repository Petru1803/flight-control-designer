"""Smoke test of the core on the Course Jet (generic, invented data - see test_data.py):
every mode x every method, with and without actuator dynamics, plus HQ 1797A and digital checks.
Usage: smoke.py [modes...]"""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core import autopilot as ap, digital, hq1797, sas, sim  # noqa: E402
from core.models import LatModel, LongModel  # noqa: E402
from core.parsing import parse_matrix  # noqa: E402
from tests import test_data as td  # noqa: E402

consts = {"g": 9.81, "TAS": td.V0, "pi": np.pi}
consts["alpha0"] = float(eval(td.CONSTS.split("=")[1], {"pi": np.pi}))
A = parse_matrix(td.A_LONG, consts)
B = parse_matrix(td.B_LONG, consts)
lm = LongModel(A, B, "uwqt", V0=td.V0)
latm = LatModel(parse_matrix(td.A_LAT), parse_matrix(td.B_LAT), V0=td.V0)

cat, cls = td.CAT, td.CLS
user = sas.LongSASTargets(3.5, 0.7, 0.1, 0.6)
rl_user = sas.design_long_sas(lm, user, cat, cls)
print("long K (user targets):", np.round(rl_user.K[0], 5), " levels", rl_user.levels_ol, "->", rl_user.levels_cl)
tt = sas.auto_lat_targets(latm, cat, cls)
tt.washout_tau = 1.0
rlat = sas.design_lat_sas(latm, tt, cat, cls)
print("lat K:\n", rlat.K, "\nlevels", rlat.levels_ol, "->", rlat.levels_cl, "poles", np.round(rlat.poles, 3))

if __name__ == "__main__":
    act = ap.ActuatorSpec(tau=0.05, delay=0.02)
    print("CAP-zeta", hq1797.cap_zeta(lm, rl_user, cat, cls))
    bw = hq1797.bandwidth(lm, rl_user, act)
    print("bandwidth", {k: bw[k] for k in ("wbw", "wbw_phase", "wbw_gain", "w180", "tau_p", "pio_prone")})
    gb = hq1797.gibson(lm, rl_user)
    print("gibson", {k: gb[k] for k in ("qpk_qss", "db_qss", "ok")})
    rp = hq1797.roll_performance(latm, rlat, act, "II-L", cat)
    print("roll perf", {k: rp[k] for k in ("req_bank", "req_time", "t_reach", "ok")})
    modes = sys.argv[1:] or list(ap.MODES)
    failed = 0
    for mode in modes:
        for actuator in (None, act):
            try:
                plant = ap.build_plant(mode, lm, rl_user, latm, rlat, actuator=actuator)
            except ap.APError as e:
                print(mode, "->", e)
                continue
            info = ap.MODES[mode]
            st = ap.APSettings(ref=info["ref"], T=info["T"], x0_track=info.get("x0", [0.0]))
            lqi = None
            for meth in ap.methods_for(mode):
                t0 = time.time()
                try:
                    c = ap.tune(meth, plant, st, lqi)
                    if meth == "LQI":
                        lqi = c
                    r = ap.evaluate(plant, c, st)
                    ns = sim.run_noise(plant, c, st)
                    sat = sim.run_saturated(plant, c, st)
                    dg = digital.digital_check(plant, c, st, 0.02, "bilinear", True)
                    print(f"{mode:12s} {'act' if actuator else 'ideal':5s} {meth:11s} {time.time()-t0:5.1f}s "
                          f"GM={r.margins['gm_db']:.1f} PM={r.margins['pm']:.1f} DM={r.margins['dm']:.3f} "
                          f"yend={np.round(r.y[-1], 3)} peak={np.round(r.peak_effort, 1)} "
                          f"feas={all(r.feasible.values())} bank={r.peak_bank:.1f} noise={ns['rms_effect']:.4f} "
                          f"sat_end={np.round(sat['y'][-1], 2)} dig_stable={dg['stable']} "
                          f"dig_PM={dg['margins']['pm']:.1f} dev={dg['max_dev_pct']:.1f}%")
                except Exception as e:  # noqa: BLE001
                    import traceback
                    traceback.print_exc()
                    print(mode, meth, "FAILED", e)
                    failed += 1
    print("SMOKE OK" if not failed else f"SMOKE: {failed} FAILED")
