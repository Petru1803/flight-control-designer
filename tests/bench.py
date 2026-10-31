"""Timing benchmark of the slow syntheses (H-inf, PIDF) and of the Monte Carlo analysis.
Prints the time and a fingerprint of the result so serial / parallel runs can be compared."""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def main():
    from tests.smoke import latm, lm, rl_user
    from core import autopilot as ap, robust, sas
    from ui.project import Project
    act = ap.ActuatorSpec(0.05, 0.02)
    tlat = sas.design_lat_sas(latm, sas.auto_lat_targets(latm, "B", "II-L"), "B", "II-L")
    out = {}
    for mode, meth in (("theta", "Hinf"), ("alt_speed", "Hinf"), ("heading_beta", "Hinf"), ("theta", "PIDF")):
        plant = ap.build_plant(mode, lm, rl_user, latm, tlat, actuator=act)
        st = ap.APSettings(ref=ap.MODES[mode]["ref"], T=ap.MODES[mode]["T"])
        t0 = time.time()
        c = ap.tune(meth, plant, st)
        dt = time.time() - t0
        r = ap.evaluate(plant, c, st)
        out[(mode, meth)] = c
        print(f"{mode:13s} {meth:5s} {dt:6.1f}s  J={c.info.get('cost', np.nan):.5f}  "
              f"PM={r.margins['pm']:.2f} DM={r.margins['dm']:.3f} feas={all(r.feasible.values())}")
    p = Project(); p.cat, p.cls = "B", "II-L"; p.long_model, p.lat_model = lm, latm
    p.long_sas, p.lat_sas, p.actuator = rl_user, tlat, act
    plant = ap.build_plant("theta", lm, rl_user, latm, tlat, actuator=act)
    st = ap.APSettings(ref=5, T=20)
    p.ap["theta"] = {"plant": plant, "settings": st, "ctrls": {"LQI": ap.tune("LQI", plant, st), "Hinf": out[("theta", "Hinf")]},
                     "results": {}}
    t0 = time.time()
    res = robust.run(p, 0.2, 200, 1, True, True, ["theta"])
    print(f"Monte Carlo 200 samples {time.time() - t0:6.1f}s  long stable {100 * res['sas']['long'].stable:.1f}%  "
          f"lat stable {100 * res['sas']['lat'].stable:.1f}%  LQI meets {100 * res['ap'][('theta', 'LQI')].meets:.1f}%")


if __name__ == "__main__":
    main()
