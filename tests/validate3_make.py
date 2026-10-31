"""Builds a project (Course Jet, actuators, theta and psi+beta autopilots, Mach schedule), exports it
with core.matlab_export for validate3_matlab.m, and writes the Python disk margins for validate4_disk.m.
Usage: validate3_make.py <folder>"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tests.smoke import latm, lm  # noqa: E402
from tests import test_data as td  # noqa: E402
from core import autopilot as ap, matlab_export, robust, sas  # noqa: E402
from core.schedule import FlightPoint, run_schedule  # noqa: E402
from ui.project import Project  # noqa: E402

folder = sys.argv[1]
cat, cls = td.CAT, td.CLS
p = Project()
p.name, p.cat, p.cls = "Course Jet test", cat, cls
p.long_model, p.lat_model = lm, latm
p.long_sas = sas.design_long_sas(lm, sas.auto_long_targets(lm, cat, cls, True), cat, cls)
p.lat_sas = sas.design_lat_sas(latm, sas.auto_lat_targets(latm, cat, cls, True), cat, cls)
p.actuator = ap.ActuatorSpec(0.05, 0.02)
cases = (("theta", ("PIDF", "LQI", "LQG", "Hinf")), ("heading_beta", ("LQI",)))
disk = []
for mode, meths in cases:
    plant = ap.build_plant(mode, lm, p.long_sas, latm, p.lat_sas, actuator=p.actuator)
    st = ap.APSettings(ref=ap.MODES[mode]["ref"], T=ap.MODES[mode]["T"])
    ctrls, res = {}, {}
    for m in meths:
        c = ap.tune(m, plant, st)
        ctrls[m], res[m] = c, ap.evaluate(plant, c, st, with_margins=True)
        _, dgm, dpm = robust.disk_margins(plant, c)["multi"]
        disk.append([dgm, dpm])
    p.ap[mode] = {"plant": plant, "settings": st, "ctrls": ctrls, "results": res, "digital": None}
pts = [FlightPoint(M, V, a, b) for M, V, a, b in td.SCHEDULE]
p.schedule = run_schedule(pts, "Vaqt", cat, cls, ["de"], ["da", "dr"])
files = matlab_export.export(p, folder, "cjtest")
K = p.schedule.interp(0.55, "long")
np.savetxt(os.path.join(folder, "sched_055.txt"), K.ravel())
np.savetxt(os.path.join(folder, "disk_py.txt"), np.array(disk))
print("\n".join(files))
