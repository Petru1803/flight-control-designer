"""Headless end-to-end UI test: model (with actuators), SAS (with washout), HQ 1797A, autopilots
(SISO + MIMO + cascade), digital check, cancel, gain schedule, export, project save/load.
Usage: ui_test.py <output folder>"""
import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QT_QPA_FONTDIR", r"C:\Windows\Fonts")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib  # noqa: E402
matplotlib.use("QtAgg")
import numpy as np  # noqa: E402
from PySide6.QtCore import QThreadPool  # noqa: E402
from PySide6.QtWidgets import QApplication, QMessageBox  # noqa: E402

from core import autopilot as ap  # noqa: E402
from core import cancel  # noqa: E402
from core.schedule import FlightPoint  # noqa: E402
from tests import test_data as td  # noqa: E402
from ui import persist  # noqa: E402
from ui import tab_autopilot  # noqa: E402
from ui.main_window import MainWindow  # noqa: E402

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "_out")
os.makedirs(OUT, exist_ok=True)

# never block on message boxes in a headless run
for name in ("warning", "critical", "information"):
    setattr(QMessageBox, name, staticmethod(lambda *a, **k: print("MSGBOX:", a[1:3])))
QMessageBox.question = staticmethod(lambda *a, **k: QMessageBox.Yes)

app = QApplication(sys.argv)
from ui import theme  # noqa: E402
theme.apply(app)
matplotlib.rcParams["font.family"] = "monospace"
matplotlib.rcParams["font.monospace"] = theme.MONO + ["DejaVu Sans Mono"]


def wait_jobs(timeout=900):
    t0 = time.time()
    while QThreadPool.globalInstance().activeThreadCount() > 0 and time.time() - t0 < timeout:
        app.processEvents()
        time.sleep(0.05)
    for _ in range(20):
        app.processEvents()


def shot(w, idx, name, sub=None):
    w.tabs.setCurrentIndex(idx)
    if sub is not None:
        sub()
    app.processEvents()
    w.grab().save(os.path.join(OUT, name))


w = MainWindow()
w.resize(1600, 1000)
w.show()
mt = w.model_tab
td.fill_course_jet(w)
assert mt.analyse(), "analyse failed"
shot(w, 0, "01_model.png")

w.sas_tab.washout.setValue(3.0)
w.sas_tab.auto_both()
print(w.sas_tab.out.toPlainText())
shot(w, 1, "02_sas.png")

w.hq_tab.evaluate()
shot(w, 2, "03_hq.png")
shot(w, 2, "03b_hq_bw.png", lambda: w.hq_tab.plots.setCurrentIndex(1))

apt = w.ap_tab
modes = list(ap.MODES)
for mode in ("theta", "alt_speed", "heading"):
    apt.mode.setCurrentIndex(modes.index(mode))
    apt.auto_tune()
    wait_jobs()
    e = w.project.ap.get(mode)
    assert e and e["ctrls"], f"{mode}: no result"
    print(mode, "methods:", list(e["ctrls"]), "errors:", e["errors"])
    apt.suggest_fs()
    apt.check_digital()
    wait_jobs()
    print(mode, "digital:", {m: (d["stable"], round(d["margins"]["pm"], 1)) for m, d in e["digital"].items()})
shot(w, 3, "04_autopilot_heading.png")
shot(w, 3, "04b_saturation.png", lambda: apt.plots.setCurrentIndex(4))
shot(w, 3, "04c_digital.png", lambda: apt.plots.setCurrentIndex(5))
apt.mode.setCurrentIndex(modes.index("alt_speed"))
shot(w, 3, "04d_alt_speed.png")

# cancel: start a slow tuning and cancel it
apt.mode.setCurrentIndex(modes.index("roll"))
apt.auto_tune()
time.sleep(1.0)
apt.cancel()
wait_jobs()
print("after cancel: busy =", apt._busy, " roll entry:", "roll" in w.project.ap)

# edited parameters path
apt.mode.setCurrentIndex(modes.index("theta"))
params = apt._read_params()
plant = apt._plant(); st = apt.settings(plant)
cancel.reset()
entry = tab_autopilot.tune_job(plant, st, list(params), "theta", params)
assert not entry["errors"], entry["errors"]

# gain schedule (Course Jet Mach sweep, [V alpha q theta])
for M, V, a, b in td.SCHEDULE:
    w.project.schedule_points.append(FlightPoint(M, V, a, b))
w.sched_tab.refresh()
w.sched_tab.conv.setCurrentIndex(w.sched_tab.conv.findData("Vaqt"))
w.sched_tab.ap_mode.setCurrentIndex(0)
w.sched_tab.run()
wait_jobs()
print("schedule:", w.sched_tab.qout.toPlainText().splitlines()[:3])
shot(w, 5, "05_schedule.png")

# aerodynamic coefficients -> matrices -> Mach sweep (separate window, its own project)
wa = MainWindow(); wa.show()
wa.model_tab.open_aero(); dlg = wa.model_tab._aero; app.processEvents()
r = dlg.run(); assert r is not None and r.trim_residual < 1e-8, "trim failed"
dlg.use_in_model(); assert wa.project.long_model is not None and wa.project.lat_model is not None
dlg.machs.setText("0.5 0.6 0.7"); dlg.sweep()
assert len(wa.project.schedule_points) == 3, "Mach sweep points missing"
print("aero: alpha0 %.3f deg, de0 %.3f deg, T0 %.0f N; sweep points %d" % (
    np.degrees(r.alpha0), np.degrees(r.de0), r.T0, len(wa.project.schedule_points)))
dlg.close(); wa.close()

rt = w.robust_tab
w.tabs.setCurrentIndex(4); app.processEvents()
rt.n.setValue(30)
rt.run()
wait_jobs()
assert w.project.robust is not None, "robustness not computed"
print("robust SAS:", [(r[0], round(r[1]), round(r[2])) for r in __import__("core.robust", fromlist=["x"]).summary_rows(
    w.project.robust, w.project)[0]["rows"]], " AP rows:", rt.t_ap.rowCount(), " disk rows:", rt.t_disk.rowCount())
shot(w, 4, "05r_robust.png")

# flight-controller tab: LQG pitch + H-inf roll -> PX4 module; PID both axes -> ArduPilot .param
ft = w.fc_tab
w.tabs.setCurrentIndex(7); app.processEvents()
ft.s_pitch.setCurrentIndex(ft.s_pitch.findData("LQG")); ft.s_roll.setCurrentIndex(ft.s_roll.findData("Hinf"))
ft.run()
wait_jobs()
assert w.project.fc and set(w.project.fc["designs"]) == {"pitch", "roll"}, "flight-controller design failed"
files = ft.export_to(os.path.join(OUT, "fc_px4"))
assert any(f.endswith("fc_controller.c") for f in files), "PX4 module not written"
print("fc PX4:", [r[:5] for r in ft.t_res.rows_text()] if hasattr(ft.t_res, "rows_text") else "", len(files), "files")
shot(w, 7, "08_flight_controller.png")
ft.fw.setCurrentText("ArduPilot")
ft.s_pitch.setCurrentIndex(ft.s_pitch.findData("PID")); ft.s_roll.setCurrentIndex(ft.s_roll.findData("PID"))
ft.run()
wait_jobs()
files = ft.export_to(os.path.join(OUT, "fc_ardu"))
assert any(f.endswith(".param") for f in files), "ArduPilot .param not written"
print("fc ArduPilot:", open(files[0]).read().count("\n"), "lines in", os.path.basename(files[0]))
shot(w, 7, "08b_flight_controller_ardupilot.png")

et = w.export_tab
et.folder.setText(os.path.join(OUT, "export"))
et.c_csv.setChecked(True)
et.do_export()
print(et.log.toPlainText().splitlines()[:3], "...")
assert any(x.endswith(".mat") for x in et.log.toPlainText().split()), "no .mat exported"
shot(w, 6, "06_export.png")

# save / load round trip
proj = os.path.join(OUT, "test.sasproj")
w.path = proj
w.save_project()
k_before = {m: {k: v for k, v in e["ctrls"][m].gains.items()} for m in e["ctrls"]} if (e := w.project.ap["theta"]) else {}
w2 = MainWindow()
w2.resize(1600, 1000)
w2.show()
w2.open_project(proj)
wait_jobs()
e2 = w2.project.ap.get("theta")
assert e2, "theta not restored"
for m, g in k_before.items():
    for k, v in g.items():
        v2 = e2["ctrls"][m].gains[k]
        assert abs(v - v2) <= 1e-6 * max(1, abs(v)), (m, k, v, v2)
print("restored modes:", list(w2.project.ap), " schedule:", w2.project.schedule is not None,
      " hq:", w2.project.hq1797 is not None, " washout:", w2.project.lat_sas.targets.washout_tau)
shot(w2, 3, "07_reloaded.png")
print("UI TEST OK")
