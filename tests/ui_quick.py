"""Fast visual check of the UI theme: model, SAS, HQ and a theta-hold LQI/LQG design.
Usage: ui_quick.py <output folder>"""
import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QT_QPA_FONTDIR", r"C:\Windows\Fonts")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib  # noqa: E402
matplotlib.use("QtAgg")
from PySide6.QtCore import QThreadPool  # noqa: E402
from PySide6.QtWidgets import QApplication, QMessageBox  # noqa: E402

for name in ("warning", "critical", "information"):
    setattr(QMessageBox, name, staticmethod(lambda *a, **k: print("MSGBOX:", a[1:3])))

app = QApplication(sys.argv)
from ui import theme  # noqa: E402
theme.apply(app)
matplotlib.rcParams["font.family"] = "monospace"
matplotlib.rcParams["font.monospace"] = theme.MONO + ["DejaVu Sans Mono"]
from ui.main_window import MainWindow  # noqa: E402
from tests.test_data import fill_course_jet  # noqa: E402

OUT = sys.argv[1]
os.makedirs(OUT, exist_ok=True)


def wait_jobs(timeout=600):
    t0 = time.time()
    while QThreadPool.globalInstance().activeThreadCount() > 0 and time.time() - t0 < timeout:
        app.processEvents(); time.sleep(0.05)
    for _ in range(20):
        app.processEvents()


def shot(w, idx, name, sub=None):
    w.tabs.setCurrentIndex(idx)
    if sub:
        sub()
    app.processEvents()
    w.grab().save(os.path.join(OUT, name))


w = MainWindow()
w.resize(1600, 1000)
w.show()
fill_course_jet(w)
assert w.model_tab.analyse()
shot(w, 0, "q1_model.png")
w.sas_tab.auto_both()
shot(w, 1, "q2_sas.png")
w.hq_tab.evaluate()
shot(w, 2, "q3_hq.png")
apt = w.ap_tab
for m, c in apt.meth.items():
    c.setChecked(m in ("LQI", "LQG"))
apt.auto_tune()
wait_jobs()
shot(w, 3, "q4_ap.png")
shot(w, 3, "q4b_effort.png", lambda: apt.plots.setCurrentIndex(1))
if hasattr(w, "help_dock"):
    w.show_help()
    shot(w, 3, "q5_help.png")
print("QUICK OK")
