"""Records docs/demo.gif: a headless run of the main workflow on the Course Jet (generic aircraft),
one captioned frame per step. Usage: python tools/make_demo_gif.py [width]"""
import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QT_QPA_FONTDIR", r"C:\Windows\Fonts")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import matplotlib  # noqa: E402
matplotlib.use("QtAgg")
from PIL import Image, ImageDraw, ImageFont  # noqa: E402
from PySide6.QtCore import QBuffer, QIODevice, QThreadPool  # noqa: E402
from PySide6.QtWidgets import QApplication, QMessageBox  # noqa: E402

for name in ("warning", "critical", "information"):
    setattr(QMessageBox, name, staticmethod(lambda *a, **k: None))
QMessageBox.question = staticmethod(lambda *a, **k: QMessageBox.Yes)

app = QApplication(sys.argv)
from ui import theme  # noqa: E402
theme.apply(app)
matplotlib.rcParams["font.family"] = "monospace"
matplotlib.rcParams["font.monospace"] = theme.MONO + ["DejaVu Sans Mono"]
from core import autopilot as ap  # noqa: E402
from core.schedule import FlightPoint  # noqa: E402
from tests import test_data as td  # noqa: E402
from ui.main_window import MainWindow  # noqa: E402

WIDTH = int(sys.argv[1]) if len(sys.argv) > 1 else 1100
OUT = os.path.join(ROOT, "docs", "demo.gif")
GREEN, BLACK = (61, 220, 132), (0, 0, 0)
FONT = ImageFont.truetype(r"C:\Windows\Fonts\consolab.ttf", 22)

# Course Jet (generic, invented data), 9000 m, Mach 0.70, [V alpha q theta]
A_LONG = """[-0.007978697209  -2.353849027   0   -9.81;
 -0.0004339922616 -1.052916825   0.9921528023   0;
 -0.000259309346  -8.787211988  -0.9684699713   0;
  0   0   1   0]"""
B_LONG = """[ 0   7.692257657e-05;
 -0.07628563466   0;
 -13.18081798   2.5e-06;
  0   0]"""
A_LAT = """[-0.1334998607  -0.003606810217  -0.9971236631   0.04613069026;
 -20.61408649  -3.580298373   0.924870685   0;
  6.336297077  -0.1350756155  -0.3623204745   0;
  0   1  -0.003606833678   0]"""
B_LAT = """[0   0.028607113;
 25.32406732   3.928504188;
 0.2252905627  -3.858100887;
 0   0]"""

frames = []   # (PIL image, duration ms)


def wait_jobs(timeout=900):
    t0 = time.time()
    while QThreadPool.globalInstance().activeThreadCount() > 0 and time.time() - t0 < timeout:
        app.processEvents()
        time.sleep(0.05)
    for _ in range(20):
        app.processEvents()


def shot(w, tab, caption, ms, sub=None):
    w.tabs.setCurrentIndex(tab)
    if sub:
        sub()
    for _ in range(5):
        app.processEvents()
    buf = QBuffer(); buf.open(QIODevice.WriteOnly)
    w.grab().save(buf, "PNG")
    import io
    im = Image.open(io.BytesIO(bytes(buf.data()))).convert("RGB")
    h = int(im.height * WIDTH / im.width)
    im = im.resize((WIDTH, h), Image.LANCZOS)
    bar = 44
    canvas = Image.new("RGB", (WIDTH, h + bar), BLACK)
    canvas.paste(im, (0, 0))
    d = ImageDraw.Draw(canvas)
    d.line([(0, h), (WIDTH, h)], fill=GREEN, width=2)
    d.text((16, h + 10), caption, font=FONT, fill=GREEN)
    frames.append((canvas, ms))
    print(f"frame {len(frames):2d}: {caption}")


w = MainWindow()
w.resize(1600, 1000)
w.show()
mt = w.model_tab
shot(w, 0, "SAS & Autopilot Designer  -  start from a linear model", 1600)

mt.name.setText("Course Jet")
mt.cls.setCurrentText("II-L"); mt.cat.setCurrentText("B")
mt.V0.setValue(212.655); mt.mach.setValue(0.70); mt.alt.setValue(9000); mt.theta0.setValue(-0.207)
mt.conv.setCurrentIndex(mt.conv.findData("Vaqt"))
mt.Along.setPlainText(A_LONG); mt.Blong.setPlainText(B_LONG)
mt.Alat.setPlainText(A_LAT); mt.Blat.setPlainText(B_LAT)
shot(w, 0, "1  Paste A/B matrices in MATLAB syntax (or build them from aero coefficients)", 2200)
assert mt.analyse()
shot(w, 0, "2  Open-loop modes rated against MIL-F-8785C (dutch roll: Level 2)", 2800)

w.sas_tab.auto_both()
shot(w, 1, "3  SAS: automatic Level 1 targets, pole placement, minimum control effort", 2800)

w.hq_tab.evaluate()
shot(w, 2, "4  Handling qualities: MIL-STD-1797A CAP, bandwidth, Gibson, roll performance", 2600)

apt = w.ap_tab
modes = list(ap.MODES)
apt.mode.setCurrentIndex(modes.index("theta"))
for m, c in apt.meth.items():
    c.setChecked(m in ("PIDF", "LQI", "LQG", "Hinf", "LoopShaping"))
apt.auto_tune()
wait_jobs()
shot(w, 3, "5  Autopilot: PIDF, LQI, LQG, H-inf, loop shaping - auto-tuned and compared", 3200)

# the plots sit below the tables: give them room and scroll the right panel down to them
from PySide6.QtWidgets import QScrollArea  # noqa: E402
apt.plots.setMinimumHeight(720)
sa = apt.plots.parent()
while sa is not None and not isinstance(sa, QScrollArea):
    sa = sa.parent()


def plot_tab(i):
    def f():
        apt.plots.setCurrentIndex(i)
        for _ in range(10):           # let the layout grow before scrolling to the bottom
            app.processEvents()
        if sa is not None:
            sa.verticalScrollBar().setValue(sa.verticalScrollBar().maximum())
    return f


shot(w, 3, "   ... step responses of every design", 2400, plot_tab(0))
shot(w, 3, "   ... control effort", 1800, plot_tab(1))
shot(w, 3, "   ... loop margins (gain, phase, delay)", 1800, plot_tab(2))
shot(w, 3, "   ... saturation, rate limits and anti-windup", 1800, plot_tab(4))
shot(w, 3, "   ... digital implementation (Tustin, one-sample delay)", 1800, plot_tab(5))

rt = w.robust_tab
w.tabs.setCurrentIndex(4); app.processEvents()
rt.n.setValue(60)
rt.run()
wait_jobs()
shot(w, 4, "6  Robustness: Monte Carlo on the aero derivatives + disk margins", 2600)

for M, V, a, b in td.SCHEDULE:
    w.project.schedule_points.append(FlightPoint(M, V, a, b))
w.sched_tab.refresh()
w.sched_tab.conv.setCurrentIndex(w.sched_tab.conv.findData("Vaqt"))
w.sched_tab.run()
wait_jobs()
shot(w, 5, "7  Gain schedule across Mach", 2400)
shot(w, 6, "8  Export: PDF, Excel, and a ready-to-run MATLAB / Simulink model", 2600)

# one shared palette keeps the GIF small; the UI is mostly black / green
pal_src = Image.new("RGB", (frames[0][0].width, frames[0][0].height * 3))
for i, k in enumerate((2, 6, 11)):
    pal_src.paste(frames[min(k, len(frames) - 1)][0], (0, i * frames[0][0].height))
pal = pal_src.quantize(colors=128, method=Image.MEDIANCUT)
q = [f.quantize(palette=pal, dither=Image.NONE) for f, _ in frames]
q[0].save(OUT, save_all=True, append_images=q[1:], duration=[d for _, d in frames], loop=0, optimize=True)
print(f"saved {OUT}  {os.path.getsize(OUT) / 1e6:.2f} MB, {len(frames)} frames, {frames[0][0].size}")
