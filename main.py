"""SAS & Autopilot Designer - entry point.

A splash screen is shown before the heavy scientific modules are imported, so the
application reacts immediately even on a cold start."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def selftest(path):
    """Check the frozen bundle: slycot/hinfsyn, envelopes, a full theta-hold design."""
    import numpy as np
    from core import autopilot as ap, digital, hq, hq1797, sas
    from core.models import LongModel
    lines = []
    try:
        A = np.array([[-0.0071, 0.0301, 0, -9.78], [-0.133, -0.708, 152.4, -0.786],
                      [0.000294, -0.0313, -1.328, 0], [0, 0, 1, 0]])
        B = np.array([[0, 0.08], [-7.2238, 0], [-3.225, 0], [0, 0]])
        lm = LongModel(A, B, "uwqt", 152.4)
        r = sas.design_long_sas(lm, sas.auto_long_targets(lm, "B", "III"), "B", "III")
        act = ap.ActuatorSpec(0.05, 0.01)
        plant = ap.build_plant("theta", lm, r, actuator=act)
        st = ap.APSettings(ref=5, T=20)
        for m in ap.methods_for("theta"):
            c = ap.tune(m, plant, st)
            res = ap.evaluate(plant, c, st)
            dg = digital.digital_check(plant, c, st, 0.02)
            lines.append(f"{m}: ok y_end={res.y[-1, 0]:.3f} PM={res.margins['pm']:.1f} digital={dg['stable']}")
        bw = hq1797.bandwidth(lm, r, act)
        lines.append(f"bandwidth: {bw['wbw']:.3f} rad/s, tau_p={bw['tau_p']:.3f}")
        lines.append("envelopes: " + ("ok" if hq.envelopes().get("cstar") is not None else "MISSING"))
        lines.append("SELFTEST OK")
    except Exception as e:  # noqa: BLE001
        import traceback
        lines.append("SELFTEST FAILED: " + repr(e) + "\n" + traceback.format_exc())
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines))


def main():
    if len(sys.argv) > 2 and sys.argv[1] == "--selftest":
        selftest(sys.argv[2])
        return
    from PySide6.QtCore import Qt
    from PySide6.QtGui import QColor, QPainter, QPixmap
    from PySide6.QtWidgets import QApplication, QSplashScreen

    from ui import theme

    from PySide6.QtGui import QIcon
    from version import APP_NAME, __version__

    app = QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    app.setApplicationVersion(__version__)
    ico = os.path.join(getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__))), "data", "app.ico")
    if os.path.isfile(ico):
        app.setWindowIcon(QIcon(ico))
    theme.apply(app)
    green = QColor(theme.GREEN)
    pm = QPixmap(560, 250)
    pm.fill(QColor(theme.BG))
    p = QPainter(pm)
    p.setPen(QColor(theme.BORDER))
    p.drawRect(pm.rect().adjusted(4, 4, -5, -5))
    p.setPen(green)
    p.setFont(theme.title_font(18))
    p.drawText(pm.rect().adjusted(26, 46, -24, 0), Qt.AlignLeft | Qt.AlignTop, "SAS & Autopilot Designer")
    p.setFont(theme.mono(10, upper=True))
    p.drawText(pm.rect().adjusted(26, 104, -24, 0), Qt.AlignLeft | Qt.AlignTop,
               "Stability augmentation  /  autopilots\nhandling qualities  /  gain scheduling\n\n"
               f"version {__version__}")
    p.end()
    splash = QSplashScreen(pm)
    splash.show()
    splash.showMessage("  INITIALISING NUMERICAL LIBRARIES ...", Qt.AlignBottom | Qt.AlignLeft, green)
    app.processEvents()

    import matplotlib
    matplotlib.use("QtAgg")
    matplotlib.rcParams["font.family"] = "monospace"
    matplotlib.rcParams["font.monospace"] = theme.MONO + ["DejaVu Sans Mono"]
    from ui.main_window import MainWindow

    w = MainWindow()
    w.show()
    splash.finish(w)
    args = [a for a in sys.argv[1:] if a.lower().endswith((".sasproj", ".json"))]
    if args and os.path.isfile(args[0]):
        w.open_project(args[0])
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
