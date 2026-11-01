"""SAS & Autopilot Designer - entry point.

A splash screen is shown before the heavy scientific modules are imported, so the
application reacts immediately even on a cold start."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def selftest(path):
    """Check the frozen bundle: slycot/hinfsyn, envelopes, a full theta-hold design and the
    PX4 / ArduPilot export (Course Jet, generic data)."""
    import numpy as np
    from core import autopilot as ap, digital, fccode, fcexport as fx, hq, hq1797, sas
    from core.models import LatModel, LongModel
    lines = []
    try:
        A = np.array([[-0.007979, -2.353849, 0, -9.81], [-0.000434, -1.052917, 0.992153, 0],
                      [-0.000259, -8.787212, -0.968470, 0], [0, 0, 1, 0]])
        B = np.array([[0, 7.69e-05], [-0.076286, 0], [-13.180818, 0], [0, 0]])
        lm = LongModel(A, B, "Vaqt", 212.655)
        r = sas.design_long_sas(lm, sas.auto_long_targets(lm, "B", "II-L"), "B", "II-L")
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
        Al = np.array([[-0.1335, -0.003607, -0.997124, 0.046131], [-20.614086, -3.580298, 0.924871, 0],
                       [6.336297, -0.135076, -0.362320, 0], [0, 1, -0.003607, 0]])
        Bl = np.array([[0, 0.028607], [25.324067, 3.928504], [0.225291, -3.858101], [0, 0]])
        latm = LatModel(Al, Bl, 212.655)
        dmax = {"de": np.radians(25), "da": np.radians(20), "dr": np.radians(25), "dT": np.inf}
        dp = fx.design("pitch", "LQG", lm, latm, dmax, "PX4", act, Ts=0.01)
        dr = fx.design("roll", "Hinf", lm, latm, dmax, "PX4", act, Ts=0.01)
        files = fccode.px4_module([fx.to_discrete(dp), fx.to_discrete(dr)], {"name": "selftest", "V0": 212.655})
        lines.append(f"PX4 export: pitch LQG digital={dp.digital['stable']}, roll Hinf digital={dr.digital['stable']}, "
                     f"{len(files)} files")
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
