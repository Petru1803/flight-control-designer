"""Dialog: aerodynamic coefficients -> trim -> A, B (longitudinal and lateral), Mach sweep."""
import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QCheckBox, QComboBox, QDialog, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
                               QLabel, QLineEdit, QMessageBox, QPlainTextEdit, QPushButton, QSplitter,
                               QVBoxLayout, QWidget)

from core import aero_lin as al
from core import hq
from core.models import lat_modes, long_modes
from core.parsing import MatrixParseError, format_matrix, parse_matrix
from core.schedule import FlightPoint
from ui.widgets import DiagnosticsView, heading, mono_font, scrolled, side_panel, spin

GEOM = [("m", "Mass m", "kg"), ("S", "Wing area S", "m²"), ("cbar", "Mean aerodynamic chord c̄", "m"),
        ("b", "Span b", "m"), ("Ixx", "Ixx", "kg m²"), ("Iyy", "Iyy", "kg m²"), ("Izz", "Izz", "kg m²"),
        ("Ixz", "Ixz", "kg m²")]
PROP = [("iT_deg", "Thrust line angle iT", "deg"), ("zT", "Thrust arm zT (+ below CG)", "m")]
LONG = ["CL0", "CLa", "CLq", "CLde", "CD0", "CDa", "CDq", "CDde", "Cm0", "Cma", "Cmq", "Cmde"]
LAT = ["CYb", "CYp", "CYr", "CYda", "CYdr", "Clb", "Clp", "Clr", "Clda", "Cldr", "Cnb", "Cnp", "Cnr", "Cnda", "Cndr"]


class AeroDialog(QDialog):
    def __init__(self, main):
        super().__init__(main)
        self.main = main
        self.setWindowTitle("Aerodynamic coefficients → trim → state-space matrices")
        self.resize(1400, 860)
        self.result = None
        split = QSplitter(Qt.Horizontal)
        QVBoxLayout(self).addWidget(split)
        left = QWidget()
        lv = QVBoxLayout(left)
        self.w = {}
        ad = al.AeroData.from_dict(main.project.aero) if getattr(main.project, "aero", None) else al.AeroData()

        def num(key, val, dec=6):
            s = spin(val, -1e12, 1e12, dec, 0.001)
            self.w[key] = s
            return s

        g = QGroupBox("Geometry and inertia"); f = QFormLayout(g)
        for k, lbl, u in GEOM:
            f.addRow(f"{lbl} [{u}]", num(k, getattr(ad, k), 4))
        lv.addWidget(g)
        g = QGroupBox("Propulsion (input 2 = thrust T [N])"); f = QFormLayout(g)
        for k, lbl, u in PROP:
            f.addRow(f"{lbl} [{u}]", num(k, getattr(ad, k), 4))
        lv.addWidget(g)
        g = QGroupBox("Longitudinal coefficients (per rad; CL0, CD0, Cm0 at alpha = 0)"); gl = QGridLayout(g)
        for i, k in enumerate(LONG):
            gl.addWidget(QLabel(k), i // 3, 2 * (i % 3)); gl.addWidget(num(k, getattr(ad, k)), i // 3, 2 * (i % 3) + 1)
        lv.addWidget(g)
        g = QGroupBox("Lateral-directional coefficients (per rad)"); gl = QGridLayout(g)
        for i, k in enumerate(LAT):
            gl.addWidget(QLabel(k), i // 3, 2 * (i % 3)); gl.addWidget(num(k, getattr(ad, k)), i // 3, 2 * (i % 3) + 1)
        lv.addWidget(g)
        g = QGroupBox("Rate derivatives"); f = QFormLayout(g)
        self.rates = QComboBox()
        self.rates.addItem("w.r.t. non-dimensional rates q̂ = qc̄/2V, p̂ = pb/2V, r̂ = rb/2V (AVL)", True)
        self.rates.addItem("w.r.t. dimensional rates q, p, r [rad/s]", False)
        self.rates.setCurrentIndex(0 if ad.rates_nondim else 1)
        f.addRow(self.rates)
        lv.addWidget(g)
        g = QGroupBox("Flight condition"); f = QFormLayout(g)
        f.addRow("Altitude h [m]", num("h", ad.h, 1))
        self.use_mach = QCheckBox("Give Mach instead of speed"); self.use_mach.setChecked(ad.use_mach)
        f.addRow(self.use_mach)
        f.addRow("True airspeed V [m/s]", num("V", ad.V, 3))
        f.addRow("Mach", num("mach", ad.mach, 4))
        f.addRow("Flight-path angle γ [deg]", num("gamma_deg", ad.gamma_deg, 3))
        lv.addWidget(g)
        g = QGroupBox("Mach sweep for gain scheduling (as aero_mach.m)"); f = QFormLayout(g)
        f.addRow("Coefficients given at Mach", num("mach_ref", ad.mach_ref, 3))
        f.addRow("Compressibility k in 1/√(1-(kM)²)", num("k_comp", ad.k_comp, 4))
        self.machs = QLineEdit("0.4 0.5 0.6 0.7 0.8")
        f.addRow("Mach values", self.machs)
        f.addRow(QLabel("k = 1: Prandtl-Glauert; k = 0.636: the fit of your AVL data (low aspect\n"
                        "ratio). Scaled: CLα, Cmα, Cmq, CLq, CLδe, Cmδe, CYβ, Clβ, Clp, Cnβ, Cnr.\n"
                        "Subsonic model; results above Mach ~0.8 are indicative only."))
        lv.addWidget(g)
        hb = QHBoxLayout()
        b = QPushButton("Trim && linearise"); b.clicked.connect(self.run); b.setDefault(True); hb.addWidget(b)
        b = QPushButton("Use in Model tab"); b.clicked.connect(self.use_in_model); hb.addWidget(b)
        lv.addLayout(hb)
        b = QPushButton("Create gain-schedule points (Mach sweep)"); b.clicked.connect(self.sweep)
        lv.addWidget(b)
        lv.addStretch(1)
        split.addWidget(side_panel(left))
        right = QWidget()
        rv = QVBoxLayout(right)
        rv.addWidget(heading("Trim and matrices"))
        self.out = QPlainTextEdit(); self.out.setReadOnly(True); self.out.setFont(mono_font(10))
        self.out.setMinimumHeight(470)
        rv.addWidget(self.out, 1)
        rv.addWidget(heading("Checks"))
        self.diag = DiagnosticsView(180)
        rv.addWidget(self.diag)
        split.addWidget(scrolled(right))
        split.setStretchFactor(1, 1)

    # ------------------------------------------------------------------
    def data(self):
        ad = al.AeroData()
        for k, s in self.w.items():
            setattr(ad, k, s.value())
        ad.rates_nondim = bool(self.rates.currentData())
        ad.use_mach = self.use_mach.isChecked()
        return ad

    def run(self):
        ad = self.data()
        self.main.project.aero = ad.to_dict()
        try:
            r = al.linearise(ad)
        except Exception as e:  # noqa: BLE001
            QMessageBox.warning(self, "Linearisation", str(e))
            return None
        self.result = r
        p = self.main.project
        L = []
        L.append(f"Flight condition: h = {r.h:.0f} m, V = {r.V:.2f} m/s, Mach = {r.mach:.3f}, "
                 f"rho = {r.rho:.4f} kg/m3")
        L.append(f"TRIM: alpha = {np.degrees(r.alpha0):+.3f} deg   delta_e = {np.degrees(r.de0):+.3f} deg   "
                 f"T = {r.T0:.0f} N   theta = {np.degrees(r.theta0):+.3f} deg   "
                 f"(residual {r.trim_residual:.1e}, {r.trim_iterations} iterations)")
        L.append("")
        L.append("A_long  states [V alpha q theta]:")
        L.append(format_matrix(r.A_long, 6))
        L.append("B_long  inputs [delta_e (rad), T (N)]:")
        L.append(format_matrix(r.B_long, 6))
        L.append("")
        L.append("A_lat  states [beta p r phi]:")
        L.append(format_matrix(r.A_lat, 6))
        L.append("B_lat  inputs [delta_a, delta_r]:")
        L.append(format_matrix(r.B_lat, 6))
        L.append("")
        ml, mt = long_modes(r.A_long), lat_modes(r.A_lat)
        na = r.info["n/alpha [g/rad]"]
        lvl = hq.evaluate_long(ml, p.cat, p.cls, na)
        lvt = hq.evaluate_lat(mt, p.cat, p.cls)
        L.append(f"Open-loop modes (class {p.cls}, category {p.cat}):")
        for k, md in ml.items():
            L.append(f"  {md.name:22s} {md.describe():42s} {hq.level_text(lvl[k])}")
        for k, md in mt.items():
            L.append(f"  {md.name:22s} {md.describe():42s} {hq.level_text(lvt.get(k, 4))}")
        L.append("")
        L.append("Derived quantities:")
        for k, v in r.info.items():
            L.append(f"  {k:28s} {v:.6g}")
        self.out.setPlainText("\n".join(L))
        items = [("warning" if ("not converge" in w or "negative" in w) else "caution",
                  w.split(":")[0], w.split(":", 1)[1].strip() if ":" in w else "", "")
                 for w in r.warnings]
        if not items:
            items = [("ok", "Trim converged, aircraft statically stable in pitch and yaw", "", "")]
        self.diag.show_items(items)
        self.main.status("Linearisation done.")
        return r

    def use_in_model(self):
        r = self.result or self.run()
        if r is None:
            return
        mt = self.main.model_tab
        mt.conv.setCurrentIndex(mt.conv.findData("Vaqt"))
        mt.V0.setValue(r.V); mt.mach.setValue(r.mach); mt.alt.setValue(r.h)
        mt.theta0.setValue(np.degrees(r.theta0))
        mt.Along.setPlainText(format_matrix(r.A_long, 10)); mt.Blong.setPlainText(format_matrix(r.B_long, 10))
        mt.Alat.setPlainText(format_matrix(r.A_lat, 10)); mt.Blat.setPlainText(format_matrix(r.B_lat, 10))
        mt.in_long.setText("de, dT"); mt.in_lat.setText("da, dr")
        mt.analyse()
        self.main.tabs.setCurrentIndex(0)
        self.main.status("Matrices from the aerodynamic data loaded in the Model tab ([V alpha q theta]; "
                         "thrust input in N).")

    def sweep(self):
        ad = self.data()
        try:
            machs = parse_matrix("[" + self.machs.text() + "]", name="Mach values").ravel()
        except MatrixParseError as e:
            QMessageBox.warning(self, "Mach sweep", str(e)); return
        self.main.project.aero = ad.to_dict()
        res = al.mach_sweep(ad, machs)
        pts = self.main.project.schedule_points
        bad = []
        for M, r in zip(machs, res):
            if r.trim_residual > 1e-6:
                bad.append(f"{M:g}")
                continue
            pts.append(FlightPoint(float(M), r.V, r.A_long, r.B_long, r.A_lat, r.B_lat))
        st = self.main.sched_tab
        st.conv.setCurrentIndex(st.conv.findData("Vaqt"))
        st.refresh()
        msg = f"{len(machs) - len(bad)} flight points added to the Gain schedule tab (h = {ad.h:.0f} m)."
        if bad:
            msg += f"  Trim failed at Mach {', '.join(bad)} (skipped)."
        QMessageBox.information(self, "Mach sweep", msg + "\nModel-tab input names should be 'de, dT' and 'da, dr'.")
        self.main.status(msg)
