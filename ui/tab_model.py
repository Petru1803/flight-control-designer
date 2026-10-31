"""Model tab: flight condition, longitudinal and lateral matrices, open-loop analysis."""
import math

import numpy as np
import scipy.io
from PySide6.QtWidgets import (QComboBox, QDialog, QDialogButtonBox, QFileDialog, QFormLayout, QGroupBox,
                               QHBoxLayout, QLabel, QLineEdit, QMessageBox, QPlainTextEdit, QPushButton,
                               QSplitter, QVBoxLayout, QWidget)
from PySide6.QtCore import Qt

from core import autopilot as ap
from core import hq
from core.models import LONG_CONVENTIONS, LatModel, LongModel, ModelError
from core.parsing import MatrixParseError, format_matrix, parse_matrix
from ui.project import MODE_HEADER, ctrb_text, lat_mode_rows, level_colors, long_mode_rows
from ui.widgets import DataTable, MatrixEdit, heading, mono_font, spin
from ui.widgets import side_panel


class ModelTab(QWidget):
    def __init__(self, main):
        super().__init__()
        self.main = main
        split = QSplitter(Qt.Horizontal)
        lay = QHBoxLayout(self)
        lay.addWidget(split)

        # ---------------- left: inputs ----------------
        left = QWidget()
        lv = QVBoxLayout(left)
        g = QGroupBox("Aircraft and flight condition")
        f = QFormLayout(g)
        self.name = QLineEdit("Aircraft")
        self.cls = QComboBox(); self.cls.addItems(hq.CLASSES); self.cls.setCurrentText("IV")
        self.cat = QComboBox(); self.cat.addItems(hq.CATEGORIES)
        self.cls.setToolTip("I small light, II-L/II-C medium (land/carrier), III large heavy, IV high-maneuverability")
        self.cat.setToolTip("A: rapid maneuvering/precision tracking, B: gradual (cruise, climb), C: terminal (takeoff, approach, landing)")
        self.V0 = spin(100.0, 0.1, 5000, 3, 1, "m/s")
        self.mach = spin(0.3, 0, 10, 3, 0.05)
        self.alt = spin(0.0, -500, 40000, 1, 100, "m")
        self.theta0 = spin(0.0, -90, 90, 3, 0.5, "deg")
        self.gg = spin(9.81, 1, 20, 4, 0.01, "m/s²")
        self.consts = QLineEdit()
        self.consts.setPlaceholderText("e.g.  alpha0 = 4.6*pi/180; x_nozzle = 12")
        self.consts.setToolTip("Named constants usable inside the matrices. Always available: "
                               "g, V0 (= TAS = V = U0 = Ue), Mach, theta0 (rad), pi")
        f.addRow("Name (for report)", self.name)
        f.addRow("Aircraft class", self.cls)
        f.addRow("Flight-phase category", self.cat)
        f.addRow("Trim speed V0 / TAS", self.V0)
        f.addRow("Mach", self.mach)
        f.addRow("Altitude", self.alt)
        f.addRow("Trim pitch angle θ0", self.theta0)
        f.addRow("g", self.gg)
        f.addRow("Constants", self.consts)
        lv.addWidget(g)

        g = QGroupBox("Longitudinal")
        f = QFormLayout(g)
        self.conv = QComboBox()
        for k, v in LONG_CONVENTIONS.items():
            self.conv.addItem(v, k)
        self.Along = MatrixEdit("[a11 a12 a13 a14; ... ]  (4x4, MATLAB syntax)")
        self.Blong = MatrixEdit("[b11 b12; ... ]  (4 x m)")
        self.in_long = QLineEdit("de, dT")
        self.in_long.setToolTip("Names of the B columns. 'dT' / throttle is treated as non-angular.")
        f.addRow("State convention", self.conv)
        f.addRow("A_long", self.Along)
        f.addRow("B_long", self.Blong)
        f.addRow("Input names", self.in_long)
        lv.addWidget(g)

        g = QGroupBox("Lateral-directional   x = [beta p r phi]")
        f = QFormLayout(g)
        self.Alat = MatrixEdit("[a11 a12 a13 a14; ... ]  (4x4)")
        self.Blat = MatrixEdit("[b11 b12; ... ]  (4 x m)")
        self.in_lat = QLineEdit("da, dr")
        f.addRow("A_lat", self.Alat)
        f.addRow("B_lat", self.Blat)
        f.addRow("Input names", self.in_lat)
        lv.addWidget(g)

        g = QGroupBox("Actuators (all control inputs)")
        f = QFormLayout(g)
        self.act_tau = spin(0.0, 0, 5, 3, 0.01, "s")
        self.act_tau.setToolTip("First-order lag 1/(tau s + 1) between command and surface. 0 = ideal.")
        self.act_delay = spin(0.0, 0, 2, 3, 0.005, "s")
        self.act_delay.setToolTip("Pure time delay (computation + transport), 1st-order Pade. 0 = none.")
        f.addRow("Time constant τ", self.act_tau)
        f.addRow("Time delay", self.act_delay)
        f.addRow(QLabel("Used by the autopilot design, the margins, the 1797A criteria\n"
                        "and the digital check (SAS targets are placed on the airframe)."))
        lv.addWidget(g)

        b = QPushButton("Build matrices from aerodynamic coefficients ...")
        b.clicked.connect(self.open_aero)
        lv.addWidget(b)
        b = QPushButton("Import matrices from a .mat file ...")
        b.clicked.connect(self.import_mat)
        lv.addWidget(b)
        self.btn = QPushButton("Parse && analyse open loop")
        self.btn.setDefault(True)
        self.btn.clicked.connect(self.analyse)
        lv.addWidget(self.btn)
        lv.addStretch(1)
        split.addWidget(side_panel(left))

        # ---------------- right: results ----------------
        right = QWidget()
        rv = QVBoxLayout(right)
        rv.addWidget(heading("Longitudinal open-loop modes"))
        self.t_long = DataTable(); rv.addWidget(self.t_long)
        self.l_long = QLabel(); self.l_long.setWordWrap(True); rv.addWidget(self.l_long)
        rv.addWidget(heading("Lateral-directional open-loop modes"))
        self.t_lat = DataTable(); rv.addWidget(self.t_lat)
        rv.addWidget(heading("Checks"))
        self.checks = QPlainTextEdit(); self.checks.setReadOnly(True); self.checks.setFont(mono_font(9))
        rv.addWidget(self.checks, 1)
        split.addWidget(right)
        split.setStretchFactor(1, 1)

    # ------------------------------------------------------------------
    def constants(self):
        V = self.V0.value()
        c = {"g": self.gg.value(), "V0": V, "TAS": V, "V": V, "U0": V, "Ue": V,
             "Mach": self.mach.value(), "theta0": math.radians(self.theta0.value())}
        txt = self.consts.text().strip()
        if txt:
            for part in txt.replace("\n", ";").split(";"):
                if not part.strip():
                    continue
                if "=" not in part:
                    raise MatrixParseError(f"Constant definition needs '=': '{part.strip()}'")
                k, v = part.split("=", 1)
                k = k.strip()
                if not k.isidentifier():
                    raise MatrixParseError(f"Invalid constant name '{k}'")
                c[k] = parse_matrix(v, c, k)[0, 0]
        return c

    def _names(self, le):
        return [s.strip() for s in le.text().replace(";", ",").split(",") if s.strip()]

    def build_models(self):
        """Parse inputs into project models. Returns list of error strings."""
        p = self.main.project
        errs = []
        try:
            consts = self.constants()
        except MatrixParseError as e:
            return [str(e)]
        p.name = self.name.text().strip() or "Aircraft"
        p.cat, p.cls = self.cat.currentText(), self.cls.currentText()
        p.consts = consts
        p.flight = {"V0 [m/s]": self.V0.value(), "Mach": self.mach.value(), "altitude [m]": self.alt.value(),
                    "theta0 [deg]": self.theta0.value(), "g": self.gg.value(),
                    "long. convention": self.conv.currentData()}
        old_long, old_lat = p.long_model, p.lat_model
        p.long_model = p.lat_model = None
        for ed in (self.Along, self.Blong, self.Alat, self.Blat):
            ed.mark(True)
        if self.Along.toPlainText().strip() or self.Blong.toPlainText().strip():
            try:
                A = self._parse(self.Along, consts, "A_long")
                B = self._parse(self.Blong, consts, "B_long")
                p.long_model = LongModel(A, B, self.conv.currentData(), self.V0.value(), self.gg.value(),
                                         self._names(self.in_long))
            except (MatrixParseError, ModelError) as e:
                errs.append(str(e))
        if self.Alat.toPlainText().strip() or self.Blat.toPlainText().strip():
            try:
                A = self._parse(self.Alat, consts, "A_lat")
                B = self._parse(self.Blat, consts, "B_lat")
                p.lat_model = LatModel(A, B, self.V0.value(), math.radians(self.theta0.value()),
                                       self.gg.value(), self._names(self.in_lat))
            except (MatrixParseError, ModelError) as e:
                errs.append(str(e))
        if not errs and p.long_model is None and p.lat_model is None:
            errs.append("Enter at least the longitudinal or the lateral matrices.")
        if _changed(old_long, p.long_model) or _changed(old_lat, p.lat_model):
            p.invalidate_sas()
        act = ap.ActuatorSpec(self.act_tau.value(), self.act_delay.value())
        if act != p.actuator:
            p.actuator = act
            p.ap.clear()
            p.hq1797 = None
        return errs

    def _parse(self, ed, consts, name):
        try:
            return parse_matrix(ed.toPlainText(), consts, name)
        except MatrixParseError as e:
            ed.mark(False, str(e))
            raise

    def analyse(self):
        errs = self.build_models()
        if errs:
            QMessageBox.warning(self, "Input error", "\n".join(errs))
            self.main.status("Input error")
            return False
        p = self.main.project
        lines = []
        if p.long_model is not None:
            m = p.long_model
            rows, _ = long_mode_rows(m.Ad, m, p.cat, p.cls)
            self.t_long.fill(MODE_HEADER, rows, level_colors(rows))
            L = hq.sp_limits(1, p.cat, p.cls, m.n_alpha())
            self.l_long.setText(
                f"n/α = {m.n_alpha():.3f} g/rad,  Tθ2 = {m.T_theta2():.3f} s.   Level 1 short period "
                f"(Cat {p.cat}, Class {p.cls}):  ωn {L['wn'][0]:.3f} – {L['wn'][1]:.3f} rad/s,  "
                f"ζ {L['zeta'][0]} – {L['zeta'][1]},  CAP {L['cap'][0]} – {L['cap'][1]}.   "
                f"Design states: [{', '.join(m.state_names)}]")
            if m.convention == "uwqt":
                lines.append("Longitudinal: converted [u w q theta] -> [u nz q theta] with nz = (w_dot - V0*q)/g "
                             "(same transformation T as in proiectv2.m).")
                lines.append("  max |eig(A) - eig(T A T^-1)| = "
                             f"{_eig_diff(m.A, m.Ad):.2e}  (must be ~0)")
            lines += ctrb_text(m, "long")
        else:
            self.t_long.fill(MODE_HEADER, [])
            self.l_long.setText("(no longitudinal model)")
        if p.lat_model is not None:
            rows, _ = lat_mode_rows(p.lat_model.Ad, p.cat, p.cls)
            self.t_lat.fill(MODE_HEADER, rows, level_colors(rows))
            t = hq.lat_level1_targets(p.cat, p.cls)
            lines.append(f"Level 1 lateral (Cat {p.cat}, Class {p.cls}): DR zeta >= {t['dr_zeta_min']}, "
                         f"zeta*wn >= {t['dr_zw_min']}, wn >= {t['dr_wn_min']} rad/s; roll tau <= "
                         f"{t['roll_tau_max']} s; spiral T2 >= {t['spiral_t2_min']} s")
            lines += ctrb_text(p.lat_model, "lat")
        else:
            self.t_lat.fill(MODE_HEADER, [])
        self.checks.setPlainText("\n".join(lines))
        self.main.model_changed()
        self.main.status("Model parsed and analysed.")
        return True

    def open_aero(self):
        from ui import theme
        from ui.aero_dialog import AeroDialog
        self._aero = AeroDialog(self.main)
        theme.capitals(self._aero)
        self._aero.show()

    def import_mat(self):
        path, _ = QFileDialog.getOpenFileName(self, "Import matrices", "", "MATLAB data (*.mat)")
        if not path:
            return
        try:
            data = scipy.io.loadmat(path)
        except Exception as e:  # noqa: BLE001
            QMessageBox.warning(self, "Import", f"Cannot read {path}:\n{e}")
            return
        mats = {k: np.asarray(v, float) for k, v in data.items()
                if not k.startswith("__") and isinstance(v, np.ndarray) and v.dtype.kind in "fiu"
                and v.ndim == 2 and v.shape[0] == 4}
        scalars = {k: float(np.asarray(v).ravel()[0]) for k, v in data.items()
                   if not k.startswith("__") and isinstance(v, np.ndarray) and v.dtype.kind in "fiu" and v.size == 1}
        if not mats:
            QMessageBox.information(self, "Import", "No 4-row numeric matrices found in this file.")
            return
        dlg = MatImportDialog(mats, self)
        if dlg.exec() != QDialog.Accepted:
            return
        for key, ed in (("A_long", self.Along), ("B_long", self.Blong), ("A_lat", self.Alat), ("B_lat", self.Blat)):
            name = dlg.choice(key)
            if name:
                ed.setPlainText(format_matrix(mats[name], prec=10))
        for k in ("TAS", "V0", "Ue", "U0", "V", "Vt"):
            if k in scalars and scalars[k] > 0:
                self.V0.setValue(scalars[k])
                break
        self.main.status(f"Matrices imported from {path}. Check the state convention, then press 'Parse & analyse'.")

    # for the gain-schedule tab
    def current_matrices(self):
        consts = self.constants()
        out = {}
        for key, ed in (("A_long", self.Along), ("B_long", self.Blong), ("A_lat", self.Alat), ("B_lat", self.Blat)):
            txt = ed.toPlainText().strip()
            out[key] = parse_matrix(txt, consts, key) if txt else None
        return out


class MatImportDialog(QDialog):
    """Assign the 4-row matrices of a .mat file to A_long, B_long, A_lat, B_lat."""
    GUESS = {"A_long": ("a_long", "along", "alon", "a_lon", "a_lg", "alg"),
             "B_long": ("b_long", "blong", "blon", "b_lon", "b_lg", "blg"),
             "A_lat": ("a_lat", "alat", "a_ld", "ald"),
             "B_lat": ("b_lat", "blat", "b_ld", "bld")}

    def __init__(self, mats, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Import matrices")
        f = QFormLayout(self)
        self.combos = {}
        for key in ("A_long", "B_long", "A_lat", "B_lat"):
            c = QComboBox()
            c.addItem("(none)", None)
            for name, M in mats.items():
                c.addItem(f"{name}  [{M.shape[0]}x{M.shape[1]}]", name)
            guess = next((n for n in mats if n.lower() in self.GUESS[key]), None)
            if guess is not None:
                c.setCurrentIndex(c.findData(guess))
            f.addRow(key, c)
            self.combos[key] = c
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept); bb.rejected.connect(self.reject)
        f.addRow(bb)

    def choice(self, key):
        return self.combos[key].currentData()


def _eig_diff(A1, A2):
    e1 = np.sort_complex(np.linalg.eigvals(A1))
    e2 = np.sort_complex(np.linalg.eigvals(A2))
    return float(np.max(np.abs(e1 - e2)))


def _changed(a, b):
    if (a is None) != (b is None):
        return True
    if a is None:
        return False
    return not (np.array_equal(a.A, b.A) and np.array_equal(a.B, b.B)
                and getattr(a, "convention", None) == getattr(b, "convention", None)
                and a.V0 == b.V0 and a.input_names == b.input_names)
