"""SAS tab: longitudinal and lateral stability augmentation by pole placement."""
import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QCheckBox, QComboBox, QFormLayout, QGroupBox, QHBoxLayout, QLabel,
                               QMessageBox, QPlainTextEdit, QPushButton, QScrollArea, QSplitter,
                               QVBoxLayout, QWidget)

from core import plots, sas
from ui.project import MODE_HEADER, R2D, lat_mode_rows, level_colors, long_mode_rows, sas_gain_rows
from core import diagnose
from ui.widgets import DataTable, DiagnosticsView, PlotTabs, heading, mono_font, scrolled, spin
from ui.widgets import side_panel


class SASTab(QWidget):
    def __init__(self, main):
        super().__init__()
        self.main = main
        split = QSplitter(Qt.Horizontal)
        QHBoxLayout(self).addWidget(split)

        left = QWidget()
        lv = QVBoxLayout(left)
        # ---- longitudinal
        g = QGroupBox("Longitudinal SAS (pole placement, Ackermann)")
        f = QFormLayout(g)
        self.l_input = QComboBox()
        self.l_struct = QComboBox()
        self.l_struct.addItem("Full state  [u/V, nz/alpha, q, theta]", "full")
        self.l_struct.addItem("Short period only  [nz/alpha, q]", "sp")
        self.wsp = spin(2.0, 0.01, 100, 4, 0.1, "rad/s")
        self.zsp = spin(0.7, 0.01, 3, 3, 0.05)
        self.wph = spin(0.1, 0.0005, 10, 4, 0.01, "rad/s")
        self.zph = spin(0.6, 0.0, 3, 3, 0.05)
        f.addRow("Control input", self.l_input)
        f.addRow("Feedback structure", self.l_struct)
        f.addRow("Short period ωn", self.wsp)
        f.addRow("Short period ζ", self.zsp)
        f.addRow("Phugoid ωn", self.wph)
        f.addRow("Phugoid ζ", self.zph)
        hb = QHBoxLayout()
        b = QPushButton("Auto targets (Level 1)"); b.clicked.connect(self.auto_long); hb.addWidget(b)
        b = QPushButton("Design longitudinal"); b.clicked.connect(self.design_long); hb.addWidget(b)
        f.addRow(hb)
        lv.addWidget(g)
        # ---- lateral
        g = QGroupBox("Lateral-directional SAS (pole placement)")
        f = QFormLayout(g)
        self.lat_inputs_box = QHBoxLayout()
        self.lat_checks = []
        w = QWidget(); w.setLayout(self.lat_inputs_box)
        self.wdr = spin(1.0, 0.01, 100, 4, 0.1, "rad/s")
        self.zdr = spin(0.6, 0.01, 3, 3, 0.05)
        self.proll = spin(-2.0, -100, -0.001, 4, 0.1, "1/s")
        self.pspir = spin(-0.02, -100, 1, 4, 0.01, "1/s")
        self.washout = spin(0.0, 0, 30, 2, 0.5, "s")
        self.washout.setToolTip("Yaw-rate washout s/(s + 1/tau) on the r feedback, so the yaw damper does not "
                                "oppose steady turns. 0 = no washout. Typical 1-4 s.")
        f.addRow("Inputs used", w)
        f.addRow("Dutch roll ωn", self.wdr)
        f.addRow("Dutch roll ζ", self.zdr)
        f.addRow("Roll pole", self.proll)
        f.addRow("Spiral pole", self.pspir)
        f.addRow("Yaw-rate washout τw", self.washout)
        hb = QHBoxLayout()
        b = QPushButton("Auto targets (Level 1)"); b.clicked.connect(self.auto_lat); hb.addWidget(b)
        b = QPushButton("Design lateral"); b.clicked.connect(self.design_lat); hb.addWidget(b)
        f.addRow(hb)
        lv.addWidget(g)
        g = QGroupBox("Automatic targets")
        f = QFormLayout(g)
        self.min_eff = QCheckBox("Minimise control effort (search the Level 1 region)")
        self.min_eff.setChecked(True)
        self.sas_lim = spin(30.0, 1, 90, 1, 1, "deg")
        self.sas_lim.setToolTip("Surface deflection used to normalise the effort of the SAS "
                                "(non-angular inputs such as thrust are not limited here).")
        f.addRow(self.min_eff)
        f.addRow("Surface deflection limit", self.sas_lim)
        f.addRow(QLabel("Effort = largest surface command for a 5° α / 5°/s q disturbance\n"
                        "(longitudinal) or 5° β, 10°/s p, 5°/s r (lateral). Unticked:\n"
                        "classic targets (ζsp = 0.7, ωn near natural; ζdr ≥ 0.6)."))
        lv.addWidget(g)
        b = QPushButton("Auto-design both axes")
        b.clicked.connect(self.auto_both)
        lv.addWidget(b)
        lv.addWidget(heading("Results"))
        self.out = QPlainTextEdit(); self.out.setReadOnly(True); self.out.setFont(mono_font(9))
        self.out.setMinimumHeight(220)
        lv.addWidget(self.out, 1)
        lv.addWidget(heading("Diagnostics"))
        self.diag = DiagnosticsView(160)
        lv.addWidget(self.diag)
        split.addWidget(side_panel(left))

        right = QWidget()
        rv = QVBoxLayout(right)
        rv.addWidget(heading("Gains  (u = -K x)"))
        self.t_klong = DataTable(); rv.addWidget(self.t_klong)
        self.t_klat = DataTable(); rv.addWidget(self.t_klat)
        rv.addWidget(heading("Closed-loop modes"))
        self.t_mlong = DataTable(); rv.addWidget(self.t_mlong)
        self.t_mlat = DataTable(); rv.addWidget(self.t_mlat)
        self.plots = PlotTabs(["Long poles", "SP gain domain", "C* / q-q_ss", "Long response",
                               "Lat poles", "Lat response"])
        rv.addWidget(self.plots, 1)
        split.addWidget(scrolled(right))
        split.setStretchFactor(1, 1)

    # ------------------------------------------------------------------
    def refresh_inputs(self):
        p = self.main.project
        self.l_input.clear()
        if p.long_model is not None:
            self.l_input.addItems(p.long_model.input_names)
        for c in self.lat_checks:
            self.lat_inputs_box.removeWidget(c); c.deleteLater()
        self.lat_checks = []
        if p.lat_model is not None:
            for i, nm in enumerate(p.lat_model.input_names):
                c = QCheckBox(nm); c.setChecked(i < 2)
                self.lat_inputs_box.addWidget(c)
                self.lat_checks.append(c)
        if p.long_model is not None:
            self.auto_long(quiet=True)
        if p.lat_model is not None:
            self.auto_lat(quiet=True)

    def _need(self, which):
        p = self.main.project
        m = p.long_model if which == "long" else p.lat_model
        if m is None:
            QMessageBox.information(self, "No model", f"Enter and parse the {which}itudinal matrices first."
                                    if which == "long" else "Enter and parse the lateral matrices first.")
        return m

    def auto_long(self, quiet=False):
        p = self.main.project
        m = p.long_model if quiet else self._need("long")
        if m is None:
            return
        t = sas.auto_long_targets(m, p.cat, p.cls, self.min_eff.isChecked(), self._lims(m)[0],
                                  max(self.l_input.currentIndex(), 0), self.l_struct.currentData())
        self.wsp.setValue(t.wn_sp); self.zsp.setValue(t.zeta_sp)
        self.wph.setValue(t.wn_ph); self.zph.setValue(t.zeta_ph)

    def auto_lat(self, quiet=False):
        p = self.main.project
        m = p.lat_model if quiet else self._need("lat")
        if m is None:
            return
        idx = [i for i, c in enumerate(self.lat_checks) if c.isChecked()] or None
        t = sas.auto_lat_targets(m, p.cat, p.cls, self.min_eff.isChecked(), self._lims(m), idx,
                                 self.washout.value())
        self.wdr.setValue(t.wn_dr); self.zdr.setValue(t.zeta_dr)
        self.proll.setValue(t.roll_pole); self.pspir.setValue(t.spiral_pole)

    def _lims(self, model):
        """Effort normalisation per input: the deflection limit for surfaces, a large value
        for thrust-type inputs (which the SAS does not use)."""
        from core.autopilot import is_angle_input
        lim = np.radians(self.sas_lim.value())
        return np.array([lim if is_angle_input(nm) else 1e12 for nm in model.input_names])

    def auto_both(self):
        p = self.main.project
        if p.long_model is not None:
            self.auto_long(True); self.design_long()
        if p.lat_model is not None:
            self.auto_lat(True); self.design_lat()

    def design_long(self):
        p = self.main.project
        m = self._need("long")
        if m is None:
            return
        t = sas.LongSASTargets(self.wsp.value(), self.zsp.value(), self.wph.value(), self.zph.value(),
                               self.l_struct.currentData(), max(self.l_input.currentIndex(), 0))
        try:
            r = sas.design_long_sas(m, t, p.cat, p.cls)
        except Exception as e:  # noqa: BLE001
            QMessageBox.warning(self, "Longitudinal SAS", str(e))
            return
        p.long_sas = r
        p.ap = {k: v for k, v in p.ap.items() if k not in ("theta", "alt", "speed")}
        h, rows = sas_gain_rows(r, m)
        self.t_klong.fill(h, rows)
        rows, _ = long_mode_rows(r.Acl, m, p.cat, p.cls)
        self.t_mlong.fill(MODE_HEADER, rows, level_colors(rows))
        self.plots.set("Long poles", plots.pz_long(m, r, p.cat, p.cls))
        P, K = sas.sp_gain_domain(m, p.cat, p.cls, t.input_index)
        self.plots.set("SP gain domain", plots.sp_domain(m, r, p.cat, p.cls, P, K, t.input_index))
        fig, chk = plots.cstar_qdiag(m, r, t.input_index)
        p.cstar_checks = chk
        self.plots.set("C* / q-q_ss", fig)
        self.plots.set("Long response", plots.sas_response_long(m, r, t.input_index))
        p.hq1797 = None
        self.plots.setCurrentIndex(0)
        self._report()
        self.main.sas_changed()

    def design_lat(self):
        p = self.main.project
        m = self._need("lat")
        if m is None:
            return
        idx = [i for i, c in enumerate(self.lat_checks) if c.isChecked()]
        if not idx:
            QMessageBox.information(self, "Lateral SAS", "Select at least one input.")
            return
        t = sas.LatSASTargets(self.wdr.value(), self.zdr.value(), self.proll.value(), self.pspir.value(), idx,
                              self.washout.value())
        try:
            r = sas.design_lat_sas(m, t, p.cat, p.cls)
        except Exception as e:  # noqa: BLE001
            QMessageBox.warning(self, "Lateral SAS", f"Pole placement failed: {e}")
            return
        p.lat_sas = r
        p.ap = {k: v for k, v in p.ap.items() if k not in ("roll", "heading", "beta")}
        h, rows = sas_gain_rows(r, m)
        self.t_klat.fill(h, rows)
        rows, _ = lat_mode_rows(r.Acl, p.cat, p.cls)
        self.t_mlat.fill(MODE_HEADER, rows, level_colors(rows))
        self.plots.set("Lat poles", plots.pz_lat(m, r, p.cat, p.cls))
        self.plots.set("Lat response", plots.sas_response_lat(m, r))
        p.hq1797 = None
        self.plots.setCurrentIndex(4)
        self._report()
        self.main.sas_changed()

    def _report(self):
        p = self.main.project
        L = []
        if p.long_sas is not None:
            r, m = p.long_sas, p.long_model
            L.append("LONGITUDINAL SAS")
            L.append(f"  K ({m.input_names[r.targets.input_index]}) = " +
                     "  ".join(f"K_{s}={k:.5g}" for s, k in zip(m.state_names, r.K[r.targets.input_index])))
            L.append("  closed-loop poles: " + ", ".join(f"{z.real:.4f}{z.imag:+.4f}j" for z in r.poles))
            L.append(f"  Level: SP {r.levels_ol['sp']} -> {r.levels_cl['sp']},  phugoid "
                     f"{r.levels_ol['ph']} -> {r.levels_cl['ph']}")
            L.append("  effort for a 5 deg alpha perturbation: " +
                     ", ".join(f"{nm} = {e * R2D:.2f} deg" for nm, e in zip(m.input_names, r.effort_alpha5)
                               if abs(e) > 0))
            if p.cstar_checks and p.cstar_checks.get("cstar") is not None:
                L.append(f"  C* inside envelope: {100 * p.cstar_checks['cstar']:.0f}% of samples,  "
                         f"q/q_ss: {100 * p.cstar_checks['qdiag']:.0f}%")
            L += ["  WARNING: " + w for w in r.warnings]
        if p.lat_sas is not None:
            r, m = p.lat_sas, p.lat_model
            L.append("LATERAL SAS")
            for i, nm in enumerate(m.input_names):
                if not np.allclose(r.K[i], 0):
                    L.append(f"  K ({nm}) = " + "  ".join(f"K_{s}={k:.5g}" for s, k in zip(m.state_names, r.K[i])))
            L.append("  closed-loop poles: " + ", ".join(f"{z.real:.4f}{z.imag:+.4f}j" for z in r.poles))
            if r.targets.washout_tau > 0:
                L.append(f"  yaw-rate washout tau = {r.targets.washout_tau:g} s (poles above include the washout; "
                         "the SAS uses r - w instead of r)")
            L.append("  Level: " + ", ".join(f"{k} {r.levels_ol[k]} -> {r.levels_cl[k]}" for k in r.levels_cl))
            L.append("  effort for a 5 deg beta perturbation: " +
                     ", ".join(f"{nm} = {e * R2D:.2f} deg" for nm, e in zip(m.input_names, r.effort_beta5)))
            big = [nm for nm, e in zip(m.input_names, r.effort_beta5) if abs(e * R2D) > 30]
            if big:
                L.append(f"  WARNING: effort above 30 deg on {', '.join(big)} - consider a milder target "
                         "(category B / lower DR frequency) or check the control derivatives.")
            L += ["  WARNING: " + w for w in r.warnings]
        self.out.setPlainText("\n".join(L))
        self.diag.show_items(diagnose.sas(p))
