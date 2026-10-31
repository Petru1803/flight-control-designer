"""Robustness tab: Monte Carlo on the aerodynamic derivatives + disk margins."""
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QCheckBox, QComboBox, QFormLayout, QGroupBox, QHBoxLayout, QLabel, QListWidget,
                               QListWidgetItem, QMessageBox, QPushButton, QSplitter, QVBoxLayout, QWidget)

from core import autopilot as ap
from core import cancel, diagnose, hq, plots, robust
from ui.widgets import DataTable, DiagnosticsView, PlotTabs, heading, run_async, scrolled, side_panel, spin


class RobustTab(QWidget):
    def __init__(self, main):
        super().__init__()
        self.main = main
        split = QSplitter(Qt.Horizontal)
        QHBoxLayout(self).addWidget(split)
        left = QWidget()
        lv = QVBoxLayout(left)
        g = QGroupBox("Monte Carlo on the aerodynamic derivatives")
        f = QFormLayout(g)
        self.frac = spin(20.0, 1, 100, 1, 5, "%")
        self.n = spin(100, 10, 5000, 0, 50)
        self.seed = spin(1, 0, 1e6, 0, 1)
        f.addRow("Uncertainty ±", self.frac)
        f.addRow("Samples", self.n)
        f.addRow("Random seed", self.seed)
        f.addRow(QLabel("Every derivative of A and B (forces / moments) is scaled\n"
                        "independently by a uniform factor in [1-x, 1+x]; kinematic\n"
                        "and gravity terms stay exact. SAS gains and autopilots are\n"
                        "kept at their nominal values."))
        lv.addWidget(g)
        g = QGroupBox("What to analyse")
        f = QFormLayout(g)
        self.c_long = QCheckBox("Longitudinal SAS"); self.c_long.setChecked(True)
        self.c_lat = QCheckBox("Lateral SAS"); self.c_lat.setChecked(True)
        f.addRow(self.c_long); f.addRow(self.c_lat)
        f.addRow(QLabel("Autopilot modes already designed:"))
        self.modes = QListWidget(); self.modes.setMaximumHeight(150)
        f.addRow(self.modes)
        lv.addWidget(g)
        hb = QHBoxLayout()
        self.b_run = QPushButton("Run analysis"); self.b_run.clicked.connect(self.run)
        self.b_cancel = QPushButton("Cancel"); self.b_cancel.setEnabled(False)
        self.b_cancel.clicked.connect(lambda: (cancel.request(), self.main.status("Cancelling ...")))
        hb.addWidget(self.b_run); hb.addWidget(self.b_cancel)
        lv.addLayout(hb)
        lv.addWidget(heading("Diagnostics"))
        self.diag = DiagnosticsView(240)
        lv.addWidget(self.diag)
        lv.addStretch(1)
        split.addWidget(side_panel(left))
        right = QWidget()
        rv = QVBoxLayout(right)
        rv.addWidget(heading("SAS"))
        self.t_sas = DataTable(); rv.addWidget(self.t_sas)
        rv.addWidget(heading("Autopilot (Monte Carlo)"))
        self.t_ap = DataTable(); rv.addWidget(self.t_ap)
        rv.addWidget(heading("Nominal disk margins"))
        self.t_disk = DataTable(); rv.addWidget(self.t_disk)
        hb = QHBoxLayout()
        hb.addWidget(QLabel("Autopilot mode shown in the plots:"))
        self.plot_mode = QComboBox(); self.plot_mode.currentIndexChanged.connect(self._plots_ap)
        hb.addWidget(self.plot_mode, 1)
        rv.addLayout(hb)
        self.plots = PlotTabs(["Long SAS poles", "Lat SAS poles", "AP margins", "AP response spread"])
        rv.addWidget(self.plots, 1)
        split.addWidget(scrolled(right))
        split.setStretchFactor(1, 1)

    def showEvent(self, ev):
        self.refresh_modes()
        super().showEvent(ev)

    def refresh_modes(self):
        checked = {self.modes.item(i).data(Qt.UserRole) for i in range(self.modes.count())
                   if self.modes.item(i).checkState() == Qt.Checked}
        first = self.modes.count() == 0
        self.modes.clear()
        for m in self.main.project.ap:
            it = QListWidgetItem(ap.MODES[m]["label"])
            it.setData(Qt.UserRole, m)
            it.setFlags(it.flags() | Qt.ItemIsUserCheckable)
            it.setCheckState(Qt.Checked if (first or m in checked) else Qt.Unchecked)
            self.modes.addItem(it)

    def run(self):
        p = self.main.project
        if p.long_sas is None and p.lat_sas is None:
            QMessageBox.information(self, "Robustness", "Design the SAS first.")
            return
        modes = [self.modes.item(i).data(Qt.UserRole) for i in range(self.modes.count())
                 if self.modes.item(i).checkState() == Qt.Checked]
        self.b_run.setEnabled(False); self.b_cancel.setEnabled(True); self.main.set_busy(True)
        cancel.reset()
        self._job = run_async(robust.run, self._done, self._failed, self.main.status,
                              p, self.frac.value() / 100, int(self.n.value()), int(self.seed.value()),
                              self.c_long.isChecked(), self.c_lat.isChecked(), modes)

    def _done(self, res):
        self.b_run.setEnabled(True); self.b_cancel.setEnabled(False); self.main.set_busy(False)
        p = self.main.project
        p.robust = res
        self.show_results()
        self.main.status(f"Robustness analysis done ({res['n']} samples, ±{100 * res['frac']:.0f}%).")

    def _failed(self, msg):
        self.b_run.setEnabled(True); self.b_cancel.setEnabled(False); self.main.set_busy(False)
        if "Cancelled" in msg.splitlines()[0]:
            self.main.status("Robustness analysis cancelled.")
            return
        QMessageBox.critical(self, "Robustness", msg[:3000])

    def show_results(self):
        p = self.main.project
        res = p.robust
        if not res:
            return
        a, b, c = robust.summary_rows(res, p)
        col = {}
        for i, r in enumerate(a["rows"]):
            col[(i, 1)] = "ok" if r[1] >= 100 else ("warn" if r[1] >= 90 else "bad")
            col[(i, 2)] = "ok" if r[2] >= 90 else ("warn" if r[2] >= 50 else "bad")
        self.t_sas.fill(a["head"], a["rows"], col)
        col = {}
        for i, r in enumerate(b["rows"]):
            col[(i, 2)] = "ok" if r[2] >= 100 else "bad"
            col[(i, 3)] = "ok" if r[3] >= 90 else ("warn" if r[3] >= 50 else "bad")
        self.t_ap.fill(b["head"], b["rows"], col)
        col = {(i, 3): "ok" if r[3] >= 6 else "bad" for i, r in enumerate(c["rows"])}
        col.update({(i, 4): "ok" if r[4] >= 35 else "bad" for i, r in enumerate(c["rows"])})
        self.t_disk.fill(c["head"], c["rows"], col)
        self.diag.show_items(diagnose.robustness(p))
        if "long" in res["sas"]:
            self.plots.set("Long SAS poles", plots.robust_poles(res["sas"]["long"], p.long_sas.poles,
                                                                "Longitudinal SAS", region=p.long_sas.sp_limits))
        if "lat" in res["sas"]:
            self.plots.set("Lat SAS poles", plots.robust_poles(res["sas"]["lat"], p.lat_sas.poles, "Lateral SAS",
                                                               lat_limits=hq.lat_level1_targets(p.cat, p.cls)))
        modes = sorted({m for (m, _) in res["ap"]}, key=list(ap.MODES).index)
        self.plot_mode.blockSignals(True)
        self.plot_mode.clear()
        for m in modes:
            self.plot_mode.addItem(ap.MODES[m]["label"], m)
        self.plot_mode.blockSignals(False)
        self._plots_ap()

    def _plots_ap(self):
        p = self.main.project
        res = p.robust
        m = self.plot_mode.currentData()
        if not res or m is None or m not in p.ap:
            return
        st = p.ap[m]["settings"]
        self.plots.set("AP margins", plots.robust_margins(res["ap"], m, (st.gm_spec, st.pm_spec, st.dm_spec)))
        self.plots.set("AP response spread", plots.robust_spread(res["ap"], m, p.ap[m]["plant"], st))

    def state(self):
        return {"frac": self.frac.value(), "n": self.n.value(), "seed": self.seed.value(),
                "long": self.c_long.isChecked(), "lat": self.c_lat.isChecked()}

    def set_state(self, s):
        self.frac.setValue(s.get("frac", 20)); self.n.setValue(s.get("n", 100)); self.seed.setValue(s.get("seed", 1))
        self.c_long.setChecked(s.get("long", True)); self.c_lat.setChecked(s.get("lat", True))
