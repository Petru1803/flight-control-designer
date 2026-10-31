"""Gain-schedule tab: flight points on Mach, automatic design at each point, interpolation."""
import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QComboBox, QFormLayout, QGroupBox, QHBoxLayout, QLabel, QMessageBox,
                               QPlainTextEdit, QPushButton, QScrollArea, QSplitter, QVBoxLayout, QWidget)

from core import autopilot as ap
from core import cancel, plots
from core.models import LONG_CONVENTIONS
from core.parsing import MatrixParseError, format_matrix, parse_matrix
from core.schedule import FlightPoint, check_interpolated, run_schedule
from ui.widgets import DataTable, MatrixEdit, PlotCanvas, heading, mono_font, run_async, scrolled, spin
from ui.widgets import side_panel


class ScheduleTab(QWidget):
    def __init__(self, main):
        super().__init__()
        self.main = main
        split = QSplitter(Qt.Horizontal)
        QHBoxLayout(self).addWidget(split)
        left = QWidget()
        lv = QVBoxLayout(left)
        lv.addWidget(heading("Flight points"))
        self.t_pts = DataTable()
        self.t_pts.itemSelectionChanged.connect(self.load_selected)
        lv.addWidget(self.t_pts)
        hb = QHBoxLayout()
        for txt, fn in (("Add from Model tab", self.add_from_model), ("New empty", self.add_empty),
                        ("Remove", self.remove)):
            b = QPushButton(txt); b.clicked.connect(fn); hb.addWidget(b)
        lv.addLayout(hb)
        g = QGroupBox("Selected point (input names as in the Model tab)")
        f = QFormLayout(g)
        self.mach = spin(0.5, 0, 10, 3, 0.05)
        self.V0 = spin(150, 0.1, 5000, 3, 1, "m/s")
        self.label = QLabel("")
        self.Al = MatrixEdit("A_long (4x4)", 4); self.Bl = MatrixEdit("B_long (4xm)", 4)
        self.Aa = MatrixEdit("A_lat (4x4, optional)", 4); self.Ba = MatrixEdit("B_lat (4xm, optional)", 4)
        f.addRow("Mach", self.mach); f.addRow("V0", self.V0)
        f.addRow("A_long", self.Al); f.addRow("B_long", self.Bl)
        f.addRow("A_lat", self.Aa); f.addRow("B_lat", self.Ba)
        b = QPushButton("Save point"); b.clicked.connect(self.save_point)
        f.addRow(b)
        lv.addWidget(g)
        g = QGroupBox("Schedule options")
        f = QFormLayout(g)
        self.conv = QComboBox()
        for k, v in LONG_CONVENTIONS.items():
            self.conv.addItem(v, k)
        f.addRow("Longitudinal states of the points", self.conv)
        self.ap_mode = QComboBox()
        self.ap_mode.addItem("SAS only", None)
        for k, v in ap.MODES.items():
            self.ap_mode.addItem(f"SAS + LQI {v['label']}", k)
        f.addRow("Scheduled gains", self.ap_mode)
        f.addRow(QLabel("At every point the SAS targets are chosen automatically inside Level 1\n"
                        "(class/category from the Model tab)."))
        self.b_run = QPushButton("Compute gain schedule"); self.b_run.clicked.connect(self.run)
        self.b_cancel = QPushButton("Cancel"); self.b_cancel.setEnabled(False)
        self.b_cancel.clicked.connect(lambda: (cancel.request(), self.main.status("Cancelling ...")))
        hb = QHBoxLayout(); hb.addWidget(self.b_run); hb.addWidget(self.b_cancel)
        f.addRow(hb)
        lv.addWidget(g)
        g = QGroupBox("Interpolation (linear in Mach, linear extrapolation outside)")
        f = QFormLayout(g)
        self.qm = spin(0.6, 0, 10, 3, 0.01)
        self.qm.valueChanged.connect(self.query)
        self.qout = QPlainTextEdit(); self.qout.setReadOnly(True); self.qout.setFont(mono_font(9))
        self.qout.setMinimumHeight(120)
        f.addRow("Mach", self.qm); f.addRow(self.qout)
        lv.addWidget(g)
        lv.addStretch(1)
        split.addWidget(side_panel(left))

        right = QWidget()
        rv = QVBoxLayout(right)
        rv.addWidget(heading("Results per flight point"))
        self.t_res = DataTable(); rv.addWidget(self.t_res)
        self.t_cross = DataTable(); rv.addWidget(self.t_cross)
        self.warn = QLabel(); self.warn.setWordWrap(True); rv.addWidget(self.warn)
        self.canvas = PlotCanvas(); rv.addWidget(self.canvas, 1)
        split.addWidget(scrolled(right))
        split.setStretchFactor(1, 1)
        self._sel = None
        self.refresh()

    # ------------------------------------------------------------------
    def refresh(self):
        pts = self.main.project.schedule_points
        rows = [[i + 1, p.mach, p.V0, "yes" if p.A_lat is not None else "no"] for i, p in enumerate(pts)]
        self.t_pts.blockSignals(True)
        self.t_pts.fill(["#", "Mach", "V0 [m/s]", "lateral"], rows)
        self.t_pts.blockSignals(False)

    def add_from_model(self):
        mt = self.main.model_tab
        try:
            mats = mt.current_matrices()
        except MatrixParseError as e:
            QMessageBox.warning(self, "Model", str(e)); return
        if mats["A_long"] is None or mats["B_long"] is None:
            QMessageBox.information(self, "Model", "The Model tab has no longitudinal matrices."); return
        pt = FlightPoint(mt.mach.value(), mt.V0.value(), mats["A_long"], mats["B_long"],
                         mats["A_lat"], mats["B_lat"])
        if not self.main.project.schedule_points:
            self.conv.setCurrentIndex(mt.conv.currentIndex())
        self.main.project.schedule_points.append(pt)
        self.refresh()
        self.t_pts.selectRow(len(self.main.project.schedule_points) - 1)

    def add_empty(self):
        pts = self.main.project.schedule_points
        m = (max(p.mach for p in pts) + 0.1) if pts else 0.4
        pts.append(FlightPoint(m, 150.0, np.zeros((4, 4)), np.zeros((4, 1))))
        self.refresh()
        self.t_pts.selectRow(len(pts) - 1)

    def remove(self):
        r = self.t_pts.currentRow()
        if 0 <= r < len(self.main.project.schedule_points):
            del self.main.project.schedule_points[r]
            self._sel = None
            self.refresh()

    def load_selected(self):
        r = self.t_pts.currentRow()
        pts = self.main.project.schedule_points
        if not 0 <= r < len(pts):
            return
        self._sel = r
        p = pts[r]
        self.mach.setValue(p.mach); self.V0.setValue(p.V0)
        self.Al.setPlainText(format_matrix(p.A_long)); self.Bl.setPlainText(format_matrix(p.B_long))
        self.Aa.setPlainText(format_matrix(p.A_lat) if p.A_lat is not None else "")
        self.Ba.setPlainText(format_matrix(p.B_lat) if p.B_lat is not None else "")

    def save_point(self):
        pts = self.main.project.schedule_points
        if self._sel is None or not 0 <= self._sel < len(pts):
            QMessageBox.information(self, "Schedule", "Select or add a point first."); return
        try:
            consts = self.main.model_tab.constants()
            consts.update({"V0": self.V0.value(), "TAS": self.V0.value(), "V": self.V0.value(),
                           "U0": self.V0.value(), "Ue": self.V0.value(), "Mach": self.mach.value()})
            Al = parse_matrix(self.Al.toPlainText(), consts, "A_long")
            Bl = parse_matrix(self.Bl.toPlainText(), consts, "B_long")
            Aa = parse_matrix(self.Aa.toPlainText(), consts, "A_lat") if self.Aa.toPlainText().strip() else None
            Ba = parse_matrix(self.Ba.toPlainText(), consts, "B_lat") if self.Ba.toPlainText().strip() else None
        except MatrixParseError as e:
            QMessageBox.warning(self, "Schedule", str(e)); return
        if Al.shape != (4, 4) or Bl.shape[0] != 4 or (Aa is not None and (Aa.shape != (4, 4) or Ba is None
                                                                          or Ba.shape[0] != 4)):
            QMessageBox.warning(self, "Schedule", "A must be 4x4 and B must have 4 rows."); return
        pts[self._sel] = FlightPoint(self.mach.value(), self.V0.value(), Al, Bl, Aa, Ba)
        sel = self._sel
        self.refresh()
        self.t_pts.selectRow(sel)
        self.main.status(f"Flight point {sel + 1} saved.")

    def job_args(self):
        p = self.main.project
        mt = self.main.model_tab
        mode = self.ap_mode.currentData()
        st = None
        if mode:
            info = ap.MODES[mode]
            st = ap.APSettings(ref=info["ref"], T=info["T"], x0_track=info.get("x0", [0.0]))
        return (p.schedule_points, self.conv.currentData(), p.cat, p.cls, mt._names(mt.in_long),
                mt._names(mt.in_lat), mt.gg.value(), mode, st, np.radians(mt.theta0.value()))

    def run(self):
        pts = self.main.project.schedule_points
        if not pts:
            QMessageBox.information(self, "Schedule", "Add flight points first."); return
        if any(np.allclose(pt.A_long, 0) for pt in pts):
            QMessageBox.information(self, "Schedule", "Some points have empty A_long."); return
        self.b_run.setEnabled(False); self.b_cancel.setEnabled(True); self.main.set_busy(True)
        cancel.reset()
        self._job = run_async(_sched_job, self._done, self._failed, self.main.status, *self.job_args())

    def _done(self, out):
        res, cross = out
        self.b_run.setEnabled(True); self.b_cancel.setEnabled(False); self.main.set_busy(False)
        self.main.project.schedule = res
        keys = list(res.table[0].keys()) if res.table else []
        self.t_res.fill(keys, [[r.get(k, "") for k in keys] for r in res.table])
        if cross:
            kk = list(cross[0].keys())
            self.t_cross.fill(kk, [[c[k] for k in kk] for c in cross])
        self.warn.setText("\n".join(res.warnings) if res.warnings else
                          "All flight points designed. Bottom table: each point's gains tested on the "
                          "neighbouring points' plants (robustness between nodes).")
        self.canvas.set_figure(plots.schedule_plot(res))
        if len(res.mach):
            self.qm.setValue(float(np.mean(res.mach)))
        self.query()
        self.main.status("Gain schedule computed.")

    def _failed(self, msg):
        self.b_run.setEnabled(True); self.b_cancel.setEnabled(False); self.main.set_busy(False)
        if "Cancelled" in msg.splitlines()[0]:
            self.main.status("Gain schedule cancelled.")
            return
        QMessageBox.critical(self, "Gain schedule", msg[:3000])

    def query(self):
        res = self.main.project.schedule
        if res is None:
            return
        M = self.qm.value()
        lines = [f"Mach {M:.3f}:"]
        K = res.interp(M, "long")
        for i, nm in enumerate(res.long_inputs[:K.shape[0]]):
            if not np.allclose(K[i], 0):
                lines.append(f"  long {nm}: " + "  ".join(f"K_{s}={v:.5g}" for s, v in zip(res.long_names, K[i])))
        if res.Klat is not None:
            K = res.interp(M, "lat")
            for i, nm in enumerate(res.lat_inputs[:K.shape[0]]):
                if not np.allclose(K[i], 0):
                    lines.append(f"  lat  {nm}: " + "  ".join(f"K_{s}={v:.5g}" for s, v in zip(res.lat_names, K[i])))
        if res.Kap is not None:
            K = res.interp(M, "ap")
            lines.append("  AP LQI: " + "  ".join(f"{n}={v:.5g}" for n, v in zip(res.ap_names or [], K)))
        if M < res.mach.min() or M > res.mach.max():
            lines.append("  (extrapolated - outside the designed Mach range)")
        self.qout.setPlainText("\n".join(lines))

    def figures(self):
        return [("Gain schedule", self.canvas.fig)] if self.canvas.fig.axes else []


def _sched_job(pts, conv, cat, cls, lin, lat, g, mode, st, theta0, progress=None):
    res = run_schedule(pts, conv, cat, cls, lin, lat, g, mode, st, theta0, progress)
    cross = check_interpolated(res, pts, conv, g)
    return res, cross
