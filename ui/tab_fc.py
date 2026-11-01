"""Flight-controller tab: designs for PX4 / ArduPilot fixed-wing autopilots and their export
(firmware parameters, C code, PX4 module)."""
import datetime
import os

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QCheckBox, QComboBox, QFileDialog, QFormLayout, QGroupBox, QHBoxLayout, QLabel,
                               QLineEdit, QMessageBox, QPushButton, QSplitter, QVBoxLayout, QWidget)

from core import cancel, fccode, plots
from core import fcexport as fx
from ui.widgets import DataTable, DiagnosticsView, PlotTabs, heading, run_async, scrolled, side_panel, spin

DEG = np.pi / 180


def fc_job(long_model, lat_model, spec, progress=None):
    out = {}
    for axis in ("pitch", "roll"):
        struct = spec["structures"][axis]
        if struct == "none":
            continue
        if progress:
            progress(f"Designing {axis} ({fx.STRUCTURES[struct]}) ...")
        out[axis] = fx.design(axis, struct, long_model, lat_model, spec["dmax"], spec["firmware"],
                              spec["actuator"], spec["airspeed"], spec["Ts"],
                              None if struct == "PID" else spec["prefilter"], specs=spec["specs"])
    return out


class FCTab(QWidget):
    def __init__(self, main):
        super().__init__()
        self.main = main
        self._job = None
        split = QSplitter(Qt.Horizontal)
        QHBoxLayout(self).addWidget(split)
        left = QWidget()
        lv = QVBoxLayout(left)

        g = QGroupBox("Target autopilot")
        f = QFormLayout(g)
        self.fw = QComboBox(); self.fw.addItems(fx.FIRMWARES)
        self.fw.currentIndexChanged.connect(self._fw_changed)
        f.addRow("Firmware", self.fw)
        self.fw_note = QLabel(); self.fw_note.setWordWrap(True)
        f.addRow(self.fw_note)
        lv.addWidget(g)

        g = QGroupBox("Aircraft and sensors")
        f = QFormLayout(g)
        self.d_e = spin(25.0, 1, 60, 1, 1, "deg")
        self.d_a = spin(20.0, 1, 60, 1, 1, "deg")
        self.d_r = spin(25.0, 1, 60, 1, 1, "deg")
        f.addRow("Elevator max deflection", self.d_e)
        f.addRow("Aileron max deflection", self.d_a)
        f.addRow("Rudder max deflection", self.d_r)
        self.trim_e = spin(0.0, -30, 30, 3, 0.1, "deg")
        f.addRow("Trim elevator δe0", self.trim_e)
        self.airspeed = QCheckBox("Airspeed sensor (pitot)"); self.airspeed.setChecked(True)
        f.addRow(self.airspeed)
        f.addRow(QLabel("Measured: body rates p, q, r; roll and pitch angles;\n"
                        "airspeed (if fitted). Never alpha or beta.\n"
                        "Actuator lag / delay: from the Model tab."))
        lv.addWidget(g)

        g = QGroupBox("Design")
        f = QFormLayout(g)
        self.s_pitch = QComboBox(); self.s_roll = QComboBox()
        for cb in (self.s_pitch, self.s_roll):
            for k, v in fx.STRUCTURES.items():
                cb.addItem(v, k)
        f.addRow("Pitch", self.s_pitch)
        f.addRow("Roll + yaw", self.s_roll)
        self.rate = spin(100.0, 20, 1000, 0, 10, "Hz")
        f.addRow("Controller rate", self.rate)
        self.pf = spin(2.0, 0, 20, 2, 0.5, "rad/s")
        f.addRow("Setpoint prefilter (LQG/H∞)", self.pf)
        self.gm = spin(6.0, 0, 30, 1, 0.5, "dB")
        self.pm = spin(35.0, 0, 90, 1, 1, "deg")
        self.dm = spin(0.2, 0, 2, 3, 0.01, "s")
        f.addRow("Gain margin ≥", self.gm)
        f.addRow("Phase margin ≥", self.pm)
        f.addRow("Delay margin ≥", self.dm)
        f.addRow(QLabel("The digital delay (1.5 samples) is part of the design model,\n"
                        "so the margins hold on the autopilot. Small fast UAVs can use\n"
                        "a smaller delay margin (e.g. 0.05 s) for a quicker response."))
        lv.addWidget(g)

        hb = QHBoxLayout()
        self.b_run = QPushButton("Design"); self.b_run.clicked.connect(self.run)
        self.b_cancel = QPushButton("Cancel"); self.b_cancel.setEnabled(False)
        self.b_cancel.clicked.connect(lambda: (cancel.request(), self.main.status("Cancelling ...")))
        hb.addWidget(self.b_run); hb.addWidget(self.b_cancel)
        lv.addLayout(hb)

        g = QGroupBox("Export")
        f = QFormLayout(g)
        self.folder = QLineEdit(os.path.join(os.path.expanduser("~"), "Documents", "flight_controller"))
        b = QPushButton("..."); b.setMaximumWidth(40); b.clicked.connect(self._browse)
        hb = QHBoxLayout(); hb.addWidget(self.folder, 1); hb.addWidget(b)
        f.addRow("Folder", hb)
        self.b_export = QPushButton("Export"); self.b_export.clicked.connect(self.export)
        f.addRow(self.b_export)
        self.log = QLabel(""); self.log.setWordWrap(True)
        f.addRow(self.log)
        lv.addWidget(g)
        lv.addStretch(1)
        split.addWidget(side_panel(left))

        right = QWidget()
        rv = QVBoxLayout(right)
        rv.addWidget(heading("Designs"))
        self.t_res = DataTable(); rv.addWidget(self.t_res)
        rv.addWidget(heading("Firmware parameters"))
        self.t_par = DataTable(max_height=360); rv.addWidget(self.t_par)
        rv.addWidget(heading("Diagnostics"))
        self.diag = DiagnosticsView(120); rv.addWidget(self.diag)
        self.plots = PlotTabs(["Attitude steps", "Surfaces"])
        self.plots.setMinimumHeight(520)
        rv.addWidget(self.plots, 1)
        split.addWidget(scrolled(right))
        split.setStretchFactor(1, 1)
        self._fw_changed()

    # ------------------------------------------------------------------
    def _fw_changed(self):
        if self.fw.currentText() == "PX4":
            self.fw_note.setText("PID structure → PX4 parameters (FW_PR_*, FW_RR_*, FW_YR_*). "
                                 "Any structure → C code + PX4 module fw_ss_control (replaces fw_rate_control).")
        else:
            self.fw_note.setText("ArduPlane accepts only its own PID structure: use 'Firmware PID structure' "
                                 "on both axes to export a .param file. LQG / H∞ are not loadable in ArduPilot.")

    def _browse(self):
        d = QFileDialog.getExistingDirectory(self, "Export folder", self.folder.text())
        if d:
            self.folder.setText(d)

    def _theta0(self):
        return float(self.main.model_tab.theta0.value()) * DEG

    def _prefill_trim(self):
        """Trim elevator from the aerodynamic data, when the model came from the coefficient window."""
        aero = getattr(self.main.project, "aero", None)
        if not aero:
            return
        try:
            from core import aero_lin as al
            r = al.linearise(al.AeroData.from_dict(aero))
            self.trim_e.setValue(np.degrees(r.de0))
        except Exception:  # noqa: BLE001
            pass

    def showEvent(self, ev):
        if abs(self.trim_e.value()) < 1e-9:
            self._prefill_trim()
        super().showEvent(ev)

    def spec(self):
        return {"firmware": self.fw.currentText(),
                "dmax": {"de": self.d_e.value() * DEG, "da": self.d_a.value() * DEG, "dr": self.d_r.value() * DEG},
                "structures": {"pitch": self.s_pitch.currentData(), "roll": self.s_roll.currentData()},
                "actuator": self.main.project.actuator, "airspeed": self.airspeed.isChecked(),
                "Ts": 1.0 / self.rate.value(), "prefilter": self.pf.value(),
                "specs": (self.gm.value(), self.pm.value(), self.dm.value())}

    def run(self):
        p = self.main.project
        if p.long_model is None or p.lat_model is None:
            QMessageBox.information(self, "Flight controller", "Enter and analyse both models in the Model tab first.")
            return
        sp = self.spec()
        # surface limits by the model's own input names
        sp["dmax"] = self._limits_by_name(p, sp["dmax"])
        self.b_run.setEnabled(False); self.b_cancel.setEnabled(True); self.main.set_busy(True)
        cancel.reset()
        self._job = run_async(fc_job, self._done, self._failed, self.main.status, p.long_model, p.lat_model, sp)

    @staticmethod
    def _limits_by_name(p, roles):
        from core import autopilot as ap
        out = {}
        for model, rl in ((p.long_model, ("de",)), (p.lat_model, ("da", "dr"))):
            for role in rl:
                j = ap._find_input(role, model.input_names)
                out[model.input_names[j]] = roles[role]
        for model in (p.long_model, p.lat_model):
            for nm in model.input_names:
                out.setdefault(nm, np.inf)
        return out

    def _done(self, res):
        self.b_run.setEnabled(True); self.b_cancel.setEnabled(False); self.main.set_busy(False)
        self.main.project.fc = {"designs": res, "spec": self.spec()}
        self.show_results()
        self.main.status("Flight-controller designs done.")

    def _failed(self, msg):
        self.b_run.setEnabled(True); self.b_cancel.setEnabled(False); self.main.set_busy(False)
        if "Cancelled" in msg.splitlines()[0]:
            self.main.status("Flight-controller design cancelled.")
            return
        QMessageBox.critical(self, "Flight controller", msg[:3000])

    # ------------------------------------------------------------------
    def show_results(self):
        fc = getattr(self.main.project, "fc", None)
        if not fc:
            return
        ds = fc["designs"]
        head = ["axis", "structure", "stable", "GM [dB]", "PM [deg]", "DM [s]", "rise [s]", "overshoot [%]",
                "peak surface [deg]", "digital PM [deg]", "controller states", "sensors"]
        rows, col, items = [], {}, []
        for i, (a, d) in enumerate(ds.items()):
            r, mg, m = d.result, d.result.margins, d.result.metrics[0]
            dg = d.digital or {}
            peak = ", ".join(f"{d.plant.input_names[j]} {r.peak_effort[j]:.1f}" for j in d.plant.js)
            rows.append([a, d.structure, "yes" if r.stable else "NO", mg["gm_db"], mg["pm"], mg["dm"],
                         m.get("rise", np.nan), m.get("overshoot", np.nan), peak,
                         dg.get("margins", {}).get("pm", np.nan), d.ctrl.nc,
                         ", ".join(d.plant.state_names[k] for k in d.plant.meas)])
            ok = all(r.feasible.values()) and dg.get("stable", False)
            col[(i, 2)] = "ok" if r.stable else "bad"
            col[(i, 4)] = "ok" if r.feasible.get("pm") else "bad"
            col[(i, 5)] = "ok" if r.feasible.get("dm") else "bad"
            col[(i, 9)] = "ok" if dg.get("stable") else "bad"
            if ok:
                items.append(("ok", f"{a}: {d.structure} meets the requirements",
                              f"GM {mg['gm_db']:.1f} dB, PM {mg['pm']:.1f} deg, DM {mg['dm']:.3f} s; "
                              f"digital at {1 / d.Ts:.0f} Hz stable (PM {dg['margins']['pm']:.1f} deg).", ""))
            else:
                bad = [k for k, v in r.feasible.items() if not v]
                items.append(("warning", f"{a}: {d.structure} misses {', '.join(bad) or 'digital stability'}",
                              "Try another structure, a faster controller rate, or larger surface limits.", ""))
        self.t_res.fill(head, rows, col)
        prm = self._params(ds, fc["spec"]["firmware"])
        self.t_par.fill(["parameter", "value", "meaning"], [[n, v, c] for n, v, c in prm] or
                        [["(none)", "", "parameters are exported only for the firmware PID structure"]])
        fw = fc["spec"]["firmware"]
        if fw == "ArduPilot" and any(d.structure != "PID" for d in ds.values()):
            items.append(("warning", "ArduPilot: only the PID structure can be exported",
                          "Choose 'Firmware PID structure' on both axes to get a .param file.", ""))
        self.diag.show_items(items)
        self.plots.set("Attitude steps", plots.fc_response(ds))
        self.plots.set("Surfaces", plots.fc_surfaces(ds))

    def _params(self, ds, fw):
        pid = {a: d for a, d in ds.items() if d.structure == "PID"}
        if not pid:
            return []
        p = self.main.project
        return fx.pid_parameters(pid, fw, p.long_model.V0, float(self.main.model_tab.alt.value()))

    # ------------------------------------------------------------------
    def export(self):
        fc = getattr(self.main.project, "fc", None)
        if not fc:
            QMessageBox.information(self, "Export", "Design the flight controller first.")
            return
        try:
            files = self.export_to(self.folder.text())
        except Exception as e:  # noqa: BLE001
            QMessageBox.critical(self, "Export", str(e))
            return
        self.log.setText("Written:\n" + "\n".join(os.path.relpath(x, self.folder.text()) for x in files))
        self.main.status(f"Flight controller exported to {self.folder.text()}")

    def export_to(self, folder):
        fc = self.main.project.fc
        ds, fw = fc["designs"], fc["spec"]["firmware"]
        p = self.main.project
        name = p.name or "aircraft"
        safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in name)
        os.makedirs(folder, exist_ok=True)
        stamp = datetime.date.today().isoformat()
        header = [f"{name} - generated by SAS & Autopilot Designer on {stamp}",
                  f"design airspeed {p.long_model.V0:.2f} m/s TAS at {self.main.model_tab.alt.value():.0f} m",
                  "check every value before flight; verify in SITL first"]
        written = []
        prm = self._params(ds, fw)
        if fw == "ArduPilot":
            if set(ds) != {"pitch", "roll"} or any(d.structure != "PID" for d in ds.values()):
                raise ValueError("ArduPilot export needs the firmware PID structure on both axes.")
            path = os.path.join(folder, f"{safe}.param")
            fx.write_params(path, prm, fw, header)
            written.append(path)
        else:
            if prm:
                path = os.path.join(folder, f"{safe}_px4.params")
                fx.write_params(path, prm, fw, header + ["load in QGroundControl: Parameters > Tools > Load from file"])
                written.append(path)
            if set(ds) == {"pitch", "roll"}:
                th0 = self._theta0()
                trim = {}
                j = ds["pitch"].plant.js[0]
                trim[ds["pitch"].plant.input_names[j]] = self.trim_e.value() * DEG
                dcs = [fx.to_discrete(ds["pitch"], th0, trim), fx.to_discrete(ds["roll"], 0.0)]
                meta = {"name": name, "V0": p.long_model.V0, "stamp": stamp,
                        "dmax": {k: v for k, v in ds["pitch"].dmax.items()},
                        "structures": {a: fx.STRUCTURES[d.structure] for a, d in ds.items()}}
                written += fccode.write_files(os.path.join(folder, "fw_ss_control"), fccode.px4_module(dcs, meta))
        return written

    # ------------------------------------------------------------------
    def state(self):
        return {"fw": self.fw.currentIndex(), "de": self.d_e.value(), "da": self.d_a.value(), "dr": self.d_r.value(),
                "trim_e": self.trim_e.value(), "airspeed": self.airspeed.isChecked(),
                "pitch": self.s_pitch.currentIndex(), "roll": self.s_roll.currentIndex(),
                "rate": self.rate.value(), "pf": self.pf.value(), "folder": self.folder.text(),
                "specs": [self.gm.value(), self.pm.value(), self.dm.value()]}

    def set_state(self, s):
        self.fw.setCurrentIndex(s.get("fw", 0)); self.d_e.setValue(s.get("de", 25)); self.d_a.setValue(s.get("da", 20))
        self.d_r.setValue(s.get("dr", 25)); self.trim_e.setValue(s.get("trim_e", 0.0))
        self.airspeed.setChecked(s.get("airspeed", True)); self.s_pitch.setCurrentIndex(s.get("pitch", 0))
        self.s_roll.setCurrentIndex(s.get("roll", 0)); self.rate.setValue(s.get("rate", 100))
        self.pf.setValue(s.get("pf", 2.0)); self.folder.setText(s.get("folder", self.folder.text()))
        gm, pm, dm = s.get("specs", [6.0, 35.0, 0.2])
        self.gm.setValue(gm); self.pm.setValue(pm); self.dm.setValue(dm)
