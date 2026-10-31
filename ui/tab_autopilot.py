"""Autopilot tab: mode selection, automatic tuning of PIDF / LQI / LQG / H-inf / loop shaping /
cascade, comparison of margins, effort, noise, saturation (anti-windup) and digital implementation."""
import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QCheckBox, QComboBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
                               QLabel, QMessageBox, QPlainTextEdit, QPushButton, QScrollArea, QSplitter,
                               QVBoxLayout, QWidget)

from core import autopilot as ap
from core import cancel, digital, plots, sim
from core.parsing import parse_matrix
from ui.project import ap_extra_rows, ap_summary_rows, ctrl_gain_rows, digital_rows
from core import diagnose
from ui.widgets import (DataTable, DiagnosticsView, PlotTabs, fmt, heading, mono_font, run_async, scrolled,
                        spin)
from ui.widgets import side_panel

DEG = np.pi / 180
NOISE_DEFAULT = {"deg": 0.5, "m": 1.0, "m/s": 0.2}


class AutopilotTab(QWidget):
    def __init__(self, main):
        super().__init__()
        self.main = main
        self._busy = False
        split = QSplitter(Qt.Horizontal)
        QHBoxLayout(self).addWidget(split)

        left = QWidget()
        lv = QVBoxLayout(left)
        g = QGroupBox("Mode and manoeuvre")
        f = QFormLayout(g)
        self.mode = QComboBox()
        for k, v in ap.MODES.items():
            self.mode.addItem(v["label"], k)
        self.mode.currentIndexChanged.connect(self.mode_changed)
        self.inp = QComboBox()
        self.ref = spin(5, -1e6, 1e6, 3, 1)
        self.ref2 = spin(0, -1e6, 1e6, 3, 1)
        self.ref2_label = QLabel("Reference 2")
        self.x0 = spin(0, -1e6, 1e6, 3, 1)
        self.T = spin(20, 1, 3600, 1, 5, "s")
        self.dt = spin(0.01, 0.0005, 1, 4, 0.005, "s")
        self.pf = spin(2.0, 0, 100, 3, 0.5, "rad/s")
        self.pf.setToolTip("Reference prefilter pf/(s+pf) (unit DC gain). 0 = no prefilter.")
        self.bank = spin(30, 0, 90, 1, 5, "deg")
        self.bank.setToolTip("Maximum bank angle for heading modes (penalised in tuning). 0 = no limit.")
        f.addRow("Autopilot mode", self.mode)
        f.addRow("Control input", self.inp)
        f.addRow("Reference step", self.ref)
        f.addRow(self.ref2_label, self.ref2)
        f.addRow("Initial offset of tracked state", self.x0)
        f.addRow("Simulation time", self.T)
        f.addRow("Time step", self.dt)
        f.addRow("Prefilter pole", self.pf)
        f.addRow("Bank-angle limit", self.bank)
        lv.addWidget(g)

        g = QGroupBox("Methods")
        gl = QGridLayout(g)
        self.meth = {}
        for i, m in enumerate(ap.METHODS):
            c = QCheckBox(ap.METHOD_LABELS[m]); c.setChecked(True)
            self.meth[m] = c
            gl.addWidget(c, i // 2, i % 2)
        lv.addWidget(g)

        g = QGroupBox("Requirements")
        f = QFormLayout(g)
        self.gm = spin(6, 0, 60, 2, 1, "dB")
        self.pm = spin(35, 0, 90, 2, 1, "deg")
        self.dm = spin(0.2, 0, 100, 3, 0.05, "s")
        f.addRow("Gain margin ≥", self.gm)
        f.addRow("Phase margin ≥", self.pm)
        f.addRow("Delay margin ≥", self.dm)
        self.lim_table = DataTable(editable=True)
        f.addRow(QLabel("Actuator limits (deflection, rate); effort must stay inside"))
        f.addRow(self.lim_table)
        lv.addWidget(g)

        g = QGroupBox("Noise, saturation, anti-windup")
        f = QFormLayout(g)
        self.nstd = spin(0.5, 0, 1e6, 4, 0.1)
        self.nbw = spin(25, 0, 1e4, 2, 5, "rad/s")
        self.sat = QCheckBox("Simulate actuator saturation and rate limit"); self.sat.setChecked(True)
        self.aw = QCheckBox("Anti-windup (conditional integration) on integrators"); self.aw.setChecked(True)
        f.addRow("Noise std on tracked measurement", self.nstd)
        f.addRow("Noise bandwidth", self.nbw)
        f.addRow(self.sat)
        f.addRow(self.aw)
        lv.addWidget(g)

        hb = QHBoxLayout()
        self.b_tune = QPushButton("Auto-tune"); self.b_tune.clicked.connect(self.auto_tune)
        self.b_tune.setToolTip("Automatically tune all selected methods")
        self.b_re = QPushButton("Recompute (edited)"); self.b_re.clicked.connect(self.recompute)
        self.b_re.setToolTip("Rebuild the controllers from the edited parameters below")
        self.b_cancel = QPushButton("Cancel"); self.b_cancel.clicked.connect(self.cancel)
        self.b_cancel.setEnabled(False)
        hb.addWidget(self.b_tune); hb.addWidget(self.b_re); hb.addWidget(self.b_cancel)
        lv.addLayout(hb)
        lv.addWidget(heading("Tuned parameters (editable)"))
        self.par = DataTable(editable=True, max_height=480)
        lv.addWidget(self.par)

        g = QGroupBox("Digital implementation")
        f = QFormLayout(g)
        self.fs = spin(50, 1, 10000, 1, 10, "Hz")
        self.dmeth = QComboBox()
        for k, v in digital.METHODS.items():
            self.dmeth.addItem(k, v)
        self.cdelay = QCheckBox("One-sample computational delay"); self.cdelay.setChecked(True)
        self.auto_fs = QCheckBox("Auto sample rate after every tuning (ωs ≈ 30 × ωc)")
        self.auto_fs.setChecked(True)
        self.auto_fs.toggled.connect(lambda on: self.fs.setEnabled(not on))
        self.fs.setEnabled(False)
        f.addRow(self.auto_fs)
        f.addRow("Sample rate", self.fs)
        f.addRow("Controller discretisation", self.dmeth)
        f.addRow(self.cdelay)
        hb = QHBoxLayout()
        b = QPushButton("Suggest sample rate"); b.clicked.connect(self.suggest_fs); hb.addWidget(b)
        self.b_dig = QPushButton("Re-check digital"); self.b_dig.clicked.connect(self.check_digital)
        self.b_dig.setToolTip("Run the digital check again with the sample rate / method chosen here")
        hb.addWidget(self.b_dig)
        f.addRow(hb)
        f.addRow(QLabel("The digital check runs automatically after every tuning.\n"
                        "SAS gains and controllers run at the same rate; the airframe\n"
                        "and actuators are discretised exactly (ZOH)."))
        lv.addWidget(g)
        lv.addStretch(1)
        split.addWidget(side_panel(left))

        right = QWidget()
        rv = QVBoxLayout(right)
        rv.addWidget(heading("Comparison"))
        self.t_sum = DataTable(); rv.addWidget(self.t_sum)
        self.t_ext = DataTable(); rv.addWidget(self.t_ext)
        self.t_dig = DataTable(); rv.addWidget(self.t_dig)
        rv.addWidget(heading("Diagnostics"))
        self.diag = DiagnosticsView(150)
        rv.addWidget(self.diag)
        rv.addWidget(heading("Gains"))
        self.gains = QPlainTextEdit(); self.gains.setReadOnly(True); self.gains.setFont(mono_font(9))
        self.gains.setMaximumHeight(150)
        rv.addWidget(self.gains)
        self.plots = PlotTabs(["Response", "Control effort", "Loop (Bode)", "Noise", "Saturation", "Digital"])
        rv.addWidget(self.plots, 1)
        split.addWidget(scrolled(right))
        split.setStretchFactor(1, 1)
        self.mode_changed()

    # ------------------------------------------------------------------
    def current_mode(self):
        return self.mode.currentData()

    def mode_changed(self):
        mode = self.current_mode()
        info = ap.MODES[mode]
        p = self.main.project
        model = p.long_model if info["axis"] == "long" else p.lat_model
        mimo = len(info["tracks"]) > 1
        self.inp.clear()
        if model is not None:
            if mimo:
                self.inp.addItem(" + ".join(model.input_names[ap._find_input(r, model.input_names)]
                                            for r in info["inputs"]))
            else:
                self.inp.addItems(model.input_names)
                self.inp.setCurrentIndex(ap.default_input(mode, model.input_names))
        self.inp.setEnabled(not mimo)
        units = info["units"]
        self.ref.setSuffix(" " + units[0]); self.x0.setSuffix(" " + units[0]); self.nstd.setSuffix(" " + units[0])
        self.ref.setValue(info["ref"][0])
        self.ref2_label.setVisible(mimo); self.ref2.setVisible(mimo)
        if mimo:
            self.ref2_label.setText(f"Reference {info['tracks'][1]}")
            self.ref2.setSuffix(" " + units[1]); self.ref2.setValue(info["ref"][1])
        self.x0.setValue(info.get("x0", [0.0])[0])
        self.T.setValue(info["T"])
        self.nstd.setValue(NOISE_DEFAULT[units[0]])
        self.bank.setEnabled(info["axis"] == "lat")
        allowed = ap.methods_for(mode)
        for m, c in self.meth.items():
            c.setEnabled(m in allowed)
            c.setChecked(m in allowed and (m != "Cascade" or mode == "heading"))
        self._fill_limits(model)
        entry = p.ap.get(mode)
        if entry:
            self.show_entry(entry)
        else:
            for t in (self.t_sum, self.t_ext, self.t_dig, self.par):
                t.fill([], [])
            self.gains.clear()
            self.diag.clear()

    def _fill_limits(self, model):
        rows = []
        if model is not None:
            for nm in model.input_names:
                if ap.is_angle_input(nm):
                    rows.append([nm, 30.0, 60.0, "deg, deg/s"])
                else:
                    rows.append([nm, "inf", "inf", "input units"])
        self.lim_table.fill(["input", "max |deflection|", "max rate", "units"], rows)

    def _read_limits(self, plant):
        m = plant.B.shape[1]
        lim, rate = np.full(m, np.inf), np.full(m, np.inf)
        for i in range(min(m, self.lim_table.rowCount())):
            for arr, col in ((lim, 1), (rate, 2)):
                it = self.lim_table.item(i, col)
                try:
                    v = float(it.text()) if it else np.inf
                except ValueError:
                    v = np.inf
                arr[i] = v / plant.input_scale[i] if np.isfinite(v) else np.inf
        return lim, rate

    def settings(self, plant):
        lim, rate = self._read_limits(plant)
        info = ap.MODES[self.current_mode()]
        refs = [self.ref.value()] + ([self.ref2.value()] if plant.p > 1 else [])
        x0 = [self.x0.value()] + [0.0] * (plant.p - 1)
        noise = [self.nstd.value()] + [NOISE_DEFAULT[u] for u in info["units"][1:]]
        return ap.APSettings(ref=refs, T=self.T.value(), dt=self.dt.value(), prefilter=self.pf.value(),
                             limits=lim, rates=rate, gm_spec=self.gm.value(), pm_spec=self.pm.value(),
                             dm_spec=self.dm.value(), x0_track=x0, noise_std=noise,
                             noise_bw=self.nbw.value(), use_saturation=self.sat.isChecked(),
                             antiwindup=self.aw.isChecked(), bank_limit=self.bank.value())

    def _plant(self, quiet=False):
        p = self.main.project
        mode = self.current_mode()
        try:
            idx = max(self.inp.currentIndex(), 0) if len(ap.MODES[mode]["tracks"]) == 1 else None
            return ap.build_plant(mode, p.long_model, p.long_sas, p.lat_model, p.lat_sas, idx, p.actuator)
        except ap.APError as e:
            if not quiet:
                QMessageBox.information(self, "Autopilot", str(e))
            return None

    # ------------------------------------------------------------------
    def _set_busy(self, b):
        self._busy = b
        self.b_tune.setEnabled(not b); self.b_re.setEnabled(not b); self.b_dig.setEnabled(not b)
        self.b_cancel.setEnabled(b)
        self.main.set_busy(b)

    def cancel(self):
        cancel.request()
        self.main.status("Cancelling ...")

    def auto_tune(self):
        if self._busy:
            return
        plant = self._plant()
        if plant is None:
            return
        methods = [m for m, c in self.meth.items() if c.isChecked() and c.isEnabled()]
        if not methods:
            QMessageBox.information(self, "Autopilot", "Select at least one method.")
            return
        st = self.settings(plant)
        self._set_busy(True)
        cancel.reset()
        self._job = run_async(tune_job, self._done, self._failed, self.main.status,
                              plant, st, methods, self.current_mode(), None, digital_cfg=self.digital_cfg())

    def recompute(self):
        if self._busy:
            return
        p = self.main.project
        mode = self.current_mode()
        if mode not in p.ap:
            QMessageBox.information(self, "Autopilot", "Run the auto-tuning first.")
            return
        plant = self._plant()
        if plant is None:
            return
        try:
            params = self._read_params()
        except ValueError as e:
            QMessageBox.warning(self, "Parameters", str(e))
            return
        st = self.settings(plant)
        self._set_busy(True)
        cancel.reset()
        self._job = run_async(tune_job, self._done, self._failed, self.main.status,
                              plant, st, list(params), mode, params, digital_cfg=self.digital_cfg())

    def digital_cfg(self):
        return {"auto": self.auto_fs.isChecked(), "Ts": 1.0 / self.fs.value(),
                "method": self.dmeth.currentData(), "delay": self.cdelay.isChecked()}

    def _done(self, entry):
        self._set_busy(False)
        dg = entry.get("digital")
        if dg:
            self.fs.setValue(round(1.0 / next(iter(dg.values()))["Ts"], 1))
        if not entry["ctrls"]:
            self.main.status("Autopilot: cancelled, nothing computed." if entry.get("cancelled") else
                             "Autopilot: no method succeeded.")
            if entry.get("errors"):
                QMessageBox.warning(self, "Autopilot", "\n\n".join(f"{k}: {v}" for k, v in entry["errors"].items()))
            return
        self.main.project.ap[entry["mode"]] = entry
        if entry["mode"] == self.current_mode():
            self.show_entry(entry)
        errs = entry.get("errors") or {}
        msg = f"Autopilot '{ap.MODES[entry['mode']]['label']}' done."
        if entry.get("cancelled"):
            msg = "Cancelled - showing the methods finished so far."
        if errs:
            msg += "  Failed: " + ", ".join(errs)
            QMessageBox.warning(self, "Autopilot", "\n\n".join(f"{k}: {v}" for k, v in errs.items()))
        self.main.status(msg)

    def _failed(self, msg):
        self._set_busy(False)
        if "Cancelled" in msg.splitlines()[0]:
            self.main.status("Cancelled.")
            return
        QMessageBox.critical(self, "Autopilot error", msg[:3000])
        self.main.status("Autopilot design failed.")

    # ------------------------------------------------------------------
    def suggest_fs(self):
        p = self.main.project
        e = p.ap.get(self.current_mode())
        if not e:
            QMessageBox.information(self, "Digital", "Design the autopilot first.")
            return
        Ts, wmax = digital.suggest_ts(e["plant"], list(e["ctrls"].values()), method=self.dmeth.currentData(),
                                      delay=self.cdelay.isChecked())
        self.auto_fs.setChecked(False)
        self.fs.setValue(round(1.0 / Ts, 1))
        self.main.status(f"Suggested Ts = {Ts*1000:g} ms ({1/Ts:g} Hz): at least 30 x the fastest loop "
                         f"({wmax:.2f} rad/s) and at most 5 deg phase-margin loss for every method.")

    def check_digital(self):
        if self._busy:
            return
        p = self.main.project
        mode = self.current_mode()
        e = p.ap.get(mode)
        if not e:
            QMessageBox.information(self, "Digital", "Design the autopilot first.")
            return
        Ts = 1.0 / self.fs.value()
        self._set_busy(True)
        cancel.reset()
        self._job = run_async(digital_job, self._dig_done, self._failed, self.main.status,
                              e, Ts, self.dmeth.currentData(), self.cdelay.isChecked())

    def _dig_done(self, e):
        self._set_busy(False)
        if e["mode"] == self.current_mode():
            self.show_entry(e)
            self.plots.setCurrentIndex(5)
        bad = [ap.METHOD_LABELS[m] for m, d in e["digital"].items() if not d["stable"]]
        self.main.status("Digital check done." + (f"  UNSTABLE: {', '.join(bad)}" if bad else ""))

    # ------------------------------------------------------------------
    def show_entry(self, e):
        plant, st = e["plant"], e["settings"]
        h, rows, eff0 = ap_summary_rows(e)
        colors = {}
        names = [m for m, r in e["results"].items() if r is not None]
        for i, r in enumerate(rows):
            res = e["results"][names[i]]
            for key, col in (("gm", 2), ("pm", 3), ("dm", 4)):
                colors[(i, col)] = "ok" if res.feasible[key] else "bad"
            for j in range(len(plant.input_names)):
                ok = res.peak_effort[j] <= st.limits[j] * plant.input_scale[j] * 1.005
                colors[(i, eff0 + j)] = "ok" if ok else "bad"
            if "bank" in res.feasible:
                colors[(i, len(r) - 2)] = "ok" if res.feasible["bank"] else "bad"
            colors[(i, len(r) - 1)] = "ok" if r[-1] else "bad"
        self.t_sum.fill(h, rows, colors)
        h, rows = ap_extra_rows(e)
        self.t_ext.fill(h, rows)
        if e.get("digital"):
            h, rows = digital_rows(e)
            col = {(i, 4): "ok" if r[4] else "bad" for i, r in enumerate(rows)}
            self.t_dig.fill(h, rows, col)
        else:
            self.t_dig.fill([], [])
        txt = []
        for m, c in e["ctrls"].items():
            _, gr = ctrl_gain_rows(c)
            txt.append(f"{ap.METHOD_LABELS[m]}:  " + "   ".join(f"{k} = {fmt(v)}" for k, v in gr))
        self.gains.setPlainText("\n".join(txt))
        self.diag.show_items(diagnose.autopilot(e))
        prow = []
        for m, c in e["ctrls"].items():
            for k, v in c.params.items():
                if isinstance(v, (list, tuple, np.ndarray)):
                    v = "[" + " ".join(f"{float(x):.6g}" for x in np.ravel(v)) + "]"
                elif isinstance(v, (float, np.floating)):
                    v = f"{float(v):.6g}"
                prow.append([m, k, v])
        self.par.fill(["method", "parameter", "value"], prow)
        for r in range(self.par.rowCount()):
            for cidx in (0, 1):
                it = self.par.item(r, cidx)
                it.setFlags(it.flags() & ~Qt.ItemIsEditable)
        title = ap.MODES[e["mode"]]["label"]
        self.plots.set("Response", plots.ap_response(e["results"], plant, st, title))
        self.plots.set("Control effort", plots.ap_effort(e["results"], plant, st))
        self.plots.set("Loop (Bode)", plots.ap_bode(e["results"], None))
        if e.get("noise"):
            self.plots.set("Noise", plots.ap_noise(e["noise"], plant, st))
        if e.get("sat"):
            self.plots.set("Saturation", plots.ap_saturation(e["results"], e["sat"], plant, st))
        if e.get("digital"):
            self.plots.set("Digital", plots.digital_plot(e["results"], e["digital"], plant, st))

    def _read_params(self):
        out = {}
        for r in range(self.par.rowCount()):
            m = self.par.item(r, 0).text()
            k = self.par.item(r, 1).text()
            v = self.par.item(r, 2).text().strip()
            if v.startswith("["):
                val = parse_matrix(v, name=f"{m}.{k}").ravel().tolist()
            else:
                try:
                    val = float(v)
                except ValueError:
                    raise ValueError(f"{m}: parameter {k} = '{v}' is not a number") from None
            out.setdefault(m, {})[k] = val
        return out

    def figures(self):
        return self.plots.figures()

    # --- project file support ---------------------------------------------------
    def state(self):
        return {"mode": self.current_mode(), "ref": self.ref.value(), "ref2": self.ref2.value(),
                "x0": self.x0.value(), "T": self.T.value(), "dt": self.dt.value(), "pf": self.pf.value(),
                "bank": self.bank.value(), "gm": self.gm.value(), "pm": self.pm.value(), "dm": self.dm.value(),
                "nstd": self.nstd.value(), "nbw": self.nbw.value(), "sat": self.sat.isChecked(),
                "aw": self.aw.isChecked(), "fs": self.fs.value(), "dmeth": self.dmeth.currentIndex(),
                "cdelay": self.cdelay.isChecked(), "auto_fs": self.auto_fs.isChecked(),
                "methods": [m for m, c in self.meth.items() if c.isChecked()],
                "limits": [[self.lim_table.item(i, c).text() if self.lim_table.item(i, c) else "inf"
                            for c in (1, 2)] for i in range(self.lim_table.rowCount())]}

    def set_state(self, s):
        i = self.mode.findData(s.get("mode", "theta"))
        self.mode.setCurrentIndex(max(i, 0))
        self.mode_changed()
        for w, k in ((self.ref, "ref"), (self.ref2, "ref2"), (self.x0, "x0"), (self.T, "T"), (self.dt, "dt"),
                     (self.pf, "pf"), (self.bank, "bank"), (self.gm, "gm"), (self.pm, "pm"), (self.dm, "dm"),
                     (self.nstd, "nstd"), (self.nbw, "nbw"), (self.fs, "fs")):
            if k in s:
                w.setValue(s[k])
        self.sat.setChecked(s.get("sat", True)); self.aw.setChecked(s.get("aw", True))
        self.cdelay.setChecked(s.get("cdelay", True)); self.dmeth.setCurrentIndex(s.get("dmeth", 0))
        self.auto_fs.setChecked(s.get("auto_fs", True))
        if "methods" in s:
            for m, c in self.meth.items():
                c.setChecked(m in s["methods"] and c.isEnabled())
        for i, (a, b) in enumerate(s.get("limits", [])):
            if i < self.lim_table.rowCount():
                self.lim_table.item(i, 1).setText(str(a)); self.lim_table.item(i, 2).setText(str(b))


def tune_job(plant, st, methods, mode, params, progress=None, digital_cfg=None):
    """digital_cfg: None (no digital check) or {"auto": bool, "Ts": s, "method": str, "delay": bool};
    with auto=True the sample time is suggested from the designed loops."""
    entry = _tune_core(plant, st, methods, mode, params, progress)
    if digital_cfg and entry["ctrls"] and not entry["cancelled"]:
        Ts = digital_cfg["Ts"]
        if digital_cfg.get("auto"):
            Ts = digital.suggest_ts(plant, list(entry["ctrls"].values()), method=digital_cfg["method"],
                                    delay=digital_cfg["delay"])[0]
        try:
            digital_job(entry, Ts, digital_cfg["method"], digital_cfg["delay"], progress)
        except cancel.Cancelled:
            entry["cancelled"] = True
        except Exception as e:  # noqa: BLE001
            entry["errors"]["digital check"] = str(e)
    return entry


def _tune_core(plant, st, methods, mode, params, progress=None):
    ctrls, results, noise, sat, errors = {}, {}, {}, {}, {}
    lqi = None
    cancelled = False
    order = [m for m in ap.METHODS if m in methods]
    for m in order:
        if progress:
            progress(f"{ap.METHOD_LABELS[m]}: {'auto-tuning' if params is None else 'rebuilding'} ...")
        try:
            if params is None:
                c = ap.tune(m, plant, st, lqi)
            else:
                c = ap.build_from_params(m, plant, params[m], st)
            if m == "LQI":
                lqi = c
            r = ap.evaluate(plant, c, st)
            nz = sim.run_noise(plant, c, st) if np.any(np.asarray(st.noise_std) > 0) else None
            s = None
            if st.use_saturation:
                s = sim.run_saturated(plant, c, st, antiwindup=st.antiwindup)
                if st.antiwindup and c.int_states:
                    s["noaw"] = sim.run_saturated(plant, c, st, antiwindup=False)
            ctrls[m], results[m] = c, r
            if nz is not None:
                noise[m] = nz
            if s is not None:
                sat[m] = s
        except cancel.Cancelled:
            cancelled = True
            break
        except Exception as e:  # noqa: BLE001
            errors[m] = str(e)
    return {"mode": mode, "plant": plant, "settings": st, "ctrls": ctrls, "results": results,
            "noise": noise or None, "sat": sat or None, "errors": errors, "cancelled": cancelled,
            "digital": None}


def digital_job(entry, Ts, method, delay, progress=None):
    out = {}
    for m, c in entry["ctrls"].items():
        cancel.check()
        if progress:
            progress(f"Digital check: {ap.METHOD_LABELS[m]} ...")
        out[m] = digital.digital_check(entry["plant"], c, entry["settings"], Ts, method, delay)
    entry["digital"] = out
    return entry
