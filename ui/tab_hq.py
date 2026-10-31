"""Handling-qualities tab: MIL-STD-1797A / MIL-F-8785C additional criteria."""
import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QComboBox, QFormLayout, QGroupBox, QHBoxLayout, QLabel, QMessageBox,
                               QPushButton, QScrollArea, QSplitter, QVBoxLayout, QWidget)

from core import diagnose, hq, hq1797, plots
from ui.widgets import DataTable, DiagnosticsView, PlotTabs, heading, scrolled, spin
from ui.widgets import side_panel


class HQTab(QWidget):
    def __init__(self, main):
        super().__init__()
        self.main = main
        split = QSplitter(Qt.Horizontal)
        QHBoxLayout(self).addWidget(split)
        left = QWidget()
        lv = QVBoxLayout(left)
        g = QGroupBox("Pitch attitude bandwidth / phase delay boundaries (editable)")
        f = QFormLayout(g)
        self.bw1 = spin(6.5, 0, 50, 2, 0.5, "rad/s"); self.tp1 = spin(0.12, 0, 1, 3, 0.01, "s")
        self.bw2 = spin(4.5, 0, 50, 2, 0.5, "rad/s"); self.tp2 = spin(0.20, 0, 1, 3, 0.01, "s")
        f.addRow("Level 1: ωBW ≥", self.bw1); f.addRow("Level 1: τp ≤", self.tp1)
        f.addRow("Level 2: ωBW ≥", self.bw2); f.addRow("Level 2: τp ≤", self.tp2)
        b = QPushButton("Defaults for the selected category"); b.clicked.connect(self.defaults)
        f.addRow(b)
        f.addRow(QLabel("Commonly quoted MIL-STD-1797A values (Cat A fighter-type,\n"
                        "Cat C used also for B). Check them against your edition of the\n"
                        "standard and the aircraft class."))
        lv.addWidget(g)
        g = QGroupBox("Manoeuvres")
        f = QFormLayout(g)
        self.e_in = QComboBox()
        self.hold = spin(4.0, 0.5, 30, 1, 0.5, "s")
        self.a_in = QComboBox()
        self.da = spin(20.0, 0.1, 90, 1, 1, "deg")
        f.addRow("Pitch input", self.e_in)
        f.addRow("Gibson: command held for", self.hold)
        f.addRow("Roll input", self.a_in)
        f.addRow("Roll performance: step of", self.da)
        lv.addWidget(g)
        b = QPushButton("Evaluate handling qualities"); b.clicked.connect(self.evaluate)
        lv.addWidget(b)
        lv.addWidget(QLabel("All criteria use the SAS-augmented aircraft; bandwidth and roll\n"
                            "performance include the actuator model from the Model tab.\n"
                            "Gibson uses the short-period approximation, as in the criterion."))
        lv.addWidget(heading("Diagnostics"))
        self.diag = DiagnosticsView(220)
        lv.addWidget(self.diag)
        lv.addStretch(1)
        split.addWidget(side_panel(left))
        right = QWidget()
        rv = QVBoxLayout(right)
        rv.addWidget(heading("Results"))
        self.table = DataTable(max_height=330)
        rv.addWidget(self.table)
        self.plots = PlotTabs(["CAP - zeta", "Bandwidth", "Gibson dropback", "Roll performance"])
        rv.addWidget(self.plots, 1)
        split.addWidget(scrolled(right))
        split.setStretchFactor(1, 1)

    def refresh_inputs(self):
        p = self.main.project
        self.e_in.clear(); self.a_in.clear()
        if p.long_model is not None:
            self.e_in.addItems(p.long_model.input_names)
        if p.lat_model is not None:
            self.a_in.addItems(p.lat_model.input_names)
        self.defaults()

    def defaults(self):
        cat = self.main.project.cat
        d = hq1797.BW_DEFAULTS[cat]
        self.bw1.setValue(d[1][0]); self.tp1.setValue(d[1][1])
        self.bw2.setValue(d[2][0]); self.tp2.setValue(d[2][1])

    def evaluate(self):
        p = self.main.project
        if p.long_sas is None and p.lat_sas is None:
            QMessageBox.information(self, "Handling qualities", "Design the SAS first.")
            return
        rows, colors = [], {}
        out = {}

        def add(name, value, ok):
            i = len(rows)
            rows.append([name, value, ok if isinstance(ok, str) else ("PASS" if ok else "FAIL")])
            txt = rows[-1][2]
            if txt != "-":
                colors[(i, 2)] = "ok" if txt in ("PASS", "Level 1") else ("warn" if txt == "Level 2" else "bad")

        if p.long_sas is not None:
            m, r = p.long_model, p.long_sas
            j = max(self.e_in.currentIndex(), 0)
            cz = hq1797.cap_zeta(m, r, p.cat, p.cls)
            out["cap"] = cz
            add("CAP [1/(g s^2)]", cz["cap"], hq.level_text(cz["level"]))
            add("zeta_sp", cz["zeta"], hq.level_text(cz["level"]))
            self.plots.set("CAP - zeta", plots.hq_cap(cz, p.cat))
            lim = {1: (self.bw1.value(), self.tp1.value()), 2: (self.bw2.value(), self.tp2.value())}
            bw = hq1797.bandwidth(m, r, p.actuator, j)
            lv = hq1797.bandwidth_level(bw, lim)
            out["bw"] = bw
            add("theta bandwidth wBW [rad/s]", bw["wbw"], hq.level_text(lv))
            add("  wBW phase (-135 deg) [rad/s]", bw["wbw_phase"], "-")
            add("  wBW gain (+6 dB) [rad/s]", bw["wbw_gain"], "-")
            add("phase delay tau_p [s]", bw["tau_p"], hq.level_text(lv))
            add("gain-limited bandwidth (PIO tendency)", "yes" if bw["pio_prone"] else "no",
                "FAIL" if bw["pio_prone"] else "PASS")
            self.plots.set("Bandwidth", plots.hq_bandwidth(bw, lim))
            gb = hq1797.gibson(m, r, j, self.hold.value())
            out["gibson"] = gb
            add("Gibson dropback DB/q_ss [s]", gb["db_qss"], gb["ok"])
            add("Gibson q_pk/q_ss", gb["qpk_qss"], gb["ok"])
            self.plots.set("Gibson dropback", plots.hq_gibson(gb))
        if p.lat_sas is not None:
            ja = max(self.a_in.currentIndex(), 0)
            rp = hq1797.roll_performance(p.lat_model, p.lat_sas, p.actuator, p.cls, p.cat, ja, self.da.value())
            out["roll"] = rp
            add(f"time to {rp['req_bank']} deg bank [s] (Level 1: {rp['req_time']} s)", rp["t_reach"], rp["ok"])
            self.plots.set("Roll performance", plots.hq_roll(rp))
        self.table.fill(["criterion", "value", "result"], rows, colors)
        p.hq1797 = {"rows": rows, "bw_limits": {1: (self.bw1.value(), self.tp1.value()),
                                                2: (self.bw2.value(), self.tp2.value())}, **out}
        self.diag.show_items(diagnose.handling(p))
        self.main.status("Handling qualities evaluated.")

    def figures(self):
        return self.plots.figures()
