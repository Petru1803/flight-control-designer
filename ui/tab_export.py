"""Export tab: PDF report, PNG figures, Excel workbook, CSV tables."""
import os

from PySide6.QtCore import QUrl
from PySide6.QtGui import QDesktopServices
from PySide6.QtWidgets import (QCheckBox, QFileDialog, QFormLayout, QGroupBox, QHBoxLayout, QLineEdit,
                               QMessageBox, QPlainTextEdit, QPushButton, QVBoxLayout, QWidget)

from core import autopilot as ap
from core import export, matlab_export, plots, sas
from ui import theme
from ui.project import project_tables
from ui.widgets import mono_font


def project_figures(p):
    figs = []
    if p.long_sas is not None:
        m, r = p.long_model, p.long_sas
        j = r.targets.input_index
        figs.append(("Longitudinal poles", plots.pz_long(m, r, p.cat, p.cls)))
        P, K = sas.sp_gain_domain(m, p.cat, p.cls, j)
        figs.append(("Short-period gain domain", plots.sp_domain(m, r, p.cat, p.cls, P, K, j)))
        figs.append(("C star and q-q_ss", plots.cstar_qdiag(m, r, j)[0]))
        figs.append(("Longitudinal SAS response", plots.sas_response_long(m, r, j)))
    if p.lat_sas is not None:
        figs.append(("Lateral poles", plots.pz_lat(p.lat_model, p.lat_sas, p.cat, p.cls)))
        figs.append(("Lateral SAS response", plots.sas_response_lat(p.lat_model, p.lat_sas)))
    hqr = p.hq1797 or {}
    if "cap" in hqr:
        figs.append(("CAP - zeta", plots.hq_cap(hqr["cap"], p.cat)))
    if "bw" in hqr:
        figs.append(("Pitch bandwidth and phase delay", plots.hq_bandwidth(hqr["bw"], hqr["bw_limits"])))
    if "gibson" in hqr:
        figs.append(("Gibson dropback", plots.hq_gibson(hqr["gibson"])))
    if "roll" in hqr:
        figs.append(("Roll performance", plots.hq_roll(hqr["roll"])))
    for mode, e in p.ap.items():
        lbl = ap.MODES[mode]["label"]
        pl, st = e["plant"], e["settings"]
        figs.append((f"AP {mode} response", plots.ap_response(e["results"], pl, st, lbl)))
        figs.append((f"AP {mode} effort", plots.ap_effort(e["results"], pl, st)))
        figs.append((f"AP {mode} loop", plots.ap_bode(e["results"], None)))
        if e.get("noise"):
            figs.append((f"AP {mode} noise", plots.ap_noise(e["noise"], pl, st)))
        if e.get("sat"):
            figs.append((f"AP {mode} saturation", plots.ap_saturation(e["results"], e["sat"], pl, st)))
        if e.get("digital"):
            figs.append((f"AP {mode} digital", plots.digital_plot(e["results"], e["digital"], pl, st)))
    if p.robust:
        res = p.robust
        if "long" in res["sas"]:
            figs.append(("Robustness longitudinal SAS", plots.robust_poles(
                res["sas"]["long"], p.long_sas.poles, "Longitudinal SAS", region=p.long_sas.sp_limits)))
        if "lat" in res["sas"]:
            from core import hq
            figs.append(("Robustness lateral SAS", plots.robust_poles(
                res["sas"]["lat"], p.lat_sas.poles, "Lateral SAS", lat_limits=hq.lat_level1_targets(p.cat, p.cls))))
        for m in sorted({m for (m, _) in res["ap"]}):
            if m in p.ap:
                st = p.ap[m]["settings"]
                figs.append((f"Robustness {m} margins", plots.robust_margins(res["ap"], m,
                                                                            (st.gm_spec, st.pm_spec, st.dm_spec))))
                figs.append((f"Robustness {m} spread", plots.robust_spread(res["ap"], m, p.ap[m]["plant"], st)))
    if p.schedule is not None:
        figs.append(("Gain schedule", plots.schedule_plot(p.schedule)))
    return figs


class ExportTab(QWidget):
    def __init__(self, main):
        super().__init__()
        self.main = main
        lv = QVBoxLayout(self)
        g = QGroupBox("Export")
        f = QFormLayout(g)
        self.folder = QLineEdit(os.path.join(os.path.expanduser("~"), "Documents", "SAS_designer_export"))
        hb = QHBoxLayout(); hb.addWidget(self.folder)
        b = QPushButton("Browse..."); b.clicked.connect(self.browse); hb.addWidget(b)
        f.addRow("Output folder", hb)
        self.title = QLineEdit("")
        self.title.setPlaceholderText("(aircraft name from the Model tab)")
        f.addRow("Report title", self.title)
        self.c_pdf = QCheckBox("PDF report (tables + all figures)"); self.c_pdf.setChecked(True)
        self.c_png = QCheckBox("PNG figures"); self.c_png.setChecked(True)
        self.c_xls = QCheckBox("Excel workbook (.xlsx)"); self.c_xls.setChecked(True)
        self.c_csv = QCheckBox("CSV tables"); self.c_csv.setChecked(False)
        self.c_mat = QCheckBox("MATLAB / Simulink (.mat + loader / Simulink-builder .m)"); self.c_mat.setChecked(True)
        self.c_hud = QCheckBox("Figures in HUD style (black / green) instead of white"); self.c_hud.setChecked(False)
        for c in (self.c_pdf, self.c_png, self.c_xls, self.c_csv, self.c_mat, self.c_hud):
            f.addRow(c)
        hb = QHBoxLayout()
        b = QPushButton("Export"); b.clicked.connect(self.do_export); hb.addWidget(b)
        b = QPushButton("Open folder"); b.clicked.connect(self.open_folder); hb.addWidget(b)
        f.addRow(hb)
        lv.addWidget(g)
        self.log = QPlainTextEdit(); self.log.setReadOnly(True); self.log.setFont(mono_font(9))
        lv.addWidget(self.log, 1)

    def browse(self):
        d = QFileDialog.getExistingDirectory(self, "Output folder", self.folder.text())
        if d:
            self.folder.setText(d)

    def open_folder(self):
        if os.path.isdir(self.folder.text()):
            QDesktopServices.openUrl(QUrl.fromLocalFile(self.folder.text()))

    def do_export(self):
        p = self.main.project
        if p.long_sas is None and p.lat_sas is None and p.long_model is None and p.lat_model is None:
            QMessageBox.information(self, "Export", "Nothing to export yet - parse a model first.")
            return
        folder = self.folder.text().strip()
        try:
            os.makedirs(folder, exist_ok=True)
        except OSError as e:
            QMessageBox.warning(self, "Export", str(e)); return
        title = self.title.text().strip() or p.name
        base = export._safe(title)
        self.main.status("Exporting ...")
        tables = project_tables(p)
        figs = project_figures(p) if (self.c_pdf.isChecked() or self.c_png.isChecked()) else []
        if self.c_hud.isChecked():
            figs = [(t, theme.hudify(f)) for t, f in figs]
        out = []
        try:
            if self.c_pdf.isChecked():
                path = os.path.join(folder, f"{base}_report.pdf")
                notes = [f"Aircraft class {p.cls}, flight-phase category {p.cat} (MIL-F-8785C).",
                         "Gains follow u = -K x (SAS) and the common outer-loop form of each autopilot.",
                         "Margins: loop broken at the autopilot command (plant input).",
                         "Efforts are total surface commands (autopilot + SAS)."]
                export.export_pdf(path, title, tables, figs, notes)
                out.append(path)
            if self.c_png.isChecked():
                out += export.export_png(os.path.join(folder, f"{base}_figures"), figs)
            if self.c_xls.isChecked():
                path = os.path.join(folder, f"{base}_results.xlsx")
                export.export_excel(path, tables)
                out.append(path)
            if self.c_csv.isChecked():
                out += export.export_csv(os.path.join(folder, f"{base}_csv"), tables)
            if self.c_mat.isChecked():
                out += matlab_export.export(p, folder, base)
        except Exception as e:  # noqa: BLE001
            QMessageBox.critical(self, "Export", f"Export failed: {e}")
            self.main.status("Export failed.")
            return
        self.log.appendPlainText("Exported:\n  " + "\n  ".join(out) + "\n")
        self.main.status(f"Exported {len(out)} file(s) to {folder}")
