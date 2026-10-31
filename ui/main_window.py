"""Main window: tabs, menus (project files), status bar and the shared project state."""
import os

from PySide6.QtCore import QSettings, Qt
from PySide6.QtGui import QAction, QKeySequence
from PySide6.QtWidgets import (QApplication, QDockWidget, QFileDialog, QLabel, QMainWindow, QMessageBox,
                               QProgressBar, QTabWidget, QTextBrowser)

from core import cancel
from ui import help as help_text
from ui import persist, theme
from ui.project import Project
from ui.tab_autopilot import AutopilotTab, digital_job, tune_job
from ui.tab_export import ExportTab
from ui.tab_hq import HQTab
from ui.tab_model import ModelTab
from ui.tab_robust import RobustTab
from ui.tab_sas import SASTab
from ui.tab_schedule import ScheduleTab, _sched_job
from ui.widgets import run_async
from version import __version__

ABOUT = f"""<b>SAS &amp; Autopilot Designer</b> &nbsp; version {__version__}<br><br>
Stability augmentation by pole placement inside the MIL-F-8785C Level 1 region,<br>
C* and q/q<sub>ss</sub>, MIL-STD-1797A CAP, bandwidth/phase delay, Gibson and roll performance,<br>
autopilots (theta, altitude, speed, roll, heading, turn coordination and the coupled<br>
h+V and psi+beta modes) designed with PIDF, LQI, LQG, H-infinity (hinfsyn),<br>
H-infinity loop shaping and a heading cascade; actuator dynamics, yaw-damper washout,<br>
noise, saturation with anti-windup, digital implementation and 1-D gain scheduling.<br><br>
Matrices are entered in MATLAB syntax. Longitudinal states [u w q theta]<br>
(converted to [u nz q theta]) or [V alpha q theta]; lateral [beta p r phi]."""

FILTER = "SAS Designer project (*.sasproj);;JSON (*.json)"


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.resize(1500, 920)
        self.project = Project()
        self.path = None
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
        self.model_tab = ModelTab(self)
        self.sas_tab = SASTab(self)
        self.hq_tab = HQTab(self)
        self.ap_tab = AutopilotTab(self)
        self.robust_tab = RobustTab(self)
        self.sched_tab = ScheduleTab(self)
        self.export_tab = ExportTab(self)
        self.tabs.addTab(self.model_tab, "1. Model")
        self.tabs.addTab(self.sas_tab, "2. SAS")
        self.tabs.addTab(self.hq_tab, "3. Handling qualities")
        self.tabs.addTab(self.ap_tab, "4. Autopilot")
        self.tabs.addTab(self.robust_tab, "5. Robustness")
        self.tabs.addTab(self.sched_tab, "6. Gain schedule")
        self.tabs.addTab(self.export_tab, "7. Export")
        self.msg = QLabel("Enter the matrices in the Model tab and press 'Parse & analyse', or open a project.")
        self.bar = QProgressBar(); self.bar.setMaximumWidth(160); self.bar.setRange(0, 0); self.bar.hide()
        self.statusBar().addWidget(self.msg, 1)
        self.statusBar().addPermanentWidget(self.bar)
        m = self.menuBar().addMenu("&File")
        for text, key, fn in (("&New project", QKeySequence.New, self.new_project),
                              ("&Open project...", QKeySequence.Open, self.open_project),
                              ("&Save project", QKeySequence.Save, self.save_project),
                              ("Save project &as...", QKeySequence.SaveAs, self.save_project_as)):
            a = QAction(text, self); a.setShortcut(key); a.triggered.connect(fn); m.addAction(a)
        self.recent_menu = m.addMenu("Recent projects")
        m.addSeparator()
        a = QAction("&Quit", self); a.setShortcut(QKeySequence.Quit); a.triggered.connect(self.close)
        m.addAction(a)
        h = self.menuBar().addMenu("&Help")
        a = QAction("&User guide", self); a.setShortcut(QKeySequence.HelpContents); a.triggered.connect(self.show_help)
        h.addAction(a)
        a = QAction("&About", self); a.triggered.connect(lambda: QMessageBox.about(self, "About", ABOUT))
        h.addAction(a)
        self.help_dock = QDockWidget("User guide", self)
        self.help_dock.setObjectName("help")
        self.help_view = QTextBrowser()
        self.help_view.setFont(theme.mono(10))
        self.help_view.setHtml(help_text.html())
        self.help_dock.setWidget(self.help_view)
        self.help_dock.setMinimumWidth(460)
        self.addDockWidget(Qt.RightDockWidgetArea, self.help_dock)
        self.help_dock.hide()
        self._update_title()
        self._update_recent()
        theme.capitals(self)

    def show_help(self):
        idx = self.tabs.currentIndex()
        anchor = help_text.TAB_ANCHORS[idx] if idx < len(help_text.TAB_ANCHORS) else "model"
        self.help_dock.show()
        self.help_dock.raise_()
        self.help_view.scrollToAnchor(anchor)

    # ------------------------------------------------------------------
    def status(self, text):
        self.msg.setText(text)

    def set_busy(self, busy):
        self.bar.setVisible(busy)
        if busy:
            QApplication.setOverrideCursor(Qt.BusyCursor)
        else:
            QApplication.restoreOverrideCursor()

    def model_changed(self):
        self.sas_tab.refresh_inputs()
        self.hq_tab.refresh_inputs()
        self.ap_tab.mode_changed()

    def sas_changed(self):
        self.ap_tab.mode_changed()

    def _update_title(self):
        name = os.path.basename(self.path) if self.path else "untitled"
        self.setWindowTitle(f"SAS & Autopilot Designer {__version__} - {name}")

    # --- project files ----------------------------------------------------------
    def _settings(self):
        return QSettings("SASDesigner", "SASDesigner")

    def _update_recent(self):
        self.recent_menu.clear()
        rec = self._settings().value("recent", []) or []
        if isinstance(rec, str):
            rec = [rec]
        for pth in rec[:8]:
            a = QAction(pth, self)
            a.triggered.connect(lambda _=False, x=pth: self.open_project(x))
            self.recent_menu.addAction(a)
        self.recent_menu.setEnabled(bool(rec))

    def _add_recent(self, path):
        s = self._settings()
        rec = s.value("recent", []) or []
        if isinstance(rec, str):
            rec = [rec]
        rec = [path] + [r for r in rec if r != path]
        s.setValue("recent", rec[:8])
        self._update_recent()

    def new_project(self):
        if QMessageBox.question(self, "New project", "Discard the current project?") != QMessageBox.Yes:
            return
        self.close_children()
        w = MainWindow()
        w.show()
        self._replacement = w
        self.close()

    def close_children(self):
        cancel.request()

    def save_project(self):
        if not self.path:
            return self.save_project_as()
        try:
            persist.save(self, self.path)
        except Exception as e:  # noqa: BLE001
            QMessageBox.critical(self, "Save", f"Could not save: {e}")
            return
        self._add_recent(self.path)
        self._update_title()
        self.status(f"Saved {self.path}")

    def save_project_as(self):
        start = self.path or os.path.join(os.path.expanduser("~"), "Documents",
                                          (self.project.name or "project") + ".sasproj")
        path, _ = QFileDialog.getSaveFileName(self, "Save project", start, FILTER)
        if path:
            self.path = path
            self.save_project()

    def open_project(self, path=None):
        if not path:
            start = os.path.dirname(self.path) if self.path else os.path.join(os.path.expanduser("~"), "Documents")
            path, _ = QFileDialog.getOpenFileName(self, "Open project", start, FILTER)
            if not path:
                return
        try:
            data = persist.load_file(path)
        except Exception as e:  # noqa: BLE001
            QMessageBox.critical(self, "Open", f"Could not open {path}:\n{e}")
            return
        self.project.__init__()
        self.path = path
        self._update_title()
        self._add_recent(path)
        try:
            jobs = persist.apply_sync(self, data)
        except Exception as e:  # noqa: BLE001
            QMessageBox.warning(self, "Open", f"The project was only partly restored:\n{e}")
            return
        if not jobs or (not jobs["ap"] and not jobs["schedule"]):
            self.status(f"Opened {path}")
            return
        sched_args = self.sched_tab.job_args() if jobs["schedule"] and self.project.schedule_points else None
        self.set_busy(True)
        cancel.reset()
        self._job = run_async(_restore_job, self._restored, self._restore_failed, self.status,
                              jobs["ap"], sched_args)

    def _restored(self, out):
        self.set_busy(False)
        entries, sched = out
        for e in entries:
            if e["ctrls"]:
                self.project.ap[e["mode"]] = e
        self.ap_tab.mode_changed()
        if sched is not None:
            self.sched_tab._done(sched)
        self.status(f"Opened {self.path} (results recomputed).")

    def _restore_failed(self, msg):
        self.set_busy(False)
        QMessageBox.warning(self, "Open", "Recomputing the saved results failed:\n" + msg[:2000])


def _restore_job(ap_jobs, sched_args, progress=None):
    entries = []
    for plant, st, params, mode, dig in ap_jobs:
        e = tune_job(plant, st, list(params), mode, params, progress)
        if dig and e["ctrls"]:
            digital_job(e, dig["Ts"], dig["method"], dig["delay"], progress)
        entries.append(e)
    sched = _sched_job(*sched_args, progress=progress) if sched_args else None
    return entries, sched
