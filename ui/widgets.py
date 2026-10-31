"""Reusable Qt widgets: matrix editor, matplotlib canvas, tables, background worker."""
import traceback

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg, NavigationToolbar2QT
from PySide6.QtCore import QEvent, QObject, QRunnable, Qt, QThreadPool, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (QAbstractItemView, QDoubleSpinBox, QFrame, QHeaderView, QLabel,
                               QPlainTextEdit, QScrollArea, QSizePolicy, QTableWidget, QTableWidgetItem,
                               QTabWidget, QTextBrowser, QVBoxLayout, QWidget)

from core.plots import new_fig
from ui import theme


def mono_font(size=9):
    return theme.mono(size)


class MatrixEdit(QPlainTextEdit):
    def __init__(self, placeholder="", rows=5, parent=None):
        super().__init__(parent)
        self.setFont(mono_font(9))
        self.setPlaceholderText(placeholder)
        self.setLineWrapMode(QPlainTextEdit.NoWrap)
        h = self.fontMetrics().lineSpacing() * rows + 14
        self.setMinimumHeight(h)
        self.setMaximumHeight(h + 40)
        self.setTabChangesFocus(True)

    def mark(self, ok, tip=""):
        self.setStyleSheet("" if ok else f"QPlainTextEdit {{ border: 2px solid {theme.RED}; }}")
        self.setToolTip(tip)


class PlotCanvas(QWidget):
    """Matplotlib figure + navigation toolbar; replace the figure with set_figure()."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.lay = QVBoxLayout(self)
        self.lay.setContentsMargins(0, 0, 0, 0)
        self.canvas = None
        self.toolbar = None
        self.set_figure(new_fig())

    def set_figure(self, fig):
        if self.canvas is not None:
            for wdg in (self.toolbar, self.canvas):
                self.lay.removeWidget(wdg)
                wdg.hide()
                wdg.setParent(None)
                wdg.deleteLater()
        self.fig = theme.hudify(fig)
        self.canvas = FigureCanvasQTAgg(fig)
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.canvas.setMinimumHeight(340)
        self.toolbar = NavigationToolbar2QT(self.canvas, self)
        self.lay.addWidget(self.toolbar)
        self.lay.addWidget(self.canvas)
        self.canvas.draw_idle()


class PlotTabs(QTabWidget):
    def __init__(self, names, parent=None):
        super().__init__(parent)
        self.canvases = {}
        for n in names:
            c = PlotCanvas()
            self.canvases[n] = c
            self.addTab(c, n)

    def set(self, name, fig):
        self.canvases[name].set_figure(fig)

    def figures(self):
        return [(n, c.fig) for n, c in self.canvases.items() if c.fig.axes]


def fmt(v, prec=4):
    if isinstance(v, (bool, np.bool_)):
        return "yes" if v else "no"
    if isinstance(v, (int, np.integer)):
        return str(int(v))
    if isinstance(v, (float, np.floating)):
        if np.isnan(v):
            return "-"
        if np.isinf(v):
            return "inf" if v > 0 else "-inf"
        a = abs(v)
        if a != 0 and (a < 1e-3 or a >= 1e5):
            return f"{v:.{prec}e}"
        return f"{v:.{prec}g}" if a >= 1 else f"{v:.{prec}f}".rstrip("0").rstrip(".") or "0"
    if isinstance(v, complex):
        return f"{v.real:.4f}{v.imag:+.4f}j"
    return str(v)


LEVEL_COLORS = {1: "ok", 2: "warn", 3: "bad", 4: "worse"}


class DataTable(QTableWidget):
    def __init__(self, parent=None, editable=False, max_height=260):
        super().__init__(parent)
        self.max_rows_height = max_height
        self.setFixedHeight(50)
        self.setAlternatingRowColors(True)
        self.verticalHeader().setVisible(False)
        self.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.horizontalHeader().setStretchLastSection(True)
        if not editable:
            self.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.setSelectionBehavior(QAbstractItemView.SelectRows)

    def fill(self, header, rows, colors=None):
        """colors: optional dict {(row, col): status key ('ok', 'warn', 'bad', 'worse')}."""
        self.clear()
        self.setColumnCount(len(header))
        self.setRowCount(len(rows))
        self.setHorizontalHeaderLabels([str(h) for h in header])
        for i, r in enumerate(rows):
            for j, v in enumerate(r):
                it = QTableWidgetItem(fmt(v))
                it.setTextAlignment(Qt.AlignCenter)
                if colors and (i, j) in colors:
                    bg, fg = theme.STATUS.get(colors[(i, j)], (colors[(i, j)], theme.GREEN))
                    it.setBackground(QColor(bg))
                    it.setForeground(QColor(fg))
                self.setItem(i, j, it)
        self.resizeColumnsToContents()
        h = self.horizontalHeader().sizeHint().height() + sum(self.rowHeight(i) for i in range(len(rows)))
        h += self.horizontalScrollBar().sizeHint().height() + 8
        self.setFixedHeight(int(min(max(h, 50), self.max_rows_height)))


def spin(value=0.0, lo=-1e9, hi=1e9, dec=4, step=0.1, suffix=""):
    s = QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setDecimals(dec)
    s.setSingleStep(step)
    s.setValue(value)
    s.setKeyboardTracking(False)
    if suffix:
        s.setSuffix(" " + suffix)
    return s


def diag_html(items):
    """Render core.diagnose items: [CAUTION] title / explanation / > suggestion."""
    col = {"ok": theme.GREEN, "info": theme.CYAN, "caution": theme.AMBER, "warning": theme.RED}
    tag = {"ok": "OK", "info": "INFO", "caution": "CAUTION", "warning": "WARNING"}
    from html import escape
    rows = []
    for sev, title, expl, sug in items:
        title, expl, sug = escape(title), escape(expl), escape(sug)
        rows.append(f"<p style='margin:2px 0 6px 0'><span style='color:{col[sev]}'><b>[{tag[sev]}]</b> {title}</span>"
                    + (f"<br>&nbsp;&nbsp;{expl}" if expl else "")
                    + (f"<br>&nbsp;&nbsp;<span style='color:{theme.CYAN}'>&#9656; {sug}</span>" if sug else "")
                    + "</p>")
    return "".join(rows) or "<p>-</p>"


class DiagnosticsView(QTextBrowser):
    def __init__(self, height=170, parent=None):
        super().__init__(parent)
        self.setFont(theme.mono(9))
        self.setMinimumHeight(height)
        self.setOpenExternalLinks(False)
        self.setPlaceholderText("Diagnostics appear here after a design.")

    def show_items(self, items):
        self.setHtml(diag_html(items))


class SidePanel(QScrollArea):
    """Vertical-only scroll area for the input column. Its width follows the content's
    minimum width (re-fitted whenever the layout changes), and it can never scroll sideways:
    otherwise moving the focus to a widget near the right edge (e.g. Cancel when Auto-tune
    gets disabled) shifts the panel left with no way back."""

    def __init__(self, widget):
        super().__init__()
        self.setWidgetResizable(True)
        self.setFrameShape(QFrame.NoFrame)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setWidget(widget)
        bar = self.horizontalScrollBar()
        bar.valueChanged.connect(lambda v: v and bar.setValue(0))
        widget.installEventFilter(self)
        self.refit()

    def refit(self):
        w = self.widget().minimumSizeHint().width() + self.verticalScrollBar().sizeHint().width() + 6
        if w != self.minimumWidth():
            self.setMinimumWidth(w)

    def eventFilter(self, obj, ev):
        if ev.type() in (QEvent.LayoutRequest, QEvent.FontChange, QEvent.Show):
            self.refit()
        return False


def side_panel(widget):
    return SidePanel(widget)


def scrolled(widget):
    from PySide6.QtWidgets import QFrame, QScrollArea
    sc = QScrollArea()
    sc.setWidgetResizable(True)
    sc.setFrameShape(QFrame.NoFrame)
    sc.setWidget(widget)
    return sc


def heading(text):
    lb = QLabel("▸ " + text)
    lb.setFont(theme.title_font(11))
    lb.setStyleSheet(f"color: {theme.GREEN}; border-bottom: 1px solid {theme.BORDER}; padding-bottom: 2px;")
    return lb


# --- background worker ------------------------------------------------------------
class _Signals(QObject):
    done = Signal(object)
    failed = Signal(str)
    progress = Signal(str)


class Worker(QRunnable):
    def __init__(self, fn, *args, **kwargs):
        super().__init__()
        self.fn, self.args, self.kwargs = fn, args, kwargs
        self.signals = _Signals()

    def run(self):
        try:
            res = self.fn(*self.args, progress=self.signals.progress.emit, **self.kwargs)
        except Exception as e:  # noqa: BLE001
            self.signals.failed.emit(f"{e}\n\n{traceback.format_exc()}")
        else:
            self.signals.done.emit(res)


def run_async(fn, on_done, on_fail, on_progress=None, *args, **kwargs):
    w = Worker(fn, *args, **kwargs)
    w.signals.done.connect(on_done)
    w.signals.failed.connect(on_fail)
    if on_progress:
        w.signals.progress.connect(on_progress)
    QThreadPool.globalInstance().start(w)
    return w
