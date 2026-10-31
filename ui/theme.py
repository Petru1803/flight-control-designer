"""Military cockpit (MFD / HUD) look: black background, phosphor-green monospace text,
amber cautions, red warnings. Also recolours matplotlib figures for on-screen display."""
from matplotlib.colors import to_rgb
from PySide6.QtGui import QColor, QFont, QFontDatabase, QPalette
from PySide6.QtWidgets import QApplication

BG = "#000000"
PANEL = "#020a02"
FIELD = "#041204"
GREEN = "#33ff33"        # primary symbology
GREEN_DIM = "#1d9e1d"
GREEN_DARK = "#0d3d0d"
BORDER = "#1f6f1f"
AMBER = "#ffbf00"        # caution
RED = "#ff4040"          # warning
CYAN = "#00e5ff"
WHITE = "#d8ffd8"

# status colours for table cells: key -> (background, foreground)
STATUS = {
    "ok": ("#0b3d0b", "#5cff5c"),
    "warn": ("#3d3000", AMBER),
    "bad": ("#4a0b0b", "#ff6060"),
    "worse": ("#5c0000", "#ff3030"),
}

MONO = ["Consolas", "Cascadia Mono", "Lucida Console", "Courier New"]
TITLE = ["OCR A Extended", "Consolas", "Cascadia Mono"]

_font_cache = {}


def family(cands):
    key = tuple(cands)
    if key not in _font_cache:
        have = set(QFontDatabase.families())
        _font_cache[key] = next((c for c in cands if c in have), "Monospace")
    return _font_cache[key]


def mono(size=10, bold=False, upper=False):
    f = QFont(family(MONO), size)
    f.setBold(bold)
    f.setStyleHint(QFont.Monospace)
    if upper:
        f.setCapitalization(QFont.AllUppercase)
        f.setLetterSpacing(QFont.PercentageSpacing, 104)
    return f


def title_font(size=11):
    f = QFont(family(TITLE), size)
    f.setCapitalization(QFont.AllUppercase)
    f.setLetterSpacing(QFont.PercentageSpacing, 108)
    return f


QSS = f"""
* {{ color: {GREEN}; selection-background-color: {GREEN_DARK}; selection-color: {WHITE}; }}
QMainWindow, QWidget {{ background: {BG}; }}
QScrollArea, QScrollArea > QWidget > QWidget {{ background: {BG}; border: none; }}
QGroupBox {{ border: 1px solid {BORDER}; border-radius: 0px; margin-top: 14px; padding: 6px 4px 4px 4px; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 8px; padding: 0 4px; color: {GREEN}; }}
QLabel {{ background: transparent; }}
QLineEdit, QPlainTextEdit, QTextEdit, QTextBrowser, QDoubleSpinBox, QSpinBox, QComboBox {{
    background: {FIELD}; border: 1px solid {BORDER}; border-radius: 0px; padding: 2px 4px; color: {GREEN}; }}
QLineEdit:focus, QPlainTextEdit:focus, QDoubleSpinBox:focus, QComboBox:focus {{ border: 1px solid {GREEN}; }}
QComboBox QAbstractItemView {{ background: {FIELD}; border: 1px solid {GREEN}; color: {GREEN};
    selection-background-color: {GREEN_DARK}; }}
QComboBox::drop-down {{ border-left: 1px solid {BORDER}; width: 16px; }}
QComboBox::down-arrow {{ image: url(@DOWN@); width: 9px; height: 6px; }}
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{ background: {PANEL}; border-left: 1px solid {BORDER}; width: 14px; }}
QDoubleSpinBox::up-arrow {{ image: url(@UP@); width: 7px; height: 5px; }}
QDoubleSpinBox::down-arrow {{ image: url(@DOWN@); width: 7px; height: 5px; }}
QCheckBox::indicator:checked {{ image: url(@CHECK@); }}
QPushButton {{ background: {PANEL}; border: 1px solid {GREEN_DIM}; border-radius: 0px; padding: 5px 10px; color: {GREEN}; }}
QPushButton:hover {{ background: {GREEN_DARK}; border: 1px solid {GREEN}; }}
QPushButton:pressed {{ background: {GREEN}; color: {BG}; }}
QPushButton:disabled {{ color: #145c14; border: 1px solid #0f3f0f; }}
QPushButton:default {{ border: 2px solid {GREEN}; }}
QCheckBox::indicator {{ width: 12px; height: 12px; border: 1px solid {GREEN_DIM}; background: {BG}; }}
QCheckBox::indicator:checked {{ background: {GREEN}; border: 1px solid {GREEN}; }}
QCheckBox:disabled {{ color: #145c14; }}
QTabWidget::pane {{ border: 1px solid {BORDER}; top: -1px; }}
QTabBar::tab {{ background: {BG}; border: 1px solid {BORDER}; padding: 5px 14px; margin-right: 2px; color: {GREEN_DIM}; }}
QTabBar::tab:selected {{ background: {GREEN_DARK}; color: {GREEN}; border: 1px solid {GREEN}; border-bottom: none; }}
QTabBar::tab:hover {{ color: {GREEN}; }}
QTableWidget, QTableView {{ background: {BG}; alternate-background-color: {PANEL}; gridline-color: {GREEN_DARK};
    border: 1px solid {BORDER}; color: {GREEN}; }}
QHeaderView::section {{ background: {GREEN_DARK}; color: {GREEN}; border: 1px solid {BORDER}; padding: 3px; }}
QTableCornerButton::section {{ background: {GREEN_DARK}; border: 1px solid {BORDER}; }}
QScrollBar:vertical {{ background: {BG}; width: 12px; border: 1px solid {GREEN_DARK}; }}
QScrollBar:horizontal {{ background: {BG}; height: 12px; border: 1px solid {GREEN_DARK}; }}
QScrollBar::handle {{ background: {GREEN_DARK}; border: 1px solid {GREEN_DIM}; min-height: 20px; min-width: 20px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; }}
QScrollBar::add-page, QScrollBar::sub-page {{ background: none; }}
QMenuBar {{ background: {BG}; border-bottom: 1px solid {BORDER}; }}
QMenuBar::item:selected {{ background: {GREEN_DARK}; }}
QMenu {{ background: {BG}; border: 1px solid {GREEN}; }}
QMenu::item:selected {{ background: {GREEN_DARK}; }}
QStatusBar {{ background: {BG}; border-top: 1px solid {BORDER}; color: {GREEN}; }}
QProgressBar {{ background: {BG}; border: 1px solid {GREEN_DIM}; text-align: center; }}
QProgressBar::chunk {{ background: {GREEN}; }}
QSplitter::handle {{ background: {GREEN_DARK}; }}
QToolTip {{ background: {BG}; color: {GREEN}; border: 1px solid {GREEN}; padding: 4px; }}
QDockWidget {{ color: {GREEN}; }}
QDockWidget::title {{ background: {GREEN_DARK}; padding: 4px; }}
QMessageBox, QDialog {{ background: {BG}; }}
QToolBar {{ background: {BG}; border: none; }}
QToolButton {{ background: {BG}; border: 1px solid transparent; }}
QToolButton:hover {{ border: 1px solid {GREEN_DIM}; background: {GREEN_DARK}; }}
"""


def apply(app: QApplication):
    app.setStyle("Fusion")
    pal = QPalette()
    for role, col in ((QPalette.Window, BG), (QPalette.WindowText, GREEN), (QPalette.Base, FIELD),
                      (QPalette.AlternateBase, PANEL), (QPalette.Text, GREEN), (QPalette.Button, PANEL),
                      (QPalette.ButtonText, GREEN), (QPalette.Highlight, GREEN_DARK),
                      (QPalette.HighlightedText, WHITE), (QPalette.ToolTipBase, BG),
                      (QPalette.ToolTipText, GREEN), (QPalette.PlaceholderText, GREEN_DIM),
                      (QPalette.BrightText, AMBER), (QPalette.Link, CYAN)):
        pal.setColor(role, QColor(col))
    app.setPalette(pal)
    app.setFont(mono(10))
    icons = _make_icons()
    qss = QSS
    for k, v in icons.items():
        qss = qss.replace(f"@{k}@", v)
    app.setStyleSheet(qss)


def _make_icons():
    """Small green arrow / check glyphs (QSS cannot draw triangles reliably)."""
    import os
    import tempfile
    from PySide6.QtCore import QPointF, Qt
    from PySide6.QtGui import QPainter, QPen, QPixmap, QPolygonF
    d = os.path.join(tempfile.gettempdir(), "sasdesigner_theme")
    os.makedirs(d, exist_ok=True)
    out = {}
    shapes = {"UP": [(0, 10), (7, 2), (14, 10)], "DOWN": [(0, 2), (7, 10), (14, 2)]}
    for name, pts in shapes.items():
        pm = QPixmap(14, 12); pm.fill(Qt.transparent)
        p = QPainter(pm); p.setRenderHint(QPainter.Antialiasing)
        p.setPen(Qt.NoPen); p.setBrush(QColor(GREEN))
        p.drawPolygon(QPolygonF([QPointF(x, y) for x, y in pts])); p.end()
        path = os.path.join(d, f"{name.lower()}.png"); pm.save(path)
        out[name] = path.replace("\\", "/")
    pm = QPixmap(12, 12); pm.fill(QColor(GREEN))
    p = QPainter(pm); p.setRenderHint(QPainter.Antialiasing)
    p.setPen(QPen(QColor(BG), 2)); p.drawLine(2, 6, 5, 9); p.drawLine(5, 9, 10, 3); p.end()
    path = os.path.join(d, "check.png"); pm.save(path)
    out["CHECK"] = path.replace("\\", "/")
    return out


UPPER_CLASSES = ("QPushButton", "QTabBar", "QGroupBox", "QHeaderView", "QMenuBar", "QDockWidget")


def capitals(root):
    """Cockpit-style capitals on buttons, tabs, group titles and table headers. Labels keep
    their case on purpose: Greek symbols (omega, zeta, tau) would change meaning in capitals."""
    from PySide6.QtWidgets import QWidget
    widgets = [root] + root.findChildren(QWidget)
    for w in widgets:
        if w.metaObject().className() in UPPER_CLASSES or any(w.inherits(c) for c in UPPER_CLASSES):
            f = w.font()
            if f.capitalization() != QFont.AllUppercase:
                f.setCapitalization(QFont.AllUppercase)
                f.setLetterSpacing(QFont.PercentageSpacing, 104)
                w.setFont(f)


# ---------------------------------------------------------------------------------
# matplotlib: recolour a (light) figure for the HUD display
# ---------------------------------------------------------------------------------
_PALETTE = {
    "#d62728": AMBER, "#1f77b4": CYAN, "#2ca02c": "#39ff14", "#9467bd": "#ff4dff",
    "#ff7f0e": "#ff8c00", "#8c564b": WHITE,
}
_NAMED = {(0, 0, 1): CYAN, (0, 0.5, 0): "#39ff14", (1, 0, 0): RED, (1, 1, 0): AMBER,
          (0.75, 0.75, 0): AMBER, (0.75, 0, 0.75): "#ff4dff", (1, 0, 1): "#ff4dff"}


def _hex(rgb):
    return "#%02x%02x%02x" % tuple(int(round(255 * c)) for c in rgb[:3])


def hud_color(c, fill=False):
    try:
        rgb = to_rgb(c)
    except ValueError:
        return c
    h = _hex(rgb)
    if h in _PALETTE:
        return _PALETTE[h]
    key = tuple(round(x, 2) for x in rgb)
    if key in _NAMED:
        return _NAMED[key]
    lum = 0.299 * rgb[0] + 0.587 * rgb[1] + 0.114 * rgb[2]
    grey = max(rgb) - min(rgb) < 0.08
    if fill and lum > 0.75:
        # light fills (Level regions etc.) -> dark tinted panels
        r, g, b = rgb
        if g > r + 0.03 and g > b:
            return STATUS["ok"][0]
        if r > b + 0.1:
            return STATUS["warn"][0]
        return GREEN_DARK
    if grey:
        if lum < 0.2:
            return WHITE              # black reference lines -> bright
        return GREEN_DIM              # greys -> dim symbology
    return c


def hudify(fig):
    """Recolour a figure in place for the black/green display (figures are built light,
    so exports stay printable)."""
    fig.set_facecolor(BG)
    if fig._suptitle is not None:
        fig._suptitle.set_color(GREEN)
    for ax in fig.axes:
        ax.set_facecolor(BG)
        for sp in ax.spines.values():
            sp.set_color(GREEN_DIM)
        ax.tick_params(colors=GREEN, which="both")
        ax.xaxis.label.set_color(GREEN)
        ax.yaxis.label.set_color(GREEN)
        ax.title.set_color(GREEN)
        for gl in ax.get_xgridlines() + ax.get_ygridlines():
            gl.set_color(GREEN_DARK)
            gl.set_alpha(1.0)
        for ln in ax.lines:
            ln.set_color(hud_color(ln.get_color()))
            mfc, mec = ln.get_markerfacecolor(), ln.get_markeredgecolor()
            if mfc not in ("none", None):
                ln.set_markerfacecolor(hud_color(mfc))
            if mec not in ("none", None):
                ln.set_markeredgecolor(hud_color(mec))
        for p in ax.patches:
            p.set_facecolor(hud_color(p.get_facecolor(), fill=True))
            p.set_edgecolor(hud_color(p.get_edgecolor()))
        for coll in ax.collections:
            try:
                coll.set_facecolor([hud_color(c, fill=True) for c in coll.get_facecolor()])
                coll.set_edgecolor([hud_color(c) for c in coll.get_edgecolor()])
            except Exception:  # noqa: BLE001
                pass
        for t in ax.texts:
            t.set_color(GREEN)
        leg = ax.get_legend()
        if leg is not None:
            loc = getattr(leg, "_loc", "best")
            size = leg.get_texts()[0].get_fontsize() if leg.get_texts() else 8
            ncol = getattr(leg, "_ncols", 1)
            new = ax.legend(loc=loc, fontsize=size, ncol=ncol)
            new.get_frame().set_facecolor(BG)
            new.get_frame().set_edgecolor(GREEN_DIM)
            for t in new.get_texts():
                t.set_color(GREEN)
    for t in fig.texts:
        t.set_color(GREEN)
    return fig
