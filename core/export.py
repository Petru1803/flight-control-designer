"""Export: PDF report (matplotlib PdfPages), PNG figures, Excel workbook, CSV tables."""
import csv
import datetime
import os
import re

import numpy as np
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.figure import Figure


def _fmt(v):
    if isinstance(v, (bool, np.bool_)):
        return "yes" if v else "no"
    if isinstance(v, (int, np.integer)):
        return str(int(v))
    if isinstance(v, (float, np.floating)):
        if not np.isfinite(v):
            return "inf" if v > 0 else ("-inf" if v < 0 else "n/a")
        return f"{v:.5g}"
    return str(v)


def _safe(name):
    return re.sub(r"[^A-Za-z0-9_\-]+", "_", name).strip("_")[:60] or "item"


def _table_page(pdf, title, header, rows, subtitle=None):
    per_page = 28
    chunks = [rows[i:i + per_page] for i in range(0, max(len(rows), 1), per_page)] or [[]]
    for ci, chunk in enumerate(chunks):
        f = Figure(figsize=(8.27, 11.69))
        ax = f.add_axes([0.05, 0.05, 0.9, 0.85])
        ax.axis("off")
        f.text(0.05, 0.95, title + (f" ({ci+1}/{len(chunks)})" if len(chunks) > 1 else ""),
               fontsize=13, weight="bold")
        if subtitle:
            f.text(0.05, 0.925, subtitle, fontsize=8.5, color="0.3")
        if chunk:
            cells = [[_fmt(v) for v in r] for r in chunk]
            tb = ax.table(cellText=cells, colLabels=[str(h) for h in header], loc="upper center",
                          cellLoc="center")
            tb.auto_set_font_size(False)
            tb.set_fontsize(7.5 if len(header) > 6 else 8.5)
            tb.scale(1, 1.35)
            for (r, c), cell in tb.get_celld().items():
                if r == 0:
                    cell.set_facecolor("#dfe7f2")
                    cell.set_text_props(weight="bold")
        pdf.savefig(f)


def export_pdf(path, title, tables, figures, notes=None):
    """tables: list of (title, header, rows); figures: list of (title, Figure)."""
    with PdfPages(path) as pdf:
        f = Figure(figsize=(8.27, 11.69))
        f.text(0.08, 0.85, title, fontsize=20, weight="bold")
        f.text(0.08, 0.81, "SAS / Autopilot design report", fontsize=13)
        f.text(0.08, 0.78, datetime.datetime.now().strftime("%Y-%m-%d %H:%M"), fontsize=10, color="0.4")
        y = 0.72
        for line in (notes or []):
            f.text(0.08, y, line, fontsize=9.5, wrap=True)
            y -= 0.025
            if y < 0.08:
                break
        pdf.savefig(f)
        for t, h, rows in tables:
            _table_page(pdf, t, h, rows)
        for t, fig in figures:
            pdf.savefig(fig)
        d = pdf.infodict()
        d["Title"] = title
        d["Creator"] = "SAS & Autopilot Designer"


def export_png(folder, figures, dpi=150):
    os.makedirs(folder, exist_ok=True)
    paths = []
    for i, (t, fig) in enumerate(figures, 1):
        p = os.path.join(folder, f"Fig_{i:02d}_{_safe(t)}.png")
        fig.savefig(p, dpi=dpi)
        paths.append(p)
    return paths


def export_excel(path, tables):
    from openpyxl import Workbook
    from openpyxl.styles import Font, PatternFill
    wb = Workbook()
    wb.remove(wb.active)
    used = set()
    for t, h, rows in tables:
        name = _safe(t)[:31] or "Sheet"
        base, k = name, 1
        while name in used:
            k += 1
            name = f"{base[:28]}_{k}"
        used.add(name)
        ws = wb.create_sheet(name)
        ws.append([str(x) for x in h])
        for c in ws[1]:
            c.font = Font(bold=True)
            c.fill = PatternFill("solid", fgColor="DFE7F2")
        for r in rows:
            ws.append([_cell(v) for v in r])
        for col in ws.columns:
            width = max(len(str(c.value)) if c.value is not None else 0 for c in col)
            ws.column_dimensions[col[0].column_letter].width = min(max(10, width + 2), 45)
    wb.save(path)


def _cell(v):
    if isinstance(v, (bool, np.bool_)):
        return bool(v)
    if isinstance(v, (np.integer,)):
        return int(v)
    if isinstance(v, (float, np.floating)):
        return float(v) if np.isfinite(v) else str(v)
    return v if isinstance(v, (int, float, str)) else str(v)


def export_csv(folder, tables):
    os.makedirs(folder, exist_ok=True)
    paths = []
    for t, h, rows in tables:
        p = os.path.join(folder, _safe(t) + ".csv")
        with open(p, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(h)
            for r in rows:
                w.writerow([_cell(v) for v in r])
        paths.append(p)
    return paths
