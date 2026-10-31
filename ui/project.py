"""Application state shared by all tabs, plus table builders used by the UI and export."""
import numpy as np

from core import autopilot as ap
from core import hq
from core.models import lat_modes, long_modes, pbh_report

R2D = 180 / np.pi


class Project:
    def __init__(self):
        self.name = "Aircraft"
        self.cat = "A"
        self.cls = "IV"
        self.consts = {}
        self.flight = {}
        self.long_model = None
        self.lat_model = None
        self.long_sas = None
        self.lat_sas = None
        self.cstar_checks = None
        self.actuator = ap.ActuatorSpec()
        self.hq1797 = None         # dict with results + table rows
        self.robust = None         # Monte Carlo / disk-margin results
        self.aero = None           # aerodynamic data (dict) of the coefficient dialog
        self.ap = {}               # mode -> dict(plant, settings, ctrls, results, noise, sat, digital)
        self.schedule_points = []
        self.schedule = None

    def invalidate_sas(self):
        self.long_sas = self.lat_sas = None
        self.hq1797 = None
        self.robust = None
        self.ap.clear()


# --------------------------------------------------------------------------------
def _poles_txt(m):
    ps = m.poles
    if len(ps) == 2 and abs(ps[0].imag) > 1e-9:
        return f"{ps[0].real:.4f} ± {abs(ps[0].imag):.4f}j"
    return ", ".join(f"{p.real:.4f}" for p in ps)


def _mode_row(m, lv):
    t2 = m.t2 if np.isfinite(m.t2) else np.nan
    tau = m.tau if (m.kind == "real" and np.isfinite(m.tau) and m.tau > 0) else np.nan
    return [m.name, _poles_txt(m), m.wn, m.zeta, tau, t2, hq.level_text(lv)]


MODE_HEADER = ["Mode", "Poles", "wn [rad/s]", "zeta", "tau [s]", "T2 [s]", "MIL-F-8785C"]


def long_mode_rows(A, model, cat, cls):
    m = long_modes(A)
    lv = hq.evaluate_long(m, cat, cls, model.n_alpha())
    return [_mode_row(m["sp"], lv["sp"]), _mode_row(m["ph"], lv["ph"])], lv


def lat_mode_rows(A, cat, cls):
    m = lat_modes(A)
    lv = hq.evaluate_lat(m, cat, cls)
    rows = [_mode_row(m["dr"], lv["dr"]), _mode_row(m["roll"], lv["roll"])]
    if "spiral" in m:
        rows.append(_mode_row(m["spiral"], lv["spiral"]))
    return rows, lv


def level_colors(rows, col=6):
    from ui.widgets import LEVEL_COLORS
    out = {}
    for i, r in enumerate(rows):
        txt = r[col]
        lv = 1 if txt == "Level 1" else 2 if txt == "Level 2" else 3 if txt == "Level 3" else 4
        out[(i, col)] = LEVEL_COLORS[lv]
    return out


def ctrb_text(model, axis):
    lines = []
    for j, nm in enumerate(model.input_names):
        rep = pbh_report(model.Ad, model.Bd[:, [j]])
        st = "controllable" if rep["controllable"] else (
            "stabilizable (not fully controllable)" if rep["stabilizable"] else "NOT stabilizable")
        lines.append(f"  {axis} input '{nm}': {st}")
    rep = pbh_report(model.Ad, model.Bd)
    lines.append(f"  {axis} all inputs: " + ("controllable" if rep["controllable"] else
                                             "stabilizable" if rep["stabilizable"] else "NOT stabilizable"))
    return lines


def sas_gain_rows(res, model):
    rows = []
    for i, nm in enumerate(model.input_names):
        if np.allclose(res.K[i], 0):
            continue
        rows.append([nm] + list(res.K[i]))
    return ["input \\ state"] + [f"K_{s}" for s in model.state_names], rows


def ap_summary_rows(entry):
    """Returns header, rows and the column indices used for colouring."""
    plant, st = entry["plant"], entry["settings"]
    header = ["Method", "stable", "GM [dB]", "PM [deg]", "DM [s]"]
    for tn, un in zip(plant.track_names, plant.track_units):
        sfx = f" {tn}" if plant.p > 1 else ""
        header += [f"rise{sfx} [s]", f"settle 2%{sfx} [s]", f"overshoot{sfx} [%]", f"final error{sfx} [{un}]"]
    eff0 = len(header)
    header += [f"peak {n} [{'deg' if plant.input_scale[i] != 1 else 'u'}]" for i, n in enumerate(plant.input_names)]
    has_bank = any("bank" in r.feasible for r in entry["results"].values() if r is not None)
    if has_bank:
        header.append("peak bank [deg]")
    header.append("meets specs")
    rows = []
    for m, r in entry["results"].items():
        if r is None:
            continue
        row = [ap.METHOD_LABELS.get(m, m), r.stable, r.margins["gm_db"], r.margins["pm"], r.margins["dm"]]
        for i in range(plant.p):
            mt = r.metrics[i] if i < len(r.metrics) else {}
            row += [mt.get("rise", np.nan), mt.get("settle", np.nan), mt.get("overshoot", np.nan),
                    mt.get("sse", np.nan)]
        row += list(r.peak_effort)
        if has_bank:
            row.append(r.peak_bank)
        row.append(all(r.feasible.values()))
        rows.append(row)
    return header, rows, eff0


def ap_extra_rows(entry):
    header = ["Method", "noise RMS effect", "noise max effect", "saturated: final value",
              "saturated: final, no anti-windup", "saturation reached", "diverged", "integrator frozen [steps]"]
    rows = []
    for m in entry["results"]:
        nz = (entry.get("noise") or {}).get(m)
        sat = (entry.get("sat") or {}).get(m)
        fin = float(sat["y"][-1, 0]) if sat and np.isfinite(sat["y"][-1, 0]) else np.nan
        naw = sat.get("noaw") if sat else None
        fin2 = float(naw["y"][-1, 0]) if naw and np.isfinite(naw["y"][-1, 0]) else np.nan
        rows.append([ap.METHOD_LABELS.get(m, m),
                     nz["rms_effect"] if nz else np.nan, nz["max_effect"] if nz else np.nan,
                     fin, fin2, sat["saturated"] if sat else "-", sat["diverged"] if sat else "-",
                     sat.get("frozen_steps", 0) if sat else "-"])
    return header, rows


def _txt(v):
    if isinstance(v, (list, tuple, np.ndarray)):
        return "[" + " ".join(f"{float(x):.6g}" for x in np.ravel(v)) + "]"
    return v


def ctrl_gain_rows(ctrl):
    rows = []
    for k, v in ctrl.gains.items():
        rows.append([k, v])
    for k, v in ctrl.params.items():
        rows.append([f"param {k}", _txt(v)])
    Nb = np.atleast_2d(np.asarray(ctrl.Nbar, float))
    if np.max(np.abs(Nb - np.eye(Nb.shape[0]))) > 1e-9:
        rows.append(["Nbar (DC correction)", _txt(Nb) if Nb.size > 1 else float(Nb[0, 0])])
    rows.append(["controller states", ctrl.nc])
    return ["quantity", "value"], rows


def digital_rows(entry):
    dg = entry.get("digital") or {}
    header = ["Method", "Ts [ms]", "discretisation", "1-sample delay", "stable", "spectral radius",
              "GM [dB]", "PM [deg]", "DM [s]", "max deviation from continuous [%]"]
    rows = []
    for m, d in dg.items():
        rows.append([ap.METHOD_LABELS.get(m, m), d["Ts"] * 1000, d["method"], d["delay"], d["stable"], d["rho"],
                     d["margins"]["gm_db"], d["margins"]["pm"], d["margins"]["dm"], d["max_dev_pct"]])
    return header, rows


def project_tables(p):
    """All tables for export: list of (title, header, rows)."""
    T = []
    fl = [["Aircraft", p.name], ["Class", p.cls], ["Flight-phase category", p.cat]]
    fl += [[k, v] for k, v in p.flight.items()]
    fl += [["actuator time constant [s]", p.actuator.tau], ["actuator time delay [s]", p.actuator.delay]]
    T.append(("Flight condition", ["item", "value"], fl))
    if p.long_model is not None:
        rows, _ = long_mode_rows(p.long_model.Ad, p.long_model, p.cat, p.cls)
        T.append(("Longitudinal open-loop modes", MODE_HEADER, rows))
    if p.lat_model is not None:
        rows, _ = lat_mode_rows(p.lat_model.Ad, p.cat, p.cls)
        T.append(("Lateral open-loop modes", MODE_HEADER, rows))
    if p.long_sas is not None:
        h, r = sas_gain_rows(p.long_sas, p.long_model)
        T.append(("Longitudinal SAS gains", h, r))
        t = p.long_sas.targets
        T.append(("Longitudinal SAS targets", ["item", "value"],
                  [["wn_sp [rad/s]", t.wn_sp], ["zeta_sp", t.zeta_sp], ["wn_ph [rad/s]", t.wn_ph],
                   ["zeta_ph", t.zeta_ph], ["structure", t.structure],
                   ["n/alpha [g/rad]", p.long_model.n_alpha()],
                   ["Level 1 wn_sp range", f"{p.long_sas.sp_limits['wn'][0]:.3f} - {p.long_sas.sp_limits['wn'][1]:.3f}"],
                   ["Level 1 zeta_sp range", f"{p.long_sas.sp_limits['zeta'][0]} - {p.long_sas.sp_limits['zeta'][1]}"]]
                  + [[f"effort for 5 deg alpha: {nm} [deg]", e * R2D]
                     for nm, e in zip(p.long_model.input_names, p.long_sas.effort_alpha5)]
                  + ([["C* inside envelope [%]", 100 * p.cstar_checks["cstar"]],
                      ["q/q_ss inside envelope [%]", 100 * p.cstar_checks["qdiag"]]]
                     if p.cstar_checks and p.cstar_checks.get("cstar") is not None else [])))
        rows, _ = long_mode_rows(p.long_sas.Acl, p.long_model, p.cat, p.cls)
        T.append(("Longitudinal modes with SAS", MODE_HEADER, rows))
    if p.lat_sas is not None:
        h, r = sas_gain_rows(p.lat_sas, p.lat_model)
        T.append(("Lateral SAS gains", h, r))
        t = p.lat_sas.targets
        T.append(("Lateral SAS targets", ["item", "value"],
                  [["wn_dr [rad/s]", t.wn_dr], ["zeta_dr", t.zeta_dr], ["roll pole", t.roll_pole],
                   ["spiral pole", t.spiral_pole], ["yaw-rate washout tau [s]", t.washout_tau]]
                  + [[f"effort for 5 deg beta: {nm} [deg]", e * R2D]
                     for nm, e in zip(p.lat_model.input_names, p.lat_sas.effort_beta5)]))
        rows, _ = lat_mode_rows(p.lat_sas.Acl, p.cat, p.cls)
        T.append(("Lateral modes with SAS", MODE_HEADER, rows))
    if p.hq1797:
        T.append(("MIL-STD-1797A criteria", ["criterion", "value", "result"], p.hq1797["rows"]))
    for mode, e in p.ap.items():
        lbl = ap.MODES[mode]["label"]
        h, r, _ = ap_summary_rows(e)
        T.append((f"AP {lbl} - comparison", h, r))
        h, r = ap_extra_rows(e)
        T.append((f"AP {lbl} - noise and saturation", h, r))
        for m, c in e["ctrls"].items():
            h, r = ctrl_gain_rows(c)
            T.append((f"AP {mode} {m} gains", h, r))
        if e.get("digital"):
            h, r = digital_rows(e)
            T.append((f"AP {lbl} - digital implementation", h, r))
            for m, d in e["digital"].items():
                rows = []
                for nm in ("Ad", "Bd", "Cd", "Dd"):
                    M = np.atleast_2d(d[nm])
                    for i, row in enumerate(M):
                        rows.append([f"{nm}[{i+1},:]", _txt(row)])
                T.append((f"AP {mode} {m} discrete controller", ["matrix row", "values"], rows))
    if p.robust:
        from core import robust
        a, b, c = robust.summary_rows(p.robust, p)
        pct = f"+/-{100 * p.robust['frac']:.0f}%, {p.robust['n']} samples"
        T.append((f"Robustness SAS ({pct})", a["head"], a["rows"]))
        T.append((f"Robustness autopilot ({pct})", b["head"], b["rows"]))
        T.append(("Nominal disk margins", c["head"], c["rows"]))
    if p.schedule is not None:
        s = p.schedule
        if s.table:
            keys = list(s.table[0].keys())
            T.append(("Gain schedule - flight points", keys, [[row.get(k, "") for k in keys] for row in s.table]))
        h = ["Mach"] + [f"{s.long_inputs[i]}:K_{n}" for i in range(s.Klong.shape[1]) for n in s.long_names]
        T.append(("Gain schedule - longitudinal SAS", h,
                  [[s.mach[k]] + list(s.Klong[k].ravel()) for k in range(len(s.mach))]))
        if s.Klat is not None:
            h = ["Mach"] + [f"{s.lat_inputs[i]}:K_{n}" for i in range(s.Klat.shape[1]) for n in s.lat_names]
            T.append(("Gain schedule - lateral SAS", h,
                      [[s.mach[k]] + list(s.Klat[k].ravel()) for k in range(len(s.mach))]))
        if s.Kap is not None:
            T.append(("Gain schedule - autopilot LQI", ["Mach"] + (s.ap_names or []),
                      [[s.mach[k]] + list(s.Kap[k]) for k in range(len(s.mach))]))
    return T
