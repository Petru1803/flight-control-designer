"""1-D gain scheduling on Mach: SAS (and optionally LQI autopilot) gains are designed at
each flight point with the same automatic rules, then interpolated linearly."""
from dataclasses import dataclass, field

import numpy as np

from . import autopilot as ap
from . import cancel, sas
from .models import LatModel, LongModel


@dataclass
class FlightPoint:
    mach: float
    V0: float
    A_long: np.ndarray
    B_long: np.ndarray
    A_lat: np.ndarray = None
    B_lat: np.ndarray = None
    label: str = ""


@dataclass
class ScheduleResult:
    mach: np.ndarray
    Klong: np.ndarray            # (N, m_long, 4)
    Klat: np.ndarray             # (N, m_lat, 4) or None
    Kap: np.ndarray              # (N, n_ap+1) LQI gains or None
    long_names: list
    lat_names: list
    ap_names: list
    long_inputs: list
    lat_inputs: list
    table: list = field(default_factory=list)   # per-point summary dicts
    warnings: list = field(default_factory=list)

    def interp(self, mach, which="long"):
        K = {"long": self.Klong, "lat": self.Klat, "ap": self.Kap}[which]
        if K is None:
            return None
        order = np.argsort(self.mach)
        M = self.mach[order]
        K = K[order]
        flat = K.reshape(len(M), -1)
        out = np.array([np.interp(mach, M, flat[:, i]) if M[0] <= mach <= M[-1]
                        else _extrap(mach, M, flat[:, i]) for i in range(flat.shape[1])])
        return out.reshape(K.shape[1:])


def _extrap(x, xp, fp):
    if len(xp) == 1:
        return fp[0]
    if x < xp[0]:
        return fp[0] + (x - xp[0]) * (fp[1] - fp[0]) / (xp[1] - xp[0])
    return fp[-1] + (x - xp[-1]) * (fp[-1] - fp[-2]) / (xp[-1] - xp[-2])


def run_schedule(points, convention, cat, cls, long_inputs, lat_inputs, g=9.81,
                 ap_mode=None, ap_settings=None, theta0=0.0, progress=None):
    if not points:
        raise ValueError("Add at least one flight point.")
    pts = sorted(points, key=lambda p: p.mach)
    Kl, Kla, Kap, table, warns = [], [], [], [], []
    have_lat = all(p.A_lat is not None for p in pts)
    long_names = lat_names = ap_names = None
    for i, p in enumerate(pts):
        cancel.check()
        if progress:
            progress(f"Flight point {i+1}/{len(pts)}  (Mach {p.mach:.3f})")
        lm = LongModel(p.A_long, p.B_long, convention, p.V0, g, long_inputs)
        tl = sas.auto_long_targets(lm, cat, cls)
        rl = sas.design_long_sas(lm, tl, cat, cls)
        Kl.append(rl.K)
        long_names = lm.state_names
        row = {"Mach": p.mach, "V0": p.V0,
               "wn_sp target": tl.wn_sp, "SP level OL": rl.levels_ol["sp"], "SP level CL": rl.levels_cl["sp"],
               "Ph level CL": rl.levels_cl["ph"], "long stable": bool(np.all(rl.poles.real < 0))}
        latm = rlat = None
        if have_lat:
            latm = LatModel(p.A_lat, p.B_lat, p.V0, theta0, g, lat_inputs)
            tt = sas.auto_lat_targets(latm, cat, cls)
            rlat = sas.design_lat_sas(latm, tt, cat, cls)
            Kla.append(rlat.K)
            lat_names = latm.state_names
            row.update({"DR level OL": rlat.levels_ol["dr"], "DR level CL": rlat.levels_cl["dr"],
                        "Roll level CL": rlat.levels_cl["roll"],
                        "lat stable": bool(np.all(rlat.poles.real < 0))})
        if ap_mode:
            try:
                plant = ap.build_plant(ap_mode, lm, rl, latm, rlat)
                st = ap_settings or ap.APSettings(ref=ap.MODES[ap_mode]["ref"], T=ap.MODES[ap_mode]["T"])
                c = ap.tune_lqi(plant, st)
                Kap.append(c.info["K"].ravel())
                ap_names = [f"K_{s}" for s in plant.state_names] + ["K_i"]
                r = ap.evaluate(plant, c, st)
                row.update({"AP PM [deg]": r.margins["pm"], "AP GM [dB]": r.margins["gm_db"],
                            "AP peak effort": float(r.peak_effort[plant.j])})
            except cancel.Cancelled:
                raise
            except Exception as e:  # noqa: BLE001
                warns.append(f"Mach {p.mach}: autopilot LQI failed: {e}")
                Kap.append(np.full(len(Kap[0]) if Kap else 6, np.nan))
        table.append(row)
        for w in rl.warnings + (rlat.warnings if rlat else []):
            warns.append(f"Mach {p.mach}: {w}")
    return ScheduleResult(np.array([p.mach for p in pts]), np.array(Kl),
                          np.array(Kla) if have_lat else None,
                          np.array(Kap) if ap_mode and Kap else None,
                          long_names, lat_names, ap_names, list(long_inputs), list(lat_inputs),
                          table, warns)


def check_interpolated(result, points, convention, g=9.81):
    """Stability of the scheduled (interpolated) SAS at the design points themselves,
    and of each point's gains applied to its neighbours (robustness between nodes)."""
    pts = sorted(points, key=lambda p: p.mach)
    out = []
    for i, p in enumerate(pts):
        lm = LongModel(p.A_long, p.B_long, convention, p.V0, g)
        for jn in (i - 1, i + 1):
            if 0 <= jn < len(pts):
                K = result.Klong[jn]
                ok = bool(np.all(np.linalg.eigvals(lm.Ad - lm.Bd[:, :K.shape[0]] @ K).real < 0))
                out.append({"plant Mach": p.mach, "gains from Mach": pts[jn].mach, "long stable": ok})
    return out
