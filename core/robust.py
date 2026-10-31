"""Robustness analysis.

Monte Carlo on the aerodynamic derivatives: every stability / control derivative entry of A
and B is scaled by (1 + d), d uniform in [-x, +x], independently. Kinematic and gravity
terms stay exact. The SAS gains and the autopilot controllers are kept at their nominal
values (as in flight), and the perturbed aircraft is re-evaluated.

Disk margins (balanced, skew 0) of every autopilot loop are computed on the nominal model,
both loop-at-a-time and for simultaneous variations at all autopilot inputs."""
from dataclasses import dataclass, field

import numpy as np

from . import autopilot as ap
from . import cancel, hq, lti
from .models import LatModel, LongModel, lat_modes, long_modes
from .sas import lat_extended


def _long_mask(conv):
    """Entries of the physical longitudinal A (4x4) that are aerodynamic derivatives."""
    M = np.zeros((4, 4), bool)
    M[:3, :3] = True
    M[1, 2] = False          # w-eq: U0 + Zq (uwqt) / alpha-eq: ~1 (Vaqt) - kinematic
    return M


def _lat_mask():
    M = np.zeros((4, 4), bool)
    M[1:3, :3] = True        # roll and yaw moment derivatives
    M[0, 0] = True           # Y_beta / V
    return M


def _scale(M, mask, frac, rng):
    d = rng.uniform(-frac, frac, M.shape)
    out = M.copy()
    out[mask] = M[mask] * (1 + d[mask])
    return out


def perturb_long(model, frac, rng):
    Bmask = np.zeros(model.B.shape, bool); Bmask[:3, :] = True
    A = _scale(model.A, _long_mask(model.convention), frac, rng)
    B = _scale(model.B, Bmask, frac, rng)
    return LongModel(A, B, model.convention, model.V0, model.g, list(model.input_names))


def perturb_lat(model, frac, rng):
    Bmask = np.zeros(model.B.shape, bool); Bmask[:3, :] = True
    A = _scale(model.A, _lat_mask(), frac, rng)
    B = _scale(model.B, Bmask, frac, rng)
    return LatModel(A, B, model.V0, model.theta0, model.g, list(model.input_names))


@dataclass
class SASRobust:
    axis: str
    n: int
    stable: float = 0.0              # fraction of samples
    level1: float = 0.0
    min_zeta: list = field(default_factory=list)
    poles: list = field(default_factory=list)   # closed-loop poles of every sample
    worst_real: float = -np.inf


@dataclass
class APRobust:
    mode: str
    method: str
    n: int
    stable: float = 0.0
    meets: float = 0.0
    pm: list = field(default_factory=list)
    gm: list = field(default_factory=list)
    dm: list = field(default_factory=list)
    t: np.ndarray = None
    ys: list = field(default_factory=list)      # tracked output of the first samples
    sse: list = field(default_factory=list)
    peak: list = field(default_factory=list)


def _min_zeta(poles):
    z = [-p.real / abs(p) for p in poles if abs(p) > 1e-9 and abs(p.imag) > 1e-9]
    return min(z) if z else 1.0


def run(project, frac=0.2, n=100, seed=1, do_long=True, do_lat=True, modes=(), keep_curves=40,
        progress=None):
    rng = np.random.default_rng(seed)
    p = project
    out = {"frac": frac, "n": n, "seed": seed, "sas": {}, "ap": {}, "disk": {}}
    long_s = SASRobust("long", n) if do_long and p.long_sas is not None else None
    lat_s = SASRobust("lat", n) if do_lat and p.lat_sas is not None else None
    entries = {m: p.ap[m] for m in modes if m in p.ap}
    apr = {(m, meth): APRobust(m, meth, n) for m, e in entries.items() for meth in e["ctrls"]}
    # nominal disk margins
    for m, e in entries.items():
        for meth, c in e["ctrls"].items():
            out["disk"][(m, meth)] = disk_margins(e["plant"], c)
    for k in range(n):
        cancel.check()
        if progress and k % 5 == 0:
            progress(f"Monte Carlo sample {k + 1}/{n} ...")
        lm = perturb_long(p.long_model, frac, rng) if p.long_model is not None else None
        latm = perturb_lat(p.lat_model, frac, rng) if p.lat_model is not None else None
        if long_s is not None:
            K = p.long_sas.K
            Acl = lm.Ad - lm.Bd @ K
            ev = np.linalg.eigvals(Acl)
            st = bool(np.all(ev.real < 0))
            lv = hq.evaluate_long(long_modes(Acl), p.cat, p.cls, lm.n_alpha())
            long_s.stable += st; long_s.level1 += st and all(v == 1 for v in lv.values())
            long_s.poles.extend(ev.tolist()); long_s.min_zeta.append(_min_zeta(ev))
            long_s.worst_real = max(long_s.worst_real, float(ev.real.max()))
        if lat_s is not None:
            Ae, Be, Ke, _, _ = lat_extended(latm, p.lat_sas)
            ev = np.linalg.eigvals(Ae - Be @ Ke)
            st = bool(np.all(ev.real < 0))
            ev4 = ev
            if p.lat_sas.targets.washout_tau > 0:
                real = [i for i, x in enumerate(ev) if abs(x.imag) < 1e-6]
                if real:
                    tw = p.lat_sas.targets.washout_tau
                    iw = min(real, key=lambda i: abs(ev[i].real + 1 / tw))
                    ev4 = np.delete(ev, iw)
            lv = hq.evaluate_lat(lat_modes(np.diag(ev4)), p.cat, p.cls)
            lat_s.stable += st; lat_s.level1 += st and all(v == 1 for v in lv.values())
            lat_s.poles.extend(ev.tolist()); lat_s.min_zeta.append(_min_zeta(ev))
            lat_s.worst_real = max(lat_s.worst_real, float(ev.real.max()))
        for m, e in entries.items():
            pl0 = e["plant"]
            try:
                pl = ap.build_plant(m, lm, p.long_sas, latm, p.lat_sas,
                                    pl0.j if pl0.q == 1 else None, p.actuator)
            except ap.APError:
                continue
            st0 = e["settings"]
            for meth, c in e["ctrls"].items():
                r = apr[(m, meth)]
                res = ap.evaluate(pl, c, st0, with_margins=True, quick=True)
                r.stable += res.stable
                ok = res.stable and all(res.feasible.values())
                r.meets += ok
                mg = res.margins
                r.pm.append(mg["pm"] if res.stable else np.nan)
                r.gm.append(abs(mg["gm_db"]) if res.stable else np.nan)
                r.dm.append(mg["dm"] if res.stable else np.nan)
                r.sse.append(float(np.max(np.abs(st0.ref - res.y[-1]))) if res.stable else np.nan)
                r.peak.append(res.peak_effort)
                if len(r.ys) < keep_curves:
                    r.t = res.t
                    r.ys.append(res.y if res.stable else np.full_like(res.y, np.nan))
    for s in (long_s, lat_s):
        if s is not None:
            s.stable /= n; s.level1 /= n
            out["sas"][s.axis] = s
    for key, r in apr.items():
        r.stable /= n; r.meets /= n
        out["ap"][key] = r
    return out


def disk_margins(plant, ctrl, npts=1500):
    """Loop-at-a-time and multi-loop balanced disk margins at the autopilot inputs."""
    AL, BL, CL = ap.loop_matrices(plant, ctrl)
    w = np.logspace(-4, 4, npts)
    L = lti.freqresp_mimo(AL, BL, CL, w)
    q = plant.q
    out = {"loop": []}
    for i in range(q):
        o = [x for x in range(q) if x != i]
        Ai = AL - BL[:, o] @ CL[o, :] if o else AL
        Li = lti.freqresp_mimo(Ai, BL[:, [i]], CL[[i], :], w)
        out["loop"].append(lti.disk_margin(Li))
    out["multi"] = lti.disk_margin(L)
    return out


def summary_rows(res, project):
    """Tables: SAS robustness, autopilot robustness, nominal disk margins."""
    sas_rows = []
    for ax, s in res["sas"].items():
        sas_rows.append(["longitudinal SAS" if ax == "long" else "lateral SAS", 100 * s.stable, 100 * s.level1,
                         float(np.min(s.min_zeta)) if s.min_zeta else np.nan, s.worst_real])
    ap_rows = []
    for (m, meth), r in res["ap"].items():
        e = project.ap[m]
        pm, dm, gm = np.array(r.pm, float), np.array(r.dm, float), np.array(r.gm, float)
        ap_rows.append([ap.MODES[m]["label"], ap.METHOD_LABELS.get(meth, meth), 100 * r.stable, 100 * r.meets,
                        np.nanmin(pm) if np.isfinite(pm).any() else np.nan,
                        np.nanmin(gm) if np.isfinite(gm).any() else np.nan,
                        np.nanmin(dm) if np.isfinite(dm).any() else np.nan,
                        np.nanmax(r.sse) if np.isfinite(r.sse).any() else np.nan])
    disk_rows = []
    for (m, meth), d in res["disk"].items():
        names = [project.ap[m]["plant"].input_names[j] for j in project.ap[m]["plant"].js]
        loops = "; ".join(f"{nm}: {dg:.1f} dB / {dp:.1f} deg" for nm, (a, dg, dp) in zip(names, d["loop"]))
        a, dg, dp = d["multi"]
        disk_rows.append([ap.MODES[m]["label"], ap.METHOD_LABELS.get(meth, meth), loops, dg, dp, a])
    return ({"head": ["system", "stable [%]", "all modes Level 1 [%]", "min damping ratio", "worst Re(pole)"],
             "rows": sas_rows},
            {"head": ["mode", "method", "stable [%]", "meets all specs [%]", "min PM [deg]", "min |GM| [dB]",
                      "min DM [s]", "max final error"], "rows": ap_rows},
            {"head": ["mode", "method", "loop-at-a-time disk margin (GM / PM)", "multi-loop disk GM [dB]",
                      "multi-loop disk PM [deg]", "alpha"], "rows": disk_rows})
