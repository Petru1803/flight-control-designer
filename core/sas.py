"""Stability augmentation (SAS) design by pole placement, with targets chosen
automatically inside the MIL-F-8785C Level 1 region (editable by the user)."""
from dataclasses import dataclass, field

import numpy as np

from . import hq
from .models import lat_modes, long_modes, second_order_poles


def acker(A, b, poles):
    """Ackermann's formula (SISO), tolerates repeated poles."""
    A = np.asarray(A, float)
    b = np.asarray(b, float).reshape(-1, 1)
    n = A.shape[0]
    Wc = np.hstack([np.linalg.matrix_power(A, k) @ b for k in range(n)])
    if np.linalg.matrix_rank(Wc) < n:
        raise np.linalg.LinAlgError("System not controllable from this input")
    coeffs = np.real(np.poly(poles))
    phi = sum(c * np.linalg.matrix_power(A, n - k) for k, c in enumerate(coeffs))
    en = np.zeros((1, n))
    en[0, -1] = 1
    return en @ np.linalg.solve(Wc, phi)


def place(A, B, poles):
    B = np.atleast_2d(B)
    if B.shape[1] == 1:
        return acker(A, B, poles)
    poles = np.asarray(poles, complex)
    # place_poles cannot take more coincident poles than rank(B): nudge duplicates
    p = list(poles)
    for i in range(len(p)):
        for j in range(i):
            if abs(p[i] - p[j]) < 1e-6:
                p[i] = p[i] * (1 + 1e-3 * (i + 1)) if p[i].imag == 0 else p[i]
    import scipy.signal  # lazy: keeps application start-up fast
    res = scipy.signal.place_poles(A, B, np.array(p), method="YT", maxiter=60)
    return np.asarray(res.gain_matrix)


# ---------------------------------------------------------------------------
@dataclass
class LongSASTargets:
    wn_sp: float
    zeta_sp: float
    wn_ph: float
    zeta_ph: float
    structure: str = "full"  # 'full' (u,nz/alpha,q,theta) | 'sp' (short-period only)
    input_index: int = 0


def auto_long_targets(model, cat, cls, min_effort=False, lim=np.radians(30), input_index=0, structure="full"):
    """Level 1 short-period / phugoid targets. min_effort=True searches the Level 1 band
    for the target needing the least control (see _min_effort_long)."""
    m = long_modes(model.Ad)
    L = hq.sp_limits(1, cat, cls, model.n_alpha())
    wlo, whi = L["wn"]
    if not np.isfinite(whi):
        whi = 10 * max(wlo, 1.0)
    if whi <= wlo:
        whi = wlo * 1.5
    lo, hi = wlo * 1.15, whi / 1.15
    if lo > hi:
        lo = hi = np.sqrt(wlo * whi)
    sp = m["sp"]
    unst = max([p.real for p in sp.poles if p.real > 0] + [0.0])
    if np.isfinite(sp.wn) and unst == 0:
        wn = sp.wn
    else:
        geo = np.sqrt(abs((sp.poles[0] * sp.poles[1]).real))
        wn = max(2.5 * unst, geo)
    wn = float(np.clip(wn, lo, hi))
    zeta = 0.7
    ph = m["ph"]
    wph = ph.wn if np.isfinite(ph.wn) else wn / 10
    wph = float(min(max(wph, 0.02), wn / 6))
    zph = float(max(ph.zeta if np.isfinite(ph.zeta) else 0.0, 0.6))
    zph = min(zph, 0.9)
    t = LongSASTargets(wn, zeta, wph, zph)
    if min_effort:
        t = _min_effort_long(model, t, cat, cls, lo, hi, lim, input_index, structure)
    t.structure, t.input_index = structure, input_index
    return t


def long_disturbances(model):
    """Design-coordinate states for the effort check: alpha = 5 deg, q = 5 deg/s."""
    d = np.radians(5)
    if model.convention == "uwqt":
        xs = [np.array([0, model.V0 * np.tan(d), 0, 0]), np.array([0, 0, d, 0])]
    else:
        xs = [np.array([0, d, 0, 0]), np.array([0, 0, d, 0])]
    return [model.T @ x for x in xs]


def lat_disturbances():
    """beta = 5 deg, p = 10 deg/s, r = 5 deg/s."""
    return [np.array([np.radians(5), 0, 0, 0]), np.array([0, np.radians(10), 0, 0]),
            np.array([0, 0, np.radians(5), 0])]


def effort_index(K, disturbances, lim):
    """Largest |command| / limit over the disturbance set (initial command of u = -K x)."""
    lim = np.atleast_1d(lim)
    return max(float(np.max(np.abs(K @ x) / lim[:K.shape[0]])) for x in disturbances)


def _min_effort_long(model, t0, cat, cls, lo, hi, lim, j, structure):
    """Short-period target inside the Level 1 band (with margin) that needs the least
    elevator for the disturbance set; a slight preference for zeta = 0.7 breaks ties."""
    L = hq.sp_limits(1, cat, cls, model.n_alpha())
    zlo = max(L["zeta"][0] + 0.1, 0.45)
    zhi = min(L["zeta"][1] - 0.05, 0.95)
    dist = long_disturbances(model)
    best = (np.inf, t0)
    for wn in np.geomspace(lo, hi, 14) if hi > lo else [lo]:
        for z in np.linspace(zlo, zhi, 9):
            t = LongSASTargets(float(wn), float(z), t0.wn_ph, t0.zeta_ph, structure, j)
            if structure != "sp":
                t.wn_ph = float(min(t0.wn_ph, wn / 6))
            try:
                K = design_long_sas(model, t, cat, cls).K
            except np.linalg.LinAlgError:
                continue
            J = effort_index(K[[j]], dist, lim) + 0.05 * abs(z - 0.7)
            if J < best[0]:
                best = (J, t)
    return best[1]


@dataclass
class LongSASResult:
    K: np.ndarray            # (m x 4) gain on design states, u = -K x
    Acl: np.ndarray
    targets: LongSASTargets
    poles: np.ndarray
    modes_ol: dict
    modes_cl: dict
    levels_ol: dict
    levels_cl: dict
    sp_limits: dict
    effort_alpha5: np.ndarray = field(default=None)  # command (rad) for 5 deg alpha perturbation
    warnings: list = field(default_factory=list)


def design_long_sas(model, targets, cat, cls):
    A, B = model.Ad, model.Bd
    j = targets.input_index
    b = B[:, [j]]
    psp = second_order_poles(targets.wn_sp, targets.zeta_sp)
    pph = second_order_poles(targets.wn_ph, targets.zeta_ph)
    warns = []
    k = np.zeros((1, 4))
    if targets.structure == "sp":
        Asp, Bsp, _, _ = model.decoupled()
        ksp = acker(Asp, Bsp[:, [j]], psp)
        k[0, 1:3] = ksp.ravel()
    else:
        k = acker(A, b, psp + pph)
    K = np.zeros((B.shape[1], 4))
    K[j] = k.ravel()
    Acl = A - B @ K
    poles = np.linalg.eigvals(Acl)
    if np.any(poles.real >= 0):
        warns.append("Closed-loop longitudinal SAS is NOT stable.")
    m_ol, m_cl = long_modes(A), long_modes(Acl)
    na = model.n_alpha()
    # 5 deg alpha perturbation in design coordinates
    if model.convention == "uwqt":
        x_phys = np.array([0, model.V0 * np.tan(np.radians(5)), 0, 0])
    else:
        x_phys = np.array([0, np.radians(5), 0, 0])
    eff = -K @ (model.T @ x_phys)
    return LongSASResult(K, Acl, targets, poles, m_ol, m_cl,
                         hq.evaluate_long(m_ol, cat, cls, na),
                         hq.evaluate_long(m_cl, cat, cls, na),
                         hq.sp_limits(1, cat, cls, na), eff, warns)


def sp_gain_domain(model, cat, cls, input_index=0, n=60):
    """Gains (K_nz/alpha, K_q) of the decoupled short period along the Level-1
    region boundary (user's 'domeniu de alocare'). Returns poles and gains."""
    Asp, Bsp, _, _ = model.decoupled()
    b = Bsp[:, [input_index]]
    L = hq.sp_limits(1, cat, cls, model.n_alpha())
    zmin, zmax = L["zeta"]
    zmax = min(zmax, 1.0)
    wmin, wmax = L["wn"]
    if not np.isfinite(wmax):
        wmax = 10 * max(wmin, 1)
    path = []
    for z in np.linspace(zmax, zmin, n):
        path.append((wmin, z))
    for w in np.linspace(wmin, wmax, n):
        path.append((w, zmin))
    for z in np.linspace(zmin, zmax, n):
        path.append((wmax, z))
    for w in np.linspace(wmax, wmin, n):
        path.append((w, zmax))
    Ks, P = [], []
    for w, z in path:
        p = second_order_poles(w, z)
        try:
            Ks.append(acker(Asp, b, p).ravel())
            P.append(p)
        except np.linalg.LinAlgError:
            pass
    return np.array(P), np.array(Ks)


# ---------------------------------------------------------------------------
@dataclass
class LatSASTargets:
    wn_dr: float
    zeta_dr: float
    roll_pole: float      # negative real
    spiral_pole: float    # real
    inputs: list = field(default_factory=lambda: [0, 1])
    washout_tau: float = 0.0   # yaw-rate washout s/(s+1/tau) on the r feedback; 0 = none


def auto_lat_targets(model, cat, cls, min_effort=False, lim=np.radians(30), inputs=None, washout=0.0):
    """Level 1 lateral targets; min_effort=True searches for the least surface command."""
    m = lat_modes(model.Ad)
    t = hq.lat_level1_targets(cat, cls)
    dr = m["dr"]
    wn = dr.wn if np.isfinite(dr.wn) else 1.0
    zeta = max(0.6, t["dr_zeta_min"] * 1.5)
    wn = max(wn, t["dr_wn_min"] * 1.15, t["dr_zw_min"] / zeta * 1.15)
    roll = m["roll"]
    if roll.kind == "real" and roll.poles[0].real < 0 and roll.tau <= 0.8 * t["roll_tau_max"]:
        rp = roll.poles[0].real
    else:
        rp = -1.0 / (0.7 * t["roll_tau_max"])
    sp = m.get("spiral")
    if sp is not None and sp.kind == "real" and sp.poles[0].real < -0.005:
        spp = sp.poles[0].real
    else:
        spp = -0.02
    if abs(spp - rp) < 1e-3:
        spp *= 0.5
    inputs = list(range(model.n_inputs))[:2] if inputs is None else inputs
    t0 = LatSASTargets(float(wn), float(zeta), float(rp), float(spp), inputs, washout)
    if not min_effort:
        return t0
    return _min_effort_lat(model, t0, m, t, lim, cat, cls)


def _min_effort_lat(model, t0, m, lv1, lim, cat, cls):
    """Search dutch-roll frequency/damping, roll and spiral targets inside Level 1 (with a
    margin) for the least surface command over the disturbance set (beta, p, r)."""
    dist = lat_disturbances()
    zmin = max(lv1["dr_zeta_min"] + 0.1, 0.3)
    dr = m["dr"]
    wnat = dr.wn if np.isfinite(dr.wn) else 1.0
    tau_max = lv1["roll_tau_max"]
    roll = m["roll"]
    rolls = [-1.0 / (0.9 * tau_max), -1.5 / tau_max, -2.5 / tau_max]
    if roll.kind == "real" and roll.poles[0].real < 0 and roll.tau <= 0.9 * tau_max:
        rolls.insert(0, float(roll.poles[0].real))
    sp = m.get("spiral")
    spirals = [-0.01, -0.03, -0.1]
    if sp is not None and sp.kind == "real" and sp.poles[0].real < -0.005:
        spirals.insert(0, float(sp.poles[0].real))
    idx = [i for i in t0.inputs if i < model.n_inputs] or [0]
    best = (np.inf, t0)
    for z in np.linspace(zmin, 0.9, 6):
        wlo = max(lv1["dr_wn_min"] * 1.1, lv1["dr_zw_min"] * 1.1 / z, 0.05)
        whi = max(3 * wlo, 1.5 * wnat)
        for wn in np.geomspace(wlo, whi, 10):
            for rp in rolls:
                for spp in spirals:
                    if abs(spp - rp) < 1e-3:
                        continue
                    t = LatSASTargets(float(wn), float(z), float(rp), float(spp), t0.inputs, t0.washout_tau)
                    try:
                        A, B = model.Ad, model.Bd
                        k = place(A, B[:, idx], second_order_poles(wn, z) + [complex(rp), complex(spp)])
                    except Exception:  # noqa: BLE001  (placement can fail for some targets)
                        continue
                    K = np.zeros((model.n_inputs, 4)); K[idx] = k
                    J = effort_index(K, dist, lim) + 0.02 * abs(z - 0.6)
                    if J < best[0]:
                        best = (J, t)
    return best[1]


@dataclass
class LatSASResult:
    K: np.ndarray
    Acl: np.ndarray
    targets: LatSASTargets
    poles: np.ndarray
    modes_ol: dict
    modes_cl: dict
    levels_ol: dict
    levels_cl: dict
    effort_beta5: np.ndarray = None
    warnings: list = field(default_factory=list)


def design_lat_sas(model, targets, cat, cls):
    A, B = model.Ad, model.Bd
    idx = [i for i in targets.inputs if i < B.shape[1]] or [0]
    Bu = B[:, idx]
    pdr = second_order_poles(targets.wn_dr, targets.zeta_dr)
    poles_t = pdr + [complex(targets.roll_pole), complex(targets.spiral_pole)]
    warns = []
    k = place(A, Bu, poles_t)
    K = np.zeros((B.shape[1], 4))
    K[idx] = k
    res = LatSASResult(K, None, targets, None, lat_modes(A), None,
                       hq.evaluate_lat(lat_modes(A), cat, cls), None, None, warns)
    if targets.washout_tau > 0:
        K[idx] = _replace_with_washout(model, res, idx, poles_t)
    Ae, Be, Ke, _, _ = lat_extended(model, res)
    Acl_e = Ae - Be @ Ke
    poles = np.linalg.eigvals(Acl_e)
    if np.any(poles.real >= 0):
        warns.append("Closed-loop lateral SAS is NOT stable.")
    tw = targets.washout_tau
    ev = poles
    if tw > 0:
        # drop the washout pole (the real pole farthest from every target) before mode identification
        real_idx = [i for i, p in enumerate(poles) if abs(p.imag) < 1e-6]
        if real_idx:
            iw = max(real_idx, key=lambda i: min(abs(poles[i] - t) for t in poles_t))
            ev = np.delete(poles, iw)
    m_cl = lat_modes_from_eigs(ev)
    res.Acl = A - B @ K
    res.Acl_ext = Acl_e
    res.poles = poles
    res.modes_cl = m_cl
    res.levels_cl = hq.evaluate_lat(m_cl, cat, cls)
    res.effort_beta5 = -K @ np.array([np.radians(5), 0, 0, 0])
    return res


def _replace_with_washout(model, res, idx, poles_t):
    """With the washout the SAS is a structured output feedback on [beta, p, r - w, phi],
    so exact pole placement is not possible: refine the gains numerically so the four
    airframe poles are as close as possible to the targets (washout pole left free, but stable)."""
    import scipy.optimize
    K0 = res.K[idx].copy()
    tgt = np.asarray(poles_t, complex)
    scale = np.maximum(np.abs(tgt), 0.05)

    def residual(kflat):
        res.K[idx] = kflat.reshape(K0.shape)
        Ae, Be, Ke, _, _ = lat_extended(model, res)
        ev = list(np.linalg.eigvals(Ae - Be @ Ke))
        r = []
        for t, s in zip(tgt, scale):
            i = int(np.argmin([abs(e - t) for e in ev]))
            e = ev.pop(i)
            r += [(e.real - t.real) / s, (e.imag - t.imag) / s]
        r.append(10 * max(0.0, ev[0].real + 0.01))   # leftover (washout) pole must be stable
        return np.array(r)

    sol = scipy.optimize.least_squares(residual, K0.ravel(), method="lm", max_nfev=4000)
    best = sol.x.reshape(K0.shape)
    res.K[idx] = K0
    return best


def lat_modes_from_eigs(ev):
    """lat_modes() for an arbitrary 4-eigenvalue set."""
    D = np.diag(np.asarray(ev, complex))
    return lat_modes(D)


def long_extended(model, res):
    """(A_ol, B, K_sas, state names, state units) in design coordinates."""
    return model.Ad.copy(), model.Bd.copy(), res.K.copy(), list(model.state_names), list(model.state_units)


def lat_extended(model, res):
    """Lateral model extended with the yaw-rate washout state (if any):
    w' = (r - w)/tau, the SAS uses r_w = r - w instead of r."""
    A, B, K = model.Ad.copy(), model.Bd.copy(), res.K.copy()
    names, units = list(model.state_names), list(model.state_units)
    tw = getattr(res.targets, "washout_tau", 0.0)
    if tw > 0:
        A = np.block([[A, np.zeros((4, 1))], [np.array([[0, 0, 1 / tw, 0]]), np.array([[-1 / tw]])]])
        B = np.vstack([B, np.zeros((1, B.shape[1]))])
        K = np.hstack([K, -K[:, [2]]])
        names.append("r_washout")
        units.append("rad/s")
    return A, B, K, names, units
