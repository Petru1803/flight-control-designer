"""Handling-qualities criteria: MIL-F-8785C (flying qualities of piloted airplanes),
plus C* and q/q_ss time-response envelopes (computed from a short-period response family).

Aircraft classes: I (small light), II-L / II-C (medium, land / carrier based),
III (large heavy), IV (high-maneuverability).
Flight-phase categories: A (non-terminal, rapid maneuvering / precision tracking),
B (non-terminal, gradual maneuvers), C (terminal: takeoff, approach, landing).
"""
import numpy as np
import scipy.signal

CLASSES = ["I", "II-L", "II-C", "III", "IV"]
CATEGORIES = ["A", "B", "C"]


# --- Longitudinal: short period (MIL-F-8785C 3.2.2.1) ------------------------
SP_ZETA = {  # level: {cat: (min, max)}
    1: {"A": (0.35, 1.30), "B": (0.30, 2.00), "C": (0.35, 1.30)},
    2: {"A": (0.25, 2.00), "B": (0.20, 2.00), "C": (0.25, 2.00)},
    3: {"A": (0.15, np.inf), "B": (0.15, np.inf), "C": (0.15, np.inf)},
}
SP_CAP = {  # CAP = wn^2/(n/alpha) [1/(g*s^2)]
    1: {"A": (0.28, 3.6), "B": (0.085, 3.6), "C": (0.16, 3.6)},
    2: {"A": (0.16, 10.0), "B": (0.038, 10.0), "C": (0.096, 10.0)},
    3: {"A": (0.0, np.inf), "B": (0.0, np.inf), "C": (0.0, np.inf)},
}


def sp_wn_floor(level, cat, cls):
    """Minimum wn_sp independent of n/alpha (Fig. 1-3 of 3.2.2.1.1)."""
    if cat == "A":
        return {1: 1.0, 2: 0.6, 3: 0.0}[level]
    if cat == "C":
        if cls in ("I", "II-C", "IV"):
            return {1: 0.87, 2: 0.6, 3: 0.0}[level]
        return {1: 0.7, 2: 0.4, 3: 0.0}[level]
    return 0.0


def sp_limits(level, cat, cls, n_alpha):
    zmin, zmax = SP_ZETA[level][cat]
    cmin, cmax = SP_CAP[level][cat]
    wmin = max(np.sqrt(cmin * n_alpha), sp_wn_floor(level, cat, cls))
    wmax = np.sqrt(cmax * n_alpha) if np.isfinite(cmax) else np.inf
    return {"zeta": (zmin, zmax), "wn": (wmin, wmax), "cap": (cmin, cmax)}


# --- Phugoid (3.2.1.2) --------------------------------------------------------
PH_ZETA_MIN = {1: 0.04, 2: 0.0}
PH_T2_MIN_L3 = 55.0

# --- Lateral-directional ------------------------------------------------------
# Dutch roll (3.3.1.1): (zeta_min, zeta*wn_min, wn_min)
def dr_limits(level, cat, cls):
    if level == 1:
        if cat == "A":
            return (0.19, 0.35, 1.0) if cls in ("I", "IV") else (0.19, 0.35, 0.4)
        if cat == "B":
            return (0.08, 0.15, 0.4)
        return (0.08, 0.15, 1.0) if cls in ("I", "II-C", "IV") else (0.08, 0.15, 0.4)
    if level == 2:
        return (0.02, 0.05, 0.4)
    return (0.0, -np.inf, 0.4)


# Roll mode time constant, maximum (3.3.1.2)
def roll_tau_max(level, cat, cls):
    fast = cls in ("I", "IV") if cat == "A" else (True if cat == "B" else cls in ("I", "II-C", "IV"))
    if cat == "B":
        return {1: 1.4, 2: 3.0, 3: 10.0}[level]
    if fast:
        return {1: 1.0, 2: 1.4, 3: 10.0}[level]
    return {1: 1.4, 2: 3.0, 3: 10.0}[level]


# Spiral minimum time to double (3.3.1.3)
def spiral_t2_min(level, cat):
    if level == 1:
        return 20.0 if cat == "B" else 12.0
    return {2: 8.0, 3: 4.0}[level]


# ----------------------------------------------------------------------------
def _inr(v, lo, hi):
    return np.isfinite(v) and lo <= v <= hi


def level_sp(mode, cat, cls, n_alpha):
    if not np.isfinite(mode.wn):
        return 4
    for lv in (1, 2, 3):
        L = sp_limits(lv, cat, cls, n_alpha)
        if _inr(mode.zeta, *L["zeta"]) and _inr(mode.wn, *L["wn"]):
            return lv
    return 4


def level_ph(mode):
    if np.isfinite(mode.zeta):
        if mode.zeta >= PH_ZETA_MIN[1]:
            return 1
        if mode.zeta >= PH_ZETA_MIN[2]:
            return 2
    lam = max(p.real for p in mode.poles)
    t2 = np.log(2) / lam if lam > 0 else np.inf
    return 3 if t2 >= PH_T2_MIN_L3 else 4


def level_dr(mode, cat, cls):
    if not np.isfinite(mode.wn):
        return 4
    for lv in (1, 2, 3):
        zmin, zwmin, wmin = dr_limits(lv, cat, cls)
        if mode.zeta >= zmin and mode.zeta * mode.wn >= zwmin and mode.wn >= wmin:
            return lv
    return 4


def level_roll(mode, cat, cls):
    if mode.kind == "coupled":
        return 4
    tau = mode.tau
    if not (np.isfinite(tau) and tau > 0):
        return 4
    for lv in (1, 2, 3):
        if tau <= roll_tau_max(lv, cat, cls):
            return lv
    return 4


def level_spiral(mode, cat):
    if mode.kind != "real":
        return 4
    t2 = mode.t2
    if not np.isfinite(t2):  # stable or neutral
        return 1
    for lv in (1, 2, 3):
        if t2 >= spiral_t2_min(lv, cat):
            return lv
    return 4


def evaluate_long(modes, cat, cls, n_alpha):
    return {"sp": level_sp(modes["sp"], cat, cls, n_alpha), "ph": level_ph(modes["ph"])}


def evaluate_lat(modes, cat, cls):
    out = {"dr": level_dr(modes["dr"], cat, cls), "roll": level_roll(modes["roll"], cat, cls)}
    if "spiral" in modes:
        out["spiral"] = level_spiral(modes["spiral"], cat)
    return out


def level_text(lv):
    return "Level 1" if lv == 1 else ("Level 2" if lv == 2 else ("Level 3" if lv == 3 else "> Level 3"))


# --- Lateral design targets that sit comfortably inside Level 1 ------------------
def lat_level1_targets(cat, cls):
    zmin, zwmin, wmin = dr_limits(1, cat, cls)
    return {"dr_zeta_min": zmin, "dr_zw_min": zwmin, "dr_wn_min": wmin,
            "roll_tau_max": roll_tau_max(1, cat, cls),
            "spiral_t2_min": spiral_t2_min(1, cat)}


# --- C* and q/q_ss envelopes -------------------------------------------------
_ENV = {}


# The envelopes are the spread of short-period responses with Level-1-like parameters.
# Short-period approximation: q/q_ss and gamma_dot/gamma_dot_ss share D(s) = s^2 + 2 zeta wn s + wn^2,
#   q/q_ss = wn^2 (1 + Ttheta2 s) / D,   nz/nz_ss = wn^2 / D,
# so C*/C*_ss = wn^2 (1 + w Ttheta2 s) / D with w = r / (1 + r), r = 12.5 g Ttheta2 / V in [0, 1].
# The responses are shown against the checked response normalised at T_END.
ENV_ZETA = (0.35, 1.30)          # MIL-F-8785C Level 1 short-period damping, categories A and C
ENV_WN = (2.0, 8.0)              # rad/s
ENV_WN_TTHETA2 = (1.5, 3.5)      # wn * Ttheta2 for q/q_ss (short-period frequency x lift lag)
ENV_WN_TAU_CSTAR = (0.0, 2.0)    # wn * w * Ttheta2 for C* (the nz part has no lead)
ENV_TOL = 0.05                   # absolute widening of both bounds
T_END = 3.0


def _sp_family(products, t):
    """Step responses wn^2 (1 + tau s) / D (steady state 1) for tau = product / wn."""
    out = []
    for z in np.linspace(*ENV_ZETA, 6):
        for wn in np.geomspace(*ENV_WN, 7):
            for pr in products:
                tau = pr / wn
                _, y = scipy.signal.step(([wn ** 2 * tau, wn ** 2], [1, 2 * z * wn, wn ** 2]), T=t)
                out.append(y)
    return np.array(out)


def envelopes():
    if not _ENV:
        t = np.linspace(0, T_END, 61)
        c = _sp_family(np.linspace(*ENV_WN_TAU_CSTAR, 6), t)
        q = _sp_family(np.linspace(*ENV_WN_TTHETA2, 6), t)
        for key, fam in (("cstar", c), ("qdiag", q)):
            lo = np.maximum(fam.min(axis=0) - ENV_TOL, 0.0)
            lo[0] = 0.0
            _ENV[key] = (t, lo, t, fam.max(axis=0) + ENV_TOL)
    return _ENV


def envelope_check(t, y, env):
    """Fraction of samples (inside each envelope's time span) that lie between bounds."""
    if env is None:
        return None
    xl, yl, xu, yu = env
    t = np.asarray(t)
    ok = np.ones_like(t, dtype=bool)
    ml = (t >= xl.min()) & (t <= xl.max())
    lo = np.interp(t[ml], xl, yl)
    ok[ml] &= y[ml] >= lo - 1e-3
    mu = (t >= xu.min()) & (t <= xu.max())
    hi = np.interp(t[mu], xu, yu)
    ok[mu] &= y[mu] <= hi + 1e-3
    span = ml | mu
    return float(ok[span].mean()) if span.any() else None
