"""Additional handling-qualities criteria (MIL-STD-1797A / MIL-F-8785C):
CAP-zeta, pitch-attitude bandwidth & phase delay, Gibson dropback, roll performance.

The bandwidth / phase-delay and Gibson boundaries are the commonly quoted values and are
editable in the UI - check them against the edition of the standard you must comply with."""
import numpy as np

from . import hq, lti
from .autopilot import add_actuators
from .models import long_modes
from .sas import lat_extended, long_extended

R2D = 180 / np.pi

# Bandwidth criterion defaults: {category: {level: (wbw_min [rad/s], tau_p_max [s])}}
BW_DEFAULTS = {"A": {1: (6.5, 0.12), 2: (4.5, 0.20)},
               "B": {1: (2.5, 0.12), 2: (1.5, 0.20)},
               "C": {1: (2.5, 0.12), 2: (1.5, 0.20)}}

# Roll performance, Level 1 (MIL-F-8785C Table IX, typical speed range): (bank [deg], time [s])
ROLL_REQ = {
    "I":    {"A": (60, 1.3), "B": (60, 1.7), "C": (30, 1.3)},
    "II-L": {"A": (45, 1.4), "B": (45, 1.9), "C": (30, 1.8)},
    "II-C": {"A": (45, 1.4), "B": (45, 1.9), "C": (25, 1.0)},
    "III":  {"A": (30, 1.5), "B": (30, 2.0), "C": (30, 2.5)},
    "IV":   {"A": (90, 1.3), "B": (90, 1.7), "C": (30, 1.0)},
}


def closed_loop_axis(axis, model, sas_res, actuator):
    """SAS-closed aircraft (with washout / actuators) driven by a pilot command added to
    the surface command: x' = (Aol - B K) x + B delta_pilot."""
    if axis == "long":
        Aol, B, K, names, units = long_extended(model, sas_res)
    else:
        Aol, B, K, names, units = lat_extended(model, sas_res)
    Aol, B, K, names, units = add_actuators(Aol, B, K, names, units, model.input_names, actuator)
    return Aol - B @ K, B, names


# --------------------------------------------------------------------------------
def cap_zeta(model, sas_res, cat, cls):
    sp = long_modes(sas_res.Acl)["sp"]
    na = model.n_alpha()
    cap = sp.wn ** 2 / na if np.isfinite(sp.wn) else np.nan
    return {"cap": cap, "zeta": sp.zeta, "wn": sp.wn, "n_alpha": na,
            "level": hq.level_sp(sp, cat, cls, na)}


def bandwidth(model, sas_res, actuator, input_index=0, w=None):
    A, B, names = closed_loop_axis("long", model, sas_res, actuator)
    n = A.shape[0]
    c = np.zeros(n); c[3] = 1.0   # theta
    if w is None:
        w = np.logspace(-2, 2.5, 3000)
    G = lti.freqresp(A, B[:, [input_index]], c, 0.0, w)
    if G[0].real < 0:             # pilot convention: positive attitude for positive command
        G = -G
    mag = 20 * np.log10(np.abs(G) + 1e-300)
    ph = np.degrees(np.unwrap(np.angle(G)))
    # bring the low-frequency phase near 0 / -90 (remove whole turns)
    ph -= 360 * np.round(ph[0] / 360)

    def cross(y, level):
        idx = np.where((y[:-1] - level) * (y[1:] - level) <= 0)[0]
        if not len(idx):
            return np.nan
        i = idx[0]
        f = (level - y[i]) / (y[i + 1] - y[i]) if y[i + 1] != y[i] else 0.0
        return float(10 ** (np.log10(w[i]) + f * (np.log10(w[i + 1]) - np.log10(w[i]))))

    w180 = cross(ph, -180.0)
    w_ph = cross(ph, -135.0)
    w_g = np.nan
    if np.isfinite(w180):
        # gain-margin bandwidth: highest frequency below w180 where |G| = |G(w180)| + 6 dB
        g180 = np.interp(np.log10(w180), np.log10(w), mag)
        lvl = g180 + 6.0
        below = w < w180
        idx = np.where(below[:-1] & ((mag[:-1] - lvl) * (mag[1:] - lvl) <= 0))[0]
        if len(idx):
            i = idx[-1]
            f = (lvl - mag[i]) / (mag[i + 1] - mag[i]) if mag[i + 1] != mag[i] else 0.0
            w_g = float(10 ** (np.log10(w[i]) + f * (np.log10(w[i + 1]) - np.log10(w[i]))))
    cands = [x for x in (w_ph, w_g) if np.isfinite(x)]
    wbw = min(cands) if cands else np.nan
    tau_p = 0.0
    if np.isfinite(w180) and 2 * w180 < w[-1]:
        ph2 = np.interp(np.log10(2 * w180), np.log10(w), ph)
        tau_p = -(ph2 + 180.0) / (R2D * 2 * w180)
    return {"wbw": wbw, "wbw_phase": w_ph, "wbw_gain": w_g, "w180": w180, "tau_p": tau_p,
            "pio_prone": bool(np.isfinite(w_g) and np.isfinite(w_ph) and w_g < w_ph),
            "w": w, "mag": mag, "phase": ph}


def bandwidth_level(bw, limits):
    """limits: {1: (wbw_min, tp_max), 2: (...)}"""
    if not np.isfinite(bw["wbw"]):
        return 4
    for lv in (1, 2):
        wmin, tmax = limits[lv]
        if bw["wbw"] >= wmin and bw["tau_p"] <= tmax:
            return lv
    return 3


def gibson(model, sas_res, input_index=0, hold=4.0, T=10.0):
    """Gibson pitch-attitude dropback on the short-period approximation with the SAS
    short-period gains: step of pilot command held for `hold` s, then released."""
    Asp, Bsp, _, _ = model.decoupled()
    K = sas_res.K[input_index, 1:3]
    b = Bsp[:, input_index]
    A = np.zeros((3, 3))
    A[:2, :2] = Asp - np.outer(b, K)
    A[2, 1] = 1.0                       # theta' = q
    Bv = np.array([b[0], b[1], 0.0]).reshape(3, 1)
    dt = 0.005
    t = np.arange(0, T + 1e-9, dt)
    u = (t < hold).astype(float).reshape(-1, 1)
    Y, X = lti.simulate(A, Bv, np.eye(3), np.zeros((3, 1)), u, dt)
    q, th = X[:, 1], X[:, 2]
    s = 1.0 if q[int(hold / dt) - 1] >= 0 else -1.0
    q, th = s * q, s * th
    ir = int(hold / dt)
    q_ss = q[ir - 1]
    q_pk = np.max(q[:ir])
    th_rel = th[ir - 1]
    after = th[ir:]
    th_fin = after[-1]
    th_pk = np.max(after)
    db = (th_pk - th_fin) if th_pk > th_fin + 1e-9 else (th_rel - th_fin)
    out = {"t": t, "q": q, "theta": th, "hold": hold, "q_ss": q_ss, "q_pk": q_pk,
           "qpk_qss": q_pk / q_ss if abs(q_ss) > 1e-12 else np.nan,
           "db_qss": db / q_ss if abs(q_ss) > 1e-12 else np.nan}
    # commonly quoted Gibson guidance: DB/q_ss in [0, 0.25] s and q_pk/q_ss <= 3 for precise tracking
    out["ok"] = bool(np.isfinite(out["db_qss"]) and 0 <= out["db_qss"] <= 0.25 and out["qpk_qss"] <= 3.0)
    return out


def roll_performance(model, sas_res, actuator, cls, cat, da_index=0, da_deg=20.0, T=6.0):
    A, B, names = closed_loop_axis("lat", model, sas_res, actuator)
    n = A.shape[0]
    dt = 0.005
    t = np.arange(0, T + 1e-9, dt)
    b = B[:, [da_index]] * np.radians(da_deg)
    _, X = lti.simulate(A, b, np.eye(n), np.zeros((n, 1)), np.ones((len(t), 1)), dt)
    phi = X[:, 3] * R2D
    if phi[min(len(phi) - 1, int(0.5 / dt))] < 0:
        phi = -phi
    req_bank, req_t = ROLL_REQ[cls][cat]
    idx = np.where(phi >= req_bank)[0]
    t_reach = float(t[idx[0]]) if len(idx) else np.inf
    return {"t": t, "phi": phi, "req_bank": req_bank, "req_time": req_t, "t_reach": t_reach,
            "ok": bool(t_reach <= req_t), "da_deg": da_deg}
