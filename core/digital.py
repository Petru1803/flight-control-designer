"""Digital implementation check: sample rate, controller discretisation (Tustin / ZOH),
optional one-sample computational delay, discrete stability, margins and response."""
import numpy as np

from . import lti
from .autopilot import _nbar, evaluate, loop_margins

METHODS = {"Tustin (bilinear)": "bilinear", "ZOH": "zoh", "Backward Euler": "backward_diff"}


def discretize(A, B, C, D, Ts, method):
    if A.shape[0] == 0:
        return A, B, C, D
    import scipy.signal  # lazy
    Ad, Bd, Cd, Dd, _ = scipy.signal.cont2discrete((A, B, C, D), Ts, method=method)
    return Ad, Bd, Cd, Dd


def suggest_ts(plant, ctrls, ratio=30.0, method="bilinear", delay=True, max_loss=5.0):
    """Sample time: start from w_s >= ratio * w_c (fastest loop crossover / airframe+SAS pole),
    then, if a discretisation method is given, the largest 'nice' Ts for which every controller
    keeps its phase margin within max_loss degrees of the continuous one."""
    wc = []
    for c in ctrls:
        mg = loop_margins(plant, c)
        for ch in mg["channels"]:
            if np.isfinite(ch["wcp"]):
                wc.append(ch["wcp"])
    # airframe + SAS poles only: the Pade / actuator states are not computer dynamics
    sas_poles = np.abs(np.linalg.eigvals(plant.design.A))
    wmax = max(wc + [float(np.max(sas_poles))] if len(sas_poles) else wc + [1.0])
    Ts = 2 * np.pi / (ratio * wmax)
    # round down to a "nice" value
    ok = NICE[NICE <= Ts]
    Ts = float(ok.max()) if len(ok) else NICE[0]
    if method is None:
        return Ts, float(wmax)
    # then make it fast enough that no controller loses more than max_loss deg of phase
    # (derivative / fast controller dynamics need more than the crossover rule)
    pm_c = [loop_margins(plant, c)["pm"] for c in ctrls]
    for cand in NICE[NICE <= Ts][::-1]:
        good = True
        for c, pmc in zip(ctrls, pm_c):
            md = _loop_margins_d(plant, c, float(cand), method, delay)
            if not (np.isfinite(md["pm"]) and (not np.isfinite(pmc) or pmc - md["pm"] <= max_loss)):
                good = False
                break
        if good:
            return float(cand), float(wmax)
    return float(NICE[0]), float(wmax)


NICE = np.array([0.001, 0.002, 0.0025, 0.005, 0.01, 0.0125, 0.02, 0.025, 0.04, 0.05, 0.1, 0.2])


def _loop_margins_d(plant, ctrl, Ts, method, comp_delay):
    m, q = plant.B.shape[1], plant.q
    Phi, Gam, Acd, Bcd, Ccd, Dcd = _discrete_parts(plant, ctrl, None, Ts, method)
    p = plant.p
    E = np.zeros((m, q))
    for i, j in enumerate(plant.js):
        E[j, i] = 1
    return _margins_d(plant, Phi, Gam, Acd, Bcd[:, p:], Ccd, Dcd[:, p:], E, Ts, comp_delay)


def _discrete_parts(plant, ctrl, settings, Ts, method):
    Phi, Gam = lti.c2d(plant.Aol, plant.B, Ts)
    Acd, Bcd, Ccd, Dcd = discretize(ctrl.Ac, ctrl.Bc, ctrl.Cc, ctrl.Dc, Ts, method)
    return Phi, Gam, Acd, Bcd, Ccd, Dcd


def digital_check(plant, ctrl, settings, Ts, method="bilinear", comp_delay=True):
    settings.ensure(plant)
    n, m, p, q, nc = plant.n, plant.B.shape[1], plant.p, plant.q, ctrl.nc
    Phi, Gam, Acd, Bcd, Ccd, Dcd = _discrete_parts(plant, ctrl, settings, Ts, method)
    Bcr, Bcx = Bcd[:, :p], Bcd[:, p:]
    Dcr, Dcx = Dcd[:, :p], Dcd[:, p:]
    E = np.zeros((m, q))
    for i, j in enumerate(plant.js):
        E[j, i] = 1
    Nb = _nbar(ctrl, p)
    pf = settings.prefilter
    a = np.exp(-pf * Ts) if pf > 0 else 0.0
    npf = p if pf > 0 else 0
    nd = m if comp_delay else 0
    N = n + nc + npf + nd
    sx, sc, sp, sd = slice(0, n), slice(n, n + nc), slice(n + nc, n + nc + npf), slice(n + nc + npf, N)
    M = np.zeros((N, N)); Br = np.zeros((N, p))
    # command delta_k = -Ksas x + E u_k,  u_k = Ccd xc + Dcr rf + Dcx x
    Dx = -plant.Ksas + E @ Dcx                    # delta from x
    Dxc = E @ Ccd                                 # delta from xc
    Dxp = E @ Dcr @ Nb if npf else np.zeros((m, 0))
    Dr = np.zeros((m, p)) if npf else E @ Dcr @ Nb
    if comp_delay:
        M[sx, sd] = Gam
        M[sd, sx] = Dx; M[sd, sc] = Dxc
        if npf:
            M[sd, sp] = Dxp
        Br[sd] = Dr
    else:
        M[sx, sx] = Gam @ Dx; M[sx, sc] = Gam @ Dxc
        if npf:
            M[sx, sp] = Gam @ Dxp
        Br[sx] = Gam @ Dr
    M[sx, sx] += Phi
    M[sc, sx] = Bcx; M[sc, sc] = Acd
    if npf:
        M[sc, sp] = Bcr @ Nb
        M[sp, sp] = a * np.eye(p)
        Br[sp] = (1 - a) * np.eye(p)
    else:
        Br[sc] = Bcr @ Nb
    rho = float(np.max(np.abs(np.linalg.eigvals(M)))) if N else 0.0
    # equivalent continuous growth rate log(rho)/Ts, with the tolerance used for the
    # continuous loop (near-zero integrator poles give rho = 1 - 1e-9 and must not flip)
    growth = np.log(max(rho, 1e-300)) / Ts
    stable = bool(growth < 1e-7)
    # step response
    sc_ = plant.state_scale[plant.ks]
    r = settings.ref / sc_
    Ns = int(round(settings.T / Ts)) + 1
    z = np.zeros(N); z[plant.ks] = settings.x0_track / sc_
    Y = np.empty((Ns, p))
    for k in range(Ns):
        Y[k] = z[plant.ks]
        z = M @ z + Br @ r
        if not np.all(np.isfinite(z)) or np.abs(z).max() > 1e9:
            Y[k + 1:] = np.nan
            break
    Y *= sc_
    td = np.arange(Ns) * Ts
    cont = evaluate(plant, ctrl, settings, with_margins=False)
    yc = np.column_stack([np.interp(td, cont.t, cont.y[:, i]) for i in range(p)])
    amp = np.maximum(np.abs(settings.ref), np.abs(settings.x0_track))
    amp = np.where(amp > 0, amp, 1.0)
    dev = float(np.nanmax(np.abs(Y - yc) / amp) * 100) if stable else np.inf
    # discrete margins, loop broken at each autopilot input (others closed)
    margins = _margins_d(plant, Phi, Gam, Acd, Bcx, Ccd, Dcx, E, Ts, comp_delay)
    return {"Ts": Ts, "method": method, "delay": comp_delay, "stable": stable, "rho": rho, "growth": growth,
            "t": td, "y": Y, "y_cont": yc, "max_dev_pct": dev, "margins": margins,
            "Ad": Acd, "Bd": Bcd, "Cd": Ccd, "Dd": Dcd}


def _margins_d(plant, Phi, Gam, Acd, Bcx, Ccd, Dcx, E, Ts, comp_delay):
    n, m, q, nc = plant.n, plant.B.shape[1], plant.q, Acd.shape[0]
    nd = m if comp_delay else 0
    N = n + nc + nd
    AL = np.zeros((N, N)); BL = np.zeros((N, q)); CL = np.zeros((q, N))
    sx, sc, sd = slice(0, n), slice(n, n + nc), slice(n + nc, N)
    if comp_delay:
        AL[sx, sx] = Phi
        AL[sx, sd] = Gam
        AL[sd, sx] = -plant.Ksas
        BL[sd] = E
    else:
        AL[sx, sx] = Phi - Gam @ plant.Ksas
        BL[sx] = Gam @ E
    AL[sc, sx] = Bcx
    AL[sc, sc] = Acd
    CL[:, sx] = -Dcx
    CL[:, sc] = -Ccd
    chans = []
    for i in range(q):
        o = [x for x in range(q) if x != i]
        Ai = AL - BL[:, o] @ CL[o, :] if o else AL
        chans.append(lti.margins_d(Ai, BL[:, [i]], CL[[i], :], 0.0, Ts, npts=2000))
    worst = dict(chans[0])
    for c in chans[1:]:
        if abs(c["gm_db"]) < abs(worst["gm_db"]):
            worst["gm_db"] = c["gm_db"]
        worst["pm"] = min(worst["pm"], c["pm"])
        worst["dm"] = min(worst["dm"], c["dm"])
    worst["channels"] = chans
    return worst
