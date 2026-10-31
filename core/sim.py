"""Time-domain simulations: measurement noise and actuator saturation / rate limits
(with optional anti-windup by conditional integration)."""
import numpy as np

from . import cancel, lti
from .autopilot import _nbar, closed_loop


def coloured_noise(t, std, bw, seed=1):
    """White noise of given std filtered by bw/(s+bw) (like the user's H_noise)."""
    rng = np.random.default_rng(seed)
    w = std * rng.standard_normal(len(t))
    if bw <= 0:
        return w
    dt = t[1] - t[0]
    a = np.exp(-bw * dt)
    y = np.zeros_like(w)
    for k in range(1, len(t)):
        y[k] = a * y[k - 1] + (1 - a) * w[k - 1]
    return y


def run_noise(plant, ctrl, settings, seed=1):
    """Linear simulation with noise on every tracked measurement (same seeds for all methods)."""
    settings.ensure(plant)
    t = np.arange(0, settings.T + 1e-9, settings.dt)
    p, n = plant.p, plant.n
    sc = plant.state_scale[plant.ks]
    noise = np.column_stack([coloured_noise(t, settings.noise_std[i] / sc[i], settings.noise_bw, seed + i)
                             for i in range(p)])
    Acl, Bcl, Ccl, Dcl = closed_loop(plant, ctrl, settings.prefilter)
    x0 = np.zeros(Acl.shape[0]); x0[plant.ks] = settings.x0_track / sc
    u_clean = np.zeros((len(t), Bcl.shape[1])); u_clean[:, :p] = settings.ref / sc
    u_noisy = u_clean.copy(); u_noisy[:, p:2 * p] = noise
    Y0, _ = lti.simulate(Acl, Bcl, Ccl, Dcl, u_clean, settings.dt, x0)
    Y1, _ = lti.simulate(Acl, Bcl, Ccl, Dcl, u_noisy, settings.dt, x0)
    q = plant.q
    y0 = Y0[:, plant.ks] * sc
    y1 = Y1[:, plant.ks] * sc
    d1 = Y1[:, n + q:] * plant.input_scale
    d0 = Y0[:, n + q:] * plant.input_scale
    diff = y1 - y0
    return {"t": t, "noise": noise * sc, "y_clean": y0, "y_noisy": y1, "y_meas": y1 + noise * sc,
            "delta_noisy": d1, "delta_clean": d0,
            "rms_effect": float(np.sqrt(np.mean(diff[:, 0] ** 2))),
            "max_effect": float(np.max(np.abs(diff[:, 0]))),
            "rms_effect_all": np.sqrt(np.mean(diff ** 2, axis=0)),
            "rms_delta_effect": np.sqrt(np.mean((d1 - d0) ** 2, axis=0))}


def run_saturated(plant, ctrl, settings, antiwindup=None, fast=True):
    """Nonlinear simulation: total surface commands go through rate limit + saturation.
    The plant uses the open-loop matrices; SAS and autopilot both act through the actuators.
    Anti-windup (conditional integration): an integrator state is frozen while an autopilot
    channel is limited and integrating would push it further into the limit."""
    settings.ensure(plant)
    if antiwindup is None:
        antiwindup = settings.antiwindup
    t = np.arange(0, settings.T + 1e-9, settings.dt)
    # internal step small enough for the fastest closed-loop dynamics (the sampled
    # loop adds one step of delay, which must stay negligible)
    Acl = closed_loop(plant, ctrl, settings.prefilter)[0]
    lam = np.abs(np.linalg.eigvals(Acl)).max() if Acl.size else 1.0
    sub = int(np.clip(np.ceil(settings.dt * lam / 0.2), 1, 200))
    dt = settings.dt / sub
    n, m, nc, p, q = plant.n, plant.B.shape[1], ctrl.nc, plant.p, plant.q
    sc = plant.state_scale[plant.ks]
    r = settings.ref / sc
    Phi_p, Gam_p = lti.c2d(plant.Aol, plant.B, dt)
    if nc:
        Phi_c, Gam_c = lti.c2d(ctrl.Ac, ctrl.Bc, dt)
    pf = settings.prefilter
    a_pf = np.exp(-pf * dt) if pf > 0 else 0.0
    Nb = _nbar(ctrl, p)
    x = np.zeros(n); x[plant.ks] = settings.x0_track / sc
    xc = np.zeros(nc)
    xp = np.zeros(p)
    d_app = np.zeros(m)
    lim = np.where(np.isfinite(settings.limits), settings.limits, np.inf)
    rate = np.where(np.isfinite(settings.rates), settings.rates, np.inf)
    X = np.empty((len(t), n)); D = np.empty((len(t), m)); Dc = np.empty((len(t), m))
    Cc, Dcm, Ks = ctrl.Cc, ctrl.Dc, plant.Ksas
    ints = [s for s in ctrl.int_states if s < nc] if antiwindup else []
    # direction in which each integrator moves each autopilot command
    if ints:
        g = np.zeros((q, nc))
        for s in ints:
            col = Cc[:, s]
            if np.allclose(col, 0) and nc:
                col = Cc @ ctrl.Ac[:, s]
            g[:, s] = col
    frozen_steps = 0
    # linear fast path: while no command touches a deflection or rate limit the loop is
    # linear, so a whole output step is advanced exactly with the closed-loop transition
    # matrix; the small internal steps are used only around the limited intervals
    Acl_f, Bcl_f, Ccl_f, Dcl_f = closed_loop(plant, ctrl, pf)
    Phi_cl, Gam_cl = lti.c2d(Acl_f, Bcl_f[:, :p], settings.dt)
    Cd, Dd = Ccl_f[n + q:], Dcl_f[n + q:, :p]
    Dr = Dd @ r
    rate_step = rate * settings.dt
    npf = p if pf > 0 else 0
    for i in range(len(t)):
        if i % 200 == 0:
            cancel.check()
        z = np.concatenate([x, xc, xp[:npf]])
        d_now = Cd @ z + Dr
        if fast and np.all(np.abs(d_now) <= lim) and np.all(np.abs(d_now - d_app) <= 1e-12 + 1e-9 * np.abs(d_now)):
            z1 = Phi_cl @ z + Gam_cl @ r
            d1 = Cd @ z1 + Dr
            if np.all(np.abs(d1) <= lim) and np.all(np.abs(d1 - d_now) <= rate_step):
                X[i] = x; D[i] = d_now; Dc[i] = d_now
                x, xc = z1[:n], z1[n:n + nc]
                if npf:
                    xp = z1[n + nc:]
                d_app = d1
                if not np.all(np.isfinite(x)) or np.abs(x).max() > 1e8:
                    X[i + 1:] = np.nan; D[i + 1:] = np.nan; Dc[i + 1:] = np.nan
                    break
                continue
        for s in range(sub):
            rf = Nb @ (xp if pf > 0 else r)
            inp = np.concatenate([rf, x])
            u = (Cc @ xc + Dcm @ inp) if nc else Dcm @ inp
            d_cmd = -Ks @ x
            d_cmd[plant.js] += u
            step = np.clip(d_cmd - d_app, -rate * dt, rate * dt)
            d_app = np.clip(d_app + step, -lim, lim)
            if s == 0:
                X[i] = x; D[i] = d_app; Dc[i] = d_cmd
            x = Phi_p @ x + Gam_p @ d_app
            if nc:
                xc_new = Phi_c @ xc + Gam_c @ inp
                if ints:
                    excess = (d_cmd - d_app)[plant.js]
                    if np.any(np.abs(excess) > 1e-12):
                        for st in ints:
                            push = g[:, st] * (xc_new[st] - xc[st])
                            if np.any(push * excess > 0):
                                xc_new[st] = xc[st]
                                frozen_steps += 1
                xc = xc_new
            if pf > 0:
                xp = a_pf * xp + (1 - a_pf) * r
        if not np.all(np.isfinite(x)) or np.abs(x).max() > 1e8:
            X[i + 1:] = np.nan; D[i + 1:] = np.nan; Dc[i + 1:] = np.nan
            break
    xs = X * plant.state_scale
    return {"t": t, "y": xs[:, plant.ks], "x": xs, "delta": D * plant.input_scale,
            "delta_cmd": Dc * plant.input_scale,
            "saturated": bool(np.any(np.abs(D) >= lim - 1e-12)),
            "diverged": bool(np.any(~np.isfinite(X))),
            "antiwindup": bool(antiwindup and ctrl.int_states), "frozen_steps": frozen_steps}
