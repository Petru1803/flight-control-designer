"""Matplotlib figures shared by the UI and the PDF/PNG export (no pyplot state)."""
import numpy as np
from matplotlib.figure import Figure

from . import hq
from . import lti
from .models import second_order_poles

COLORS = {"PIDF": "#d62728", "LQI": "#1f77b4", "LQG": "#2ca02c", "Hinf": "#9467bd",
          "LoopShaping": "#ff7f0e"}
LABELS = {"PIDF": "PIDF", "LQI": "LQI", "LQG": "LQG", "Hinf": "H∞", "LoopShaping": "H∞ LS"}


def new_fig(w=7.5, h=4.6):
    f = Figure(figsize=(w, h), dpi=100, tight_layout=True)
    return f


def _sp_region(ax, lim, color="r"):
    zmin, zmax = lim["zeta"]
    zmax = min(zmax, 1.0)
    wmin, wmax = lim["wn"]
    if not np.isfinite(wmax):
        wmax = 3 * max(wmin, 1)
    r = np.linspace(wmin, wmax, 100)
    for z in (zmin, zmax):
        for sgn in (1, -1):
            ax.plot(-z * r, sgn * r * np.sqrt(max(1 - z * z, 0)), color=color, lw=1.4)
    phi = np.linspace(np.arccos(zmax), np.arccos(zmin), 100)
    for w in (wmin, wmax):
        for sgn in (1, -1):
            ax.plot(-w * np.cos(phi), sgn * w * np.sin(phi), color=color, lw=1.4)


def pz_long(model, res, cat, cls):
    f = new_fig()
    ax = f.add_subplot(111)
    _sp_region(ax, res.sp_limits)
    ol = np.linalg.eigvals(model.Ad)
    ax.plot(ol.real, ol.imag, "kx", ms=9, mew=2, label="open loop")
    ax.plot(res.poles.real, res.poles.imag, "o", mfc="none", mec="#1f77b4", ms=9, mew=2, label="with SAS")
    ax.plot([], [], "r", label=f"Level 1 short period (Cat {cat}, Class {cls})")
    ax.axhline(0, color="0.6", lw=0.6); ax.axvline(0, color="0.6", lw=0.6)
    ax.set_xlabel("Re [1/s]"); ax.set_ylabel("Im [rad/s]")
    ax.set_title("Longitudinal poles and MIL-F-8785C Level 1 short-period region")
    ax.grid(True, alpha=0.3); ax.legend(loc="best", fontsize=8)
    ax.set_aspect("equal", adjustable="datalim")
    return f


def pz_lat(model, res, cat, cls):
    f = new_fig()
    ax = f.add_subplot(111)
    t = hq.lat_level1_targets(cat, cls)
    ol = np.linalg.eigvals(model.Ad)
    allp = np.concatenate([ol, res.poles])
    R = max(2.0, 1.3 * np.abs(allp).max())
    # dutch-roll Level 1 boundaries
    z = t["dr_zeta_min"]
    rr = np.linspace(0, R, 50)
    for sgn in (1, -1):
        ax.plot(-z * rr, sgn * rr * np.sqrt(1 - z * z), "r--", lw=1.2)
    ax.axvline(-t["dr_zw_min"], color="r", ls=":", lw=1.2)
    phi = np.linspace(0, np.pi, 100)
    ax.plot(-t["dr_wn_min"] * np.sin(phi), t["dr_wn_min"] * np.cos(phi), "r-.", lw=1.0)
    ax.axvline(-1 / t["roll_tau_max"], color="m", ls=":", lw=1.0)
    ax.plot(ol.real, ol.imag, "kx", ms=9, mew=2, label="open loop")
    ax.plot(res.poles.real, res.poles.imag, "o", mfc="none", mec="#1f77b4", ms=9, mew=2, label="with SAS")
    ax.plot([], [], "r--", label=f"DR ζ min = {z}")
    ax.plot([], [], "r:", label=f"DR ζωn min = {t['dr_zw_min']}")
    ax.plot([], [], "r-.", label=f"DR ωn min = {t['dr_wn_min']}")
    ax.plot([], [], "m:", label=f"roll 1/τ min (τ ≤ {t['roll_tau_max']} s)")
    ax.axhline(0, color="0.6", lw=0.6); ax.axvline(0, color="0.6", lw=0.6)
    ax.set_xlabel("Re [1/s]"); ax.set_ylabel("Im [rad/s]")
    ax.set_title(f"Lateral-directional poles, Level 1 limits (Cat {cat}, Class {cls})")
    ax.grid(True, alpha=0.3); ax.legend(loc="best", fontsize=8)
    return f


def sp_domain(model, res, cat, cls, P, K, input_index=0):
    f = new_fig()
    ax = f.add_subplot(111)
    if len(K):
        ax.plot(K[:, 0], K[:, 1], "b", lw=1.5, label="Level 1 boundary (decoupled SP)")
        ax.fill(K[:, 0], K[:, 1], color="b", alpha=0.08)
    k = res.K[input_index]
    if res.targets.structure == "sp":
        ax.plot(k[1], k[2], "r*", ms=14, label="chosen SAS gains")
    s1 = model.state_names[1]
    ax.set_xlabel(f"K_{s1}"); ax.set_ylabel("K_q")
    ax.set_title("Short-period gain domain giving Level 1 poles")
    ax.grid(True, alpha=0.3); ax.legend(loc="best", fontsize=8)
    return f


def cstar_qdiag(model, res, input_index=0):
    """C* = nz + 12.5 q and q/q_ss for an elevator step on the SAS-augmented aircraft."""
    t = np.linspace(0, 3, 301)
    A = res.Acl
    b = model.Bd[:, [input_index]]
    C = np.vstack([np.array([[0, 0, 1, 0]]), model.c_nz.reshape(1, 4)])
    D = np.array([[0.0], [model.d_nz[input_index]]])
    Y, _ = lti.simulate(A, b, C, D, np.ones((len(t), 1)), t[1] - t[0])
    q, nz = Y[:, 0], Y[:, 1]
    cstar = 12.5 * q + nz
    cs = cstar / cstar[-1] if abs(cstar[-1]) > 1e-12 else cstar
    qq = q / q[-1] if abs(q[-1]) > 1e-12 else q
    env = hq.envelopes()
    f = new_fig(7.5, 5.2)
    a1 = f.add_subplot(211)
    a2 = f.add_subplot(212)
    if env.get("cstar") is not None:
        xl, yl, xu, yu = env["cstar"]
        a1.plot(xl, yl, "r", label="lower bound"); a1.plot(xu, yu, "y", label="upper bound")
    a1.plot(t, cs, "b", lw=1.6, label="C* (normalised)")
    a1.set_ylabel("C*/C*(3 s)"); a1.grid(True, alpha=0.3); a1.legend(fontsize=8)
    a1.set_title("C* criterion")
    if env.get("qdiag") is not None:
        xl, yl, xu, yu = env["qdiag"]
        a2.plot(xl, yl, "r", label="lower bound"); a2.plot(xu, yu, "y", label="upper bound")
    a2.plot(t, qq, "b", lw=1.6, label="q/q_ss")
    a2.set_xlabel("t [s]"); a2.set_ylabel("q/q(3 s)"); a2.grid(True, alpha=0.3); a2.legend(fontsize=8)
    a2.set_title("q/q_ss criterion")
    checks = {"cstar": hq.envelope_check(t, cs, env.get("cstar")),
              "qdiag": hq.envelope_check(t, qq, env.get("qdiag"))}
    return f, checks


def sas_response_long(model, res, input_index=0, T=20):
    t = np.linspace(0, T, 1001)
    f = new_fig(7.5, 5.2)
    sc = [1 if u not in ("rad", "rad/s") else 180 / np.pi for u in model.state_units]
    units = [u.replace("rad", "deg") for u in model.state_units]
    x0 = np.zeros(4)
    if model.convention == "uwqt":
        x0 = model.T @ np.array([0, model.V0 * np.tan(np.radians(5)), 0, 0])
    else:
        x0[1] = np.radians(5)
    for i, (A, lbl, st) in enumerate([(model.Ad, "open loop", "k--"), (res.Acl, "with SAS", "b")]):
        _, X = lti.simulate(A, np.zeros((4, 1)), np.eye(4), np.zeros((4, 1)), np.zeros((len(t), 1)),
                            t[1] - t[0], x0)
        for s in range(4):
            ax = f.add_subplot(2, 2, s + 1) if i == 0 else f.axes[s]
            ax.plot(t, X[:, s] * sc[s], st, lw=1.3, label=lbl)
            ax.set_title(f"{model.state_names[s]} [{units[s]}]", fontsize=9)
            ax.grid(True, alpha=0.3)
    f.axes[0].legend(fontsize=8)
    f.suptitle("Response to a 5° angle-of-attack perturbation", fontsize=10)
    return f


def sas_response_lat(model, res, T=20):
    t = np.linspace(0, T, 1001)
    f = new_fig(7.5, 5.2)
    x0 = np.array([np.radians(5), 0, 0, 0])
    for i, (A, lbl, st) in enumerate([(model.Ad, "open loop", "k--"), (res.Acl, "with SAS", "b")]):
        _, X = lti.simulate(A, np.zeros((4, 1)), np.eye(4), np.zeros((4, 1)), np.zeros((len(t), 1)),
                            t[1] - t[0], x0)
        for s in range(4):
            ax = f.add_subplot(2, 2, s + 1) if i == 0 else f.axes[s]
            y = X[:, s] * 180 / np.pi
            ax.plot(t, np.clip(y, -1e3, 1e3), st, lw=1.3, label=lbl)
            u = model.state_units[s].replace("rad", "deg")
            ax.set_title(f"{model.state_names[s]} [{u}]", fontsize=9)
            ax.grid(True, alpha=0.3)
    for s in range(4):
        _, X = lti.simulate(res.Acl, np.zeros((4, 1)), np.eye(4), np.zeros((4, 1)),
                            np.zeros((len(t), 1)), t[1] - t[0], x0)
        m = max(np.abs(X[:, s]).max() * 180 / np.pi, 1e-3)
        f.axes[s].set_ylim(-2.5 * m, 2.5 * m)
    f.axes[0].legend(fontsize=8)
    f.suptitle("Response to a 5° sideslip perturbation", fontsize=10)
    return f


COLORS["Cascade"] = "#8c564b"
LABELS["Cascade"] = "Cascade"


def ap_response(results, plant, settings, title):
    p = plant.p
    show_bank = plant.phi_index is not None and "psi" in plant.track_names
    nplots = p + (1 if show_bank else 0)
    f = new_fig(7.5, 2.9 * nplots + 0.6)
    for i in range(p):
        ax = f.add_subplot(nplots, 1, i + 1)
        for m, r in results.items():
            if r is None:
                continue
            ax.plot(r.t, r.y[:, i], color=COLORS.get(m), lw=1.6, label=LABELS.get(m, m))
        ax.axhline(settings.ref[i], color="k", ls="--", lw=1, label="reference")
        ax.set_ylabel(f"{plant.track_names[i]} [{plant.track_units[i]}]")
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.set_title(title)
            ax.legend(fontsize=8, loc="best")
    if show_bank:
        ax = f.add_subplot(nplots, 1, nplots)
        for m, r in results.items():
            if r is not None:
                ax.plot(r.t, r.x[:, plant.phi_index], color=COLORS.get(m), lw=1.3)
        if settings.bank_limit > 0:
            ax.axhline(settings.bank_limit, color="k", ls=":", lw=1)
            ax.axhline(-settings.bank_limit, color="k", ls=":", lw=1)
        ax.set_ylabel("bank φ [deg]"); ax.grid(True, alpha=0.3)
    f.axes[-1].set_xlabel("t [s]")
    return f


def fc_response(designs):
    """Flight-controller export: attitude steps, continuous (solid) and digital at the export
    sample time with a one-sample delay (dashed)."""
    axes = [a for a in ("pitch", "roll") if a in designs]
    f = new_fig(7.5, 2.9 * max(len(axes), 1) + 0.6)
    for i, a in enumerate(axes):
        d = designs[a]
        ax = f.add_subplot(len(axes), 1, i + 1)
        r = d.result
        ax.plot(r.t, r.y[:, 0], lw=1.6, label=f"{d.structure} (continuous)")
        if d.digital and d.digital.get("stable"):
            ax.plot(d.digital["t"], d.digital["y"][:, 0], ls="--", lw=1.2,
                    label=f"digital {1 / d.Ts:.0f} Hz + 1-sample delay")
        ax.axhline(d.settings.ref[0], color="k", ls=":", lw=1)
        ax.set_ylabel(("θ" if a == "pitch" else "φ") + " [deg]")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc="best")
        ax.set_title(f"{a.capitalize()} attitude step ({d.firmware} export)")
    if f.axes:
        f.axes[-1].set_xlabel("t [s]")
    return f


def fc_surfaces(designs):
    rows = []
    for a in ("pitch", "roll"):
        if a in designs:
            d = designs[a]
            for j in d.plant.js:
                rows.append((d, j))
    f = new_fig(7.5, 2.3 * max(len(rows), 1) + 0.6)
    for i, (d, j) in enumerate(rows):
        ax = f.add_subplot(len(rows), 1, i + 1)
        r = d.result
        ax.plot(r.t, r.delta[:, j], lw=1.4)
        lim = np.degrees(d.dmax[d.plant.input_names[j]])
        ax.axhline(lim, color="k", ls="--", lw=1); ax.axhline(-lim, color="k", ls="--", lw=1)
        ax.set_ylabel(f"{d.plant.input_names[j]} [deg]")
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.set_title("Surface deflections (dashed: maximum deflection = full torque)")
    if f.axes:
        f.axes[-1].set_xlabel("t [s]")
    return f


def ap_effort(results, plant, settings):
    m = plant.B.shape[1]
    f = new_fig(7.5, 2.6 * m + 0.6)
    for i in range(m):
        ax = f.add_subplot(m, 1, i + 1)
        for meth, r in results.items():
            if r is None:
                continue
            ax.plot(r.t, r.delta[:, i], color=COLORS.get(meth), lw=1.4, label=LABELS.get(meth, meth))
        lim = settings.limits[i] * plant.input_scale[i]
        if np.isfinite(lim):
            ax.axhline(lim, color="k", ls="--", lw=1); ax.axhline(-lim, color="k", ls="--", lw=1)
        unit = "deg" if plant.input_scale[i] != 1 else "input units"
        ax.set_ylabel(f"{plant.input_names[i]} [{unit}]")
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.set_title("Total control effort (autopilot + SAS)")
            ax.legend(fontsize=8, loc="best")
    f.axes[-1].set_xlabel("t [s]")
    return f


def ap_noise(noise, plant, settings):
    f = new_fig(7.5, 5.4)
    a1 = f.add_subplot(211)
    a2 = f.add_subplot(212)
    first = True
    for meth, nz in noise.items():
        if nz is None:
            continue
        if first:
            a1.plot(nz["t"], nz["y_meas"][:, 0], color="0.75", lw=0.6, label="measurement (example)")
            first = False
        a1.plot(nz["t"], nz["y_noisy"][:, 0], color=COLORS.get(meth), lw=1.2, label=LABELS.get(meth, meth))
        a2.plot(nz["t"], nz["y_noisy"][:, 0] - nz["y_clean"][:, 0], color=COLORS.get(meth), lw=1.0,
                label=f"{LABELS.get(meth, meth)}  RMS={nz['rms_effect']:.4f}")
    a1.axhline(settings.ref[0], color="k", ls="--", lw=1)
    nm = plant.track_names[0]
    a1.set_ylabel(f"{nm} [{plant.track_unit}]"); a1.grid(True, alpha=0.3); a1.legend(fontsize=7)
    a1.set_title(f"Response with measurement noise on {nm} (σ = {settings.noise_std[0]:g} {plant.track_unit})")
    a2.set_ylabel(f"Δ{nm} (noise effect)"); a2.set_xlabel("t [s]")
    a2.grid(True, alpha=0.3); a2.legend(fontsize=7)
    return f


def ap_saturation(results, sat, plant, settings):
    f = new_fig(7.5, 5.4)
    a1 = f.add_subplot(211)
    a2 = f.add_subplot(212)
    for meth, s in sat.items():
        if s is None:
            continue
        c = COLORS.get(meth)
        r = results.get(meth)
        if r is not None:
            a1.plot(r.t, r.y[:, 0], color=c, lw=0.9, ls="--")
        if s.get("noaw") is not None:
            a1.plot(s["t"], s["noaw"]["y"][:, 0], color=c, lw=1.0, ls=":")
        a1.plot(s["t"], s["y"][:, 0], color=c, lw=1.6, label=LABELS.get(meth, meth) + (" (diverged)" if s["diverged"] else ""))
        a2.plot(s["t"], s["delta"][:, plant.j], color=c, lw=1.3, label=LABELS.get(meth, meth))
    a1.axhline(settings.ref[0], color="k", ls=":", lw=1)
    lim = settings.limits[plant.j] * plant.input_scale[plant.j]
    if np.isfinite(lim):
        a2.axhline(lim, color="k", ls="--", lw=1); a2.axhline(-lim, color="k", ls="--", lw=1)
    nm = plant.track_names[0]
    a1.set_title("Saturation + rate limit: anti-windup (solid), no anti-windup (dotted), linear (dashed)",
                 fontsize=9)
    a1.set_ylabel(f"{nm} [{plant.track_unit}]"); a1.grid(True, alpha=0.3); a1.legend(fontsize=8)
    unit = "deg" if plant.input_scale[plant.j] != 1 else "input units"
    a2.set_ylabel(f"{plant.input_names[plant.j]} applied [{unit}]"); a2.set_xlabel("t [s]")
    a2.grid(True, alpha=0.3)
    return f


def ap_bode(results, plants_ctrl):
    """Loop transfer at the plant input for each method."""
    f = new_fig(7.5, 5.2)
    a1 = f.add_subplot(211)
    a2 = f.add_subplot(212, sharex=a1)
    for meth, r in results.items():
        if r is None or "w" not in r.margins:
            continue
        w, L = r.margins["w"], r.margins["L"]
        a1.semilogx(w, 20 * np.log10(np.abs(L) + 1e-300), color=COLORS.get(meth), label=LABELS.get(meth, meth))
        a2.semilogx(w, np.degrees(np.unwrap(np.angle(L))), color=COLORS.get(meth))
    a1.axhline(0, color="k", lw=0.8)
    a1.set_ylabel("|L| [dB]"); a1.grid(True, which="both", alpha=0.3); a1.legend(fontsize=8)
    a1.set_title("Open-loop transfer at the autopilot command (loop broken at plant input)")
    a1.set_ylim(-80, 80)
    a2.set_ylabel("phase [deg]"); a2.set_xlabel("ω [rad/s]"); a2.grid(True, which="both", alpha=0.3)
    return f


def digital_plot(results, digital, plant, settings):
    p = plant.p
    f = new_fig(7.5, 2.9 * p + 0.6)
    for i in range(p):
        ax = f.add_subplot(p, 1, i + 1)
        for meth, d in digital.items():
            if d is None:
                continue
            c = COLORS.get(meth)
            r = results.get(meth)
            if r is not None:
                ax.plot(r.t, r.y[:, i], color=c, lw=0.9, ls="--")
            ax.step(d["t"], d["y"][:, i], where="post", color=c, lw=1.3,
                    label=f"{LABELS.get(meth, meth)} ({'stable' if d['stable'] else 'UNSTABLE'})")
        ax.axhline(settings.ref[i], color="k", ls=":", lw=1)
        ax.set_ylabel(f"{plant.track_names[i]} [{plant.track_units[i]}]"); ax.grid(True, alpha=0.3)
        if i == 0:
            d0 = next(iter(digital.values()))
            ax.set_title(f"Digital implementation (Ts = {d0['Ts']*1000:g} ms, {d0['method']}"
                         f"{', 1-sample delay' if d0['delay'] else ''}) vs continuous (dashed)", fontsize=9)
            ax.legend(fontsize=8)
    f.axes[-1].set_xlabel("t [s]")
    return f


def hq_cap(cz, cat):
    f = new_fig(6.5, 4.6)
    ax = f.add_subplot(111)
    for lv, col in ((2, "#fff3cd"), (1, "#d4edda")):
        zmin, zmax = hq.SP_ZETA[lv][cat]
        cmin, cmax = hq.SP_CAP[lv][cat]
        ax.fill([zmin, zmax, zmax, zmin], [cmin, cmin, cmax, cmax], color=col, ec="0.4", lw=0.8,
                label=f"Level {lv}")
    ax.plot(cz["zeta"], cz["cap"], "b*", ms=16, label="SAS-augmented aircraft")
    ax.set_xscale("log"); ax.set_yscale("log")
    ax.set_xlim(0.1, 3); ax.set_ylim(0.01, 20)
    ax.set_xlabel("short-period damping ζsp"); ax.set_ylabel("CAP = ωn²/(n/α)  [1/(g s²)]")
    ax.set_title(f"CAP - ζ (Category {cat})"); ax.grid(True, which="both", alpha=0.3); ax.legend(fontsize=8)
    return f


def hq_bandwidth(bw, limits):
    f = new_fig(7.5, 6.2)
    a0 = f.add_subplot(211)
    l1, l2 = limits[1], limits[2]
    a0.fill([l1[0], 20, 20, l1[0]], [0, 0, l1[1], l1[1]], color="#d4edda", label="Level 1")
    a0.fill([l2[0], 20, 20, l2[0]], [l1[1], l1[1], l2[1], l2[1]], color="#fff3cd")
    a0.fill([l2[0], l1[0], l1[0], l2[0]], [0, 0, l2[1], l2[1]], color="#fff3cd", label="Level 2")
    if np.isfinite(bw["wbw"]):
        a0.plot(bw["wbw"], bw["tau_p"], "b*", ms=16, label="aircraft")
    a0.set_xlim(0, max(10, 1.2 * (bw["wbw"] if np.isfinite(bw["wbw"]) else 0)))
    a0.set_ylim(0, max(0.3, 1.2 * bw["tau_p"]))
    a0.set_xlabel("ωBWθ [rad/s]"); a0.set_ylabel("τp [s]")
    a0.set_title("Pitch attitude bandwidth / phase delay"); a0.grid(True, alpha=0.3); a0.legend(fontsize=8)
    a1 = f.add_subplot(212)
    a1.semilogx(bw["w"], bw["phase"], "b", lw=1.3, label="phase θ/δ [deg]")
    a1.axhline(-135, color="r", ls=":", lw=1); a1.axhline(-180, color="k", ls=":", lw=1)
    a1b = a1.twinx()
    a1b.semilogx(bw["w"], bw["mag"], "0.4", lw=1.0)
    a1b.set_ylabel("|θ/δ| [dB]", color="0.4")
    for k, c in (("wbw_phase", "r"), ("wbw_gain", "m"), ("w180", "k")):
        if np.isfinite(bw[k]):
            a1.axvline(bw[k], color=c, ls="--", lw=0.8)
    a1.set_xlabel("ω [rad/s]"); a1.set_ylabel("phase [deg]"); a1.grid(True, which="both", alpha=0.3)
    a1.legend(fontsize=8, loc="lower left")
    return f


def hq_gibson(gb):
    f = new_fig(7.5, 5.6)
    a1 = f.add_subplot(211)
    a1.plot(gb["t"], np.degrees(gb["q"]) / max(abs(np.degrees(gb["q_ss"])), 1e-12), "b", label="q / q_ss")
    a1.plot(gb["t"], gb["theta"] / max(abs(gb["q_ss"]), 1e-12), "g", label="θ / q_ss  [s]")
    a1.axvline(gb["hold"], color="k", ls=":", lw=1)
    a1.set_title("Gibson criterion: pilot step held, then released (short-period approximation)", fontsize=9)
    a1.set_xlabel("t [s]"); a1.grid(True, alpha=0.3); a1.legend(fontsize=8)
    a2 = f.add_subplot(212)
    a2.fill([0, 0.25, 0.25, 0], [1, 1, 3, 3], color="#d4edda", label="satisfactory (guidance)")
    a2.plot(gb["db_qss"], gb["qpk_qss"], "b*", ms=16, label="aircraft")
    a2.set_xlim(-0.5, max(1.0, 1.2 * (gb["db_qss"] if np.isfinite(gb["db_qss"]) else 0)))
    a2.set_ylim(0.8, max(4.0, 1.2 * (gb["qpk_qss"] if np.isfinite(gb["qpk_qss"]) else 0)))
    a2.set_xlabel("dropback DB/q_ss [s]"); a2.set_ylabel("q_pk / q_ss"); a2.grid(True, alpha=0.3)
    a2.legend(fontsize=8)
    return f


def hq_roll(rp):
    f = new_fig(7.5, 4.2)
    ax = f.add_subplot(111)
    ax.plot(rp["t"], rp["phi"], "b", lw=1.6, label=f"bank angle, aileron step {rp['da_deg']:g}°")
    ax.axhline(rp["req_bank"], color="k", ls="--", lw=1)
    ax.axvline(rp["req_time"], color="k", ls="--", lw=1)
    ax.plot(rp["req_time"], rp["req_bank"], "ro", ms=8,
            label=f"Level 1: {rp['req_bank']}° in {rp['req_time']} s")
    ax.set_xlabel("t [s]"); ax.set_ylabel("φ [deg]")
    ax.set_title("Roll performance (SAS-augmented, with actuators)")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    return f


def robust_poles(s, nominal_poles, title, region=None, lat_limits=None):
    """Cloud of closed-loop poles of every Monte Carlo sample over the nominal poles."""
    f = new_fig()
    ax = f.add_subplot(111)
    if region is not None:
        _sp_region(ax, region)
    P = np.array(s.poles)
    ax.plot(P.real, P.imag, ".", color="#1f77b4", ms=2, alpha=0.5, label="perturbed samples")
    nom = np.asarray(nominal_poles)
    ax.plot(nom.real, nom.imag, "kx", ms=10, mew=2, label="nominal")
    if lat_limits is not None:
        z = lat_limits["dr_zeta_min"]
        R = max(1.0, np.abs(P).max() if len(P) else 1.0)
        rr = np.linspace(0, R, 50)
        for sgn in (1, -1):
            ax.plot(-z * rr, sgn * rr * np.sqrt(1 - z * z), "r--", lw=1.2)
        ax.plot([], [], "r--", label=f"DR ζ min = {z}")
    ax.axvline(0, color="r", lw=1)
    ax.axhline(0, color="0.6", lw=0.6)
    ax.set_xlabel("Re [1/s]"); ax.set_ylabel("Im [rad/s]")
    ax.set_title(f"{title}: {100 * s.stable:.0f}% stable, {100 * s.level1:.0f}% all modes Level 1")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8, loc="best")
    return f


def robust_margins(res_ap, mode, specs):
    """PM vs DM of every sample, one colour per method, with the requirement box."""
    f = new_fig()
    ax = f.add_subplot(111)
    gm_s, pm_s, dm_s = specs
    for (m, meth), r in res_ap.items():
        if m != mode:
            continue
        ax.plot(r.dm, r.pm, ".", color=COLORS.get(meth), ms=4, alpha=0.7,
                label=f"{LABELS.get(meth, meth)} ({100 * r.meets:.0f}% meet specs)")
    ax.axvline(dm_s, color="k", ls="--", lw=1)
    ax.axhline(pm_s, color="k", ls="--", lw=1)
    ax.set_xscale("log")
    ax.set_xlabel("delay margin [s]"); ax.set_ylabel("phase margin [deg]")
    ax.set_title("Margins of the perturbed aircraft (dashed: requirements)")
    ax.grid(True, which="both", alpha=0.3); ax.legend(fontsize=8, loc="best")
    return f


def robust_spread(res_ap, mode, plant, settings):
    """Tracked response of the perturbed samples, one panel per method."""
    items = [(meth, r) for (m, meth), r in res_ap.items() if m == mode and r.ys]
    k = max(len(items), 1)
    f = new_fig(7.5, 2.4 * k + 0.6)
    for i, (meth, r) in enumerate(items):
        ax = f.add_subplot(k, 1, i + 1)
        for y in r.ys:
            ax.plot(r.t, y[:, 0], color=COLORS.get(meth), lw=0.6, alpha=0.35)
        ax.axhline(settings.ref[0], color="k", ls="--", lw=1)
        ax.set_ylabel(f"{plant.track_names[0]} [{plant.track_unit}]")
        ax.set_title(f"{LABELS.get(meth, meth)}: {len(r.ys)} perturbed samples", fontsize=9)
        ax.grid(True, alpha=0.3)
    if f.axes:
        f.axes[-1].set_xlabel("t [s]")
    return f


def schedule_plot(sres):
    nplots = 1 + (sres.Klat is not None) + (sres.Kap is not None)
    f = new_fig(7.5, 2.8 * nplots + 0.4)
    M = sres.mach
    order = np.argsort(M)
    Mf = np.linspace(M.min(), M.max(), 100) if len(M) > 1 else M
    i = 1
    ax = f.add_subplot(nplots, 1, i)
    for r in range(sres.Klong.shape[1]):
        for s in range(4):
            y = sres.Klong[order, r, s]
            if np.allclose(y, 0):
                continue
            ax.plot(M[order], y, "-o", ms=4, label=f"{sres.long_inputs[r]}: K_{sres.long_names[s]}")
    ax.set_title("Longitudinal SAS gains vs Mach"); ax.grid(True, alpha=0.3); ax.legend(fontsize=7, ncol=2)
    if sres.Klat is not None:
        i += 1
        ax = f.add_subplot(nplots, 1, i)
        for r in range(sres.Klat.shape[1]):
            for s in range(4):
                y = sres.Klat[order, r, s]
                if np.allclose(y, 0):
                    continue
                ax.plot(M[order], y, "-o", ms=4, label=f"{sres.lat_inputs[r]}: K_{sres.lat_names[s]}")
        ax.set_title("Lateral SAS gains vs Mach"); ax.grid(True, alpha=0.3); ax.legend(fontsize=7, ncol=2)
    if sres.Kap is not None:
        i += 1
        ax = f.add_subplot(nplots, 1, i)
        for s in range(sres.Kap.shape[1]):
            ax.plot(M[order], sres.Kap[order, s], "-o", ms=4, label=sres.ap_names[s] if sres.ap_names else str(s))
        ax.set_title("Autopilot LQI gains vs Mach"); ax.grid(True, alpha=0.3); ax.legend(fontsize=7, ncol=3)
    f.axes[-1].set_xlabel("Mach")
    return f
