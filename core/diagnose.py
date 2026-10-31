"""Plain-language diagnostics: why a design misses a requirement and which knob to turn.

Each function returns a list of (severity, title, explanation, suggestion) with severity in
{"ok", "info", "caution", "warning"}."""
import numpy as np

from . import autopilot as ap
from . import hq

R2D = 180 / np.pi

# per-method knobs that lower / raise the loop bandwidth
SLOWER = {
    "PIDF": "reduce Kp and Kd (or increase Tf)",
    "LQI": "increase R (control weight) or reduce the integrator weight in Q",
    "LQG": "increase R, or reduce Qn so the Kalman filter is less aggressive",
    "Hinf": "increase W2 (control weight) or nu (measurement-noise level)",
    "LoopShaping": "reduce wc (target crossover)",
    "Cascade": "reduce Kp_psi, or increase R_inner (slower roll loop)",
}
FASTER = {
    "PIDF": "increase Kp / Ki",
    "LQI": "decrease R or increase the integrator weight in Q",
    "LQG": "decrease R (LQI part)",
    "Hinf": "decrease W2 or increase W4 (integrator weight)",
    "LoopShaping": "increase wc",
    "Cascade": "increase Kp_psi",
}
INTEGRAL = {
    "PIDF": "Ki", "LQI": "the integrator entries of Q", "LQG": "the integrator entries of Q",
    "Hinf": "W4", "LoopShaping": "rho (W1 zero)", "Cascade": "Ki_psi",
}


def _f(x, d=3):
    return "n/a" if x is None or not np.isfinite(x) else f"{x:.{d}g}"


def slow_zeros(plant, limit=0.05):
    """Transmission zeros of tracked output / AP input near the origin (they make SISO
    output feedback converge very slowly, typically in theta loops of large transports)."""
    import scipy.linalg
    n = plant.n
    A, b, c = plant.A, plant.b, plant.c
    M = np.block([[A, b], [c, np.zeros((1, 1))]])
    N = np.zeros_like(M); N[:n, :n] = np.eye(n)
    try:
        z = scipy.linalg.eigvals(M, N)
    except Exception:  # noqa: BLE001
        return []
    z = z[np.isfinite(z)]
    return [complex(x) for x in z if abs(x) < limit and abs(x) > 1e-9]


def autopilot(entry):
    plant, st = entry["plant"], entry["settings"]
    out = []
    base = ap._baseline_peak(plant, st)
    lim_disp = st.limits * plant.input_scale
    act = plant.ideal is not None
    zeros = slow_zeros(plant) if plant.q == 1 else []
    for m, r in entry["results"].items():
        if r is None:
            continue
        lbl = ap.METHOD_LABELS.get(m, m)
        f = r.feasible
        if not f["stable"]:
            out.append(("warning", f"{lbl}: closed loop unstable",
                        "At least one closed-loop pole is in the right half plane"
                        + (" (the design model has ideal actuators; the actuator lag/delay of the "
                           "evaluation model destabilises it)" if act else "") + ".",
                        f"Lower the bandwidth: {SLOWER.get(m, '')}."))
            continue
        mg = r.margins
        wc = mg.get("wcp", np.nan)
        if not f["dm"]:
            msg = (f"Delay margin {_f(mg['dm'])} s < {st.dm_spec} s. DM = PM/wc, and the gain crossover "
                   f"wc = {_f(wc)} rad/s is high")
            if act:
                msg += "; the actuator lag/delay already uses part of the phase"
            out.append(("caution", f"{lbl}: delay margin", msg + ".",
                        f"Lower the crossover: {SLOWER.get(m, '')}. A slower prefilter does NOT help "
                        "(margins are a feedback property)."))
        if not f["pm"]:
            out.append(("caution", f"{lbl}: phase margin",
                        f"PM {_f(mg['pm'])} deg < {st.pm_spec} deg at wc = {_f(wc)} rad/s.",
                        f"Add damping / lower the crossover: {SLOWER.get(m, '')}"
                        + (" or add derivative action (Kd)" if m == "PIDF" else "") + "."))
        if not f["gm"]:
            out.append(("caution", f"{lbl}: gain margin",
                        f"GM {_f(mg['gm_db'])} dB is within +/-{st.gm_spec} dB of instability "
                        f"(phase crossover at {_f(mg.get('wcg', np.nan))} rad/s).",
                        f"Reduce the loop gain near that frequency: {SLOWER.get(m, '')}."))
        if not f["effort"]:
            for i, nm in enumerate(plant.input_names):
                if r.peak_effort[i] <= lim_disp[i] * 1.005:
                    continue
                if base[i] >= lim_disp[i]:
                    out.append(("warning", f"{lbl}: {nm} over its limit",
                                f"Peak {nm} {_f(r.peak_effort[i])} > {_f(lim_disp[i])}, but the SAS alone already "
                                f"needs {_f(base[i])} for this manoeuvre - the autopilot cannot fix it.",
                                "Soften the SAS (lower dutch-roll / short-period target frequency, a milder "
                                "flight-phase category) or reduce the manoeuvre amplitude."))
                else:
                    out.append(("caution", f"{lbl}: {nm} over its limit",
                                f"Peak {nm} {_f(r.peak_effort[i])} > limit {_f(lim_disp[i])}.",
                                f"Make the response gentler: lower the prefilter pole, a smaller reference step, "
                                f"or {SLOWER.get(m, '')}."))
        if "bank" in f and not f["bank"]:
            out.append(("caution", f"{lbl}: bank angle",
                        f"Peak bank {_f(r.peak_bank)} deg exceeds {st.bank_limit} deg.",
                        f"Slow the heading loop ({SLOWER.get(m, '')}) or use the cascade structure, "
                        "which commands phi explicitly."))
        for i, mt in enumerate(r.metrics):
            if not mt:
                continue
            ref = st.ref[i]
            sse = mt.get("sse", np.nan)
            amp = abs(ref) if abs(ref) > 0 else max(abs(st.x0_track[i]), 1.0)
            tn = plant.track_names[i]
            if np.isfinite(sse) and abs(sse) > 0.02 * amp:
                txt = (f"{tn} is still {_f(abs(sse))} {plant.track_units[i]} away from the target at "
                       f"t = {st.T:g} s.")
                sug = f"Increase the simulation time, or strengthen integral action ({INTEGRAL.get(m, '')})."
                if zeros and m in ("PIDF", "LoopShaping"):
                    txt += (" The tracked channel has a slow transmission zero at s = "
                            + ", ".join(f"{z.real:.4f}" for z in zeros)
                            + "; feedback from the tracked output alone cannot move it, so the last part "
                            "of the response is very slow.")
                    sug = "Prefer LQI / LQG / H-inf (full-state feedback), or accept the slow tail."
                out.append(("caution", f"{lbl}: {tn} not settled", txt, sug))
            os_ = mt.get("overshoot", np.nan)
            if np.isfinite(os_) and os_ > 20:
                out.append(("caution", f"{lbl}: overshoot {_f(os_)} %",
                            "Large overshoot usually comes from too much integral action or low damping.",
                            f"Reduce {INTEGRAL.get(m, 'the integral gain')} or lower the prefilter pole."))
        sat = (entry.get("sat") or {}).get(m)
        if sat is not None:
            if sat["diverged"]:
                out.append(("warning", f"{lbl}: diverges with actuator limits",
                            "With saturation and rate limits the loop is not recovered.",
                            "Enable anti-windup, reduce the gains, or reduce the manoeuvre amplitude."))
            elif sat.get("noaw") is not None and sat["noaw"]["diverged"]:
                out.append(("info", f"{lbl}: anti-windup needed",
                            "Without anti-windup the saturated loop diverges; with it the response is fine.",
                            "Keep anti-windup in the implementation."))
        dg = (entry.get("digital") or {}).get(m)
        if dg is not None:
            if not dg["stable"]:
                out.append(("warning", f"{lbl}: unstable when implemented digitally",
                            f"At Ts = {dg['Ts']*1000:g} ms ({dg['method']}) the sampled loop is unstable.",
                            "Increase the sample rate (use 'Suggest sample rate'), or lower the bandwidth."))
            else:
                loss = mg["pm"] - dg["margins"]["pm"]
                if np.isfinite(loss) and loss > 10:
                    out.append(("caution", f"{lbl}: digital phase loss {_f(loss)} deg",
                                "The sample-and-hold and the computation delay cost ~ wc*Ts*(0.5+1) rad of phase.",
                                "Increase the sample rate."))
        if not any(t[1].startswith(lbl) for t in out):
            out.append(("ok", f"{lbl}: all requirements met",
                        f"GM {_f(mg['gm_db'])} dB, PM {_f(mg['pm'])} deg, DM {_f(mg['dm'])} s.", ""))
    for m, e in (entry.get("errors") or {}).items():
        out.append(("warning", f"{ap.METHOD_LABELS.get(m, m)}: failed", e,
                    "Check controllability of the selected input and the actuator limits."))
    return out


def sas(project):
    out = []
    p = project
    if p.long_sas is not None:
        r, m = p.long_sas, p.long_model
        sp, ph = r.modes_cl["sp"], r.modes_cl["ph"]
        L = r.sp_limits
        if r.levels_cl["sp"] != 1:
            out.append(("caution", "Short period not Level 1",
                        f"wn = {_f(sp.wn)} rad/s, zeta = {_f(sp.zeta)}; Level 1 needs wn in "
                        f"[{_f(L['wn'][0])}, {_f(L['wn'][1])}] and zeta in [{L['zeta'][0]}, {L['zeta'][1]}].",
                        "Use 'Auto targets', or move the short-period target inside that band."))
        if r.levels_cl["ph"] != 1:
            out.append(("caution", "Phugoid not Level 1", f"zeta_ph = {_f(ph.zeta)} (needs >= 0.04).",
                        "Raise the phugoid damping target (full-state structure)."))
        big = [(nm, e * R2D) for nm, e in zip(m.input_names, r.effort_alpha5) if abs(e * R2D) > 30]
        for nm, e in big:
            out.append(("caution", f"Longitudinal SAS effort on {nm}",
                        f"{_f(e)} deg for a 5 deg alpha disturbance.",
                        "Lower the short-period target frequency (stay in Level 1)."))
        if p.cstar_checks and p.cstar_checks.get("cstar") is not None and p.cstar_checks["cstar"] < 0.9:
            out.append(("info", "C* partly outside its envelope",
                        f"{100 * p.cstar_checks['cstar']:.0f}% of the C* samples are inside the band.",
                        "C* responds to the short-period frequency/damping and to the pitch-attitude feedback; "
                        "a 'Short period only' structure keeps the classic C* shape."))
    if p.lat_sas is not None:
        r, m = p.lat_sas, p.lat_model
        t = hq.lat_level1_targets(p.cat, p.cls)
        if np.any(r.poles.real >= 0):
            out.append(("warning", "Lateral SAS unstable",
                        "A closed-loop lateral pole is in the right half plane.",
                        "Check the targets (spiral pole must be negative) and the washout time constant."))
        lv = r.levels_cl
        if lv.get("dr", 1) != 1:
            dr = r.modes_cl["dr"]
            out.append(("caution", "Dutch roll not Level 1",
                        f"wn = {_f(dr.wn)}, zeta = {_f(dr.zeta)}; Level 1 needs zeta >= {t['dr_zeta_min']}, "
                        f"zeta*wn >= {t['dr_zw_min']}, wn >= {t['dr_wn_min']}.",
                        "Raise the dutch-roll damping / frequency target."))
        if lv.get("roll", 1) != 1:
            out.append(("caution", "Roll mode not Level 1",
                        f"Roll time constant must be <= {t['roll_tau_max']} s.",
                        f"Move the roll pole left of -1/{t['roll_tau_max']} = {-1 / t['roll_tau_max']:.2f}."))
        if lv.get("spiral", 1) != 1:
            out.append(("caution", "Spiral mode not Level 1",
                        f"Spiral time to double must be >= {t['spiral_t2_min']} s.",
                        "Make the spiral target slightly negative (e.g. -0.02)."))
        for nm, e in zip(m.input_names, r.effort_beta5):
            if abs(e * R2D) > 30:
                out.append(("caution", f"Lateral SAS effort on {nm}",
                            f"{_f(e * R2D)} deg for a 5 deg sideslip - beyond a typical 30 deg limit.",
                            "Lower the dutch-roll target frequency (it is often set by the Level 1 minimum "
                            "wn of category A / class IV), or check the control derivative of this surface."))
    if not out:
        out.append(("ok", "SAS: all modes Level 1", "Short period, phugoid, dutch roll, roll and spiral meet "
                    "MIL-F-8785C Level 1 with acceptable effort.", ""))
    return out


def robustness(project):
    res = project.robust
    out = []
    if not res:
        return out
    pct = f"±{100 * res['frac']:.0f}%"
    for ax_, s in res["sas"].items():
        name = "Longitudinal SAS" if ax_ == "long" else "Lateral SAS"
        if s.stable < 1:
            sug = "Move the targets further from the imaginary axis (more damping, faster roll/spiral)."
            if ax_ == "lat" and project.lat_sas is not None and project.lat_sas.targets.spiral_pole > -0.05:
                sug = (f"The spiral target ({project.lat_sas.targets.spiral_pole:g}) is close to the imaginary axis; "
                       "a more negative spiral pole (e.g. -0.05 ... -0.1) tolerates larger derivative errors.")
            out.append(("warning", f"{name}: unstable for {100 * (1 - s.stable):.0f}% of the samples",
                        f"With {pct} errors on the derivatives some closed-loop poles cross into the right half plane "
                        f"(worst Re = {s.worst_real:.3g}).", sug))
        elif s.level1 < 0.9:
            out.append(("caution", f"{name}: Level 1 kept in only {100 * s.level1:.0f}% of the samples",
                        "The nominal design sits close to a Level 1 boundary.",
                        "Choose targets nearer the middle of the Level 1 region (untick 'Minimise control effort' "
                        "or raise the damping targets)."))
        else:
            out.append(("ok", f"{name}: robust to {pct}", f"{100 * s.stable:.0f}% stable, "
                        f"{100 * s.level1:.0f}% Level 1.", ""))
    for (m, meth), r in res["ap"].items():
        lbl = f"{ap.MODES[m]['label']} / {ap.METHOD_LABELS.get(meth, meth)}"
        if r.stable < 1:
            out.append(("warning", f"{lbl}: unstable for {100 * (1 - r.stable):.0f}% of the samples", "",
                        f"Lower the loop bandwidth ({SLOWER.get(meth, '')}) or prefer LQG / H-inf."))
        elif r.meets < 0.9:
            worst = []
            dm = np.array(r.dm, float); pm = np.array(r.pm, float)
            if np.nanmin(dm) < project.ap[m]["settings"].dm_spec:
                worst.append(f"DM down to {np.nanmin(dm):.3g} s")
            if np.nanmin(pm) < project.ap[m]["settings"].pm_spec:
                worst.append(f"PM down to {np.nanmin(pm):.3g} deg")
            out.append(("caution", f"{lbl}: requirements met in {100 * r.meets:.0f}% of the samples",
                        ", ".join(worst) + ("." if worst else "Effort or tracking limits are exceeded in some samples."),
                        f"Add margin on the nominal design: {SLOWER.get(meth, '')}."))
    for (m, meth), d in res["disk"].items():
        a, dg, dp = d["multi"]
        if dg < 6 or dp < 35:
            out.append(("caution", f"{ap.MODES[m]['label']} / {ap.METHOD_LABELS.get(meth, meth)}: small disk margin",
                        f"Simultaneous gain/phase variations: only ±{dg:.1f} dB / {dp:.1f} deg (disk margins are "
                        "stricter than classical margins).", f"{SLOWER.get(meth, '')}."))
    return out


def handling(project):
    out = []
    h = project.hq1797 or {}
    bw = h.get("bw")
    if bw is not None:
        lim = h["bw_limits"]
        if np.isfinite(bw["wbw"]) and bw["wbw"] < lim[1][0]:
            out.append(("caution", "Pitch bandwidth below Level 1",
                        f"wBW = {_f(bw['wbw'])} rad/s < {lim[1][0]} rad/s.",
                        "Raise the short-period frequency target, or reduce actuator lag/delay."))
        if bw["tau_p"] > lim[1][1]:
            out.append(("caution", "Phase delay above Level 1",
                        f"tau_p = {_f(bw['tau_p'])} s > {lim[1][1]} s; it comes from actuator lag, time delay and "
                        "high-order filters.", "Reduce the actuator time constant / delay in the Model tab."))
        if bw["pio_prone"]:
            out.append(("caution", "Gain-limited bandwidth (PIO tendency)",
                        "The +6 dB gain bandwidth is lower than the -135 deg phase bandwidth: the attitude "
                        "response is sensitive to pilot gain.", "Increase short-period damping or reduce delay."))
    gb = h.get("gibson")
    if gb is not None and not gb["ok"]:
        out.append(("caution", "Gibson dropback outside the guidance box",
                    f"DB/q_ss = {_f(gb['db_qss'])} s, q_pk/q_ss = {_f(gb['qpk_qss'])}. For the short-period "
                    "approximation DB/q_ss = T_theta2 - 2 zeta/wn.",
                    "Large dropback: lower wn or raise zeta of the short period (or reduce T_theta2). "
                    "Large q_pk/q_ss: raise the short-period damping."))
    rp = h.get("roll")
    if rp is not None and not rp["ok"]:
        txt = (f"{_f(rp['t_reach'])} s to {rp['req_bank']} deg (Level 1: {rp['req_time']} s) with a "
               f"{rp['da_deg']:g} deg aileron step.")
        sug = "Use the full aileron authority, or a faster roll-mode target."
        if project.lat_sas is not None and abs(project.lat_sas.K[0, 3]) > 1e-6:
            sug += (" Note: the lateral SAS feeds back phi (attitude-command), so a held aileron "
                    "command gives a bank angle, not a roll rate.")
        out.append(("caution", "Roll performance not met", txt, sug))
    if h and not out:
        out.append(("ok", "Handling qualities: all checks passed", "", ""))
    return out
