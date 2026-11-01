"""Flight-controller export: designs that run on PX4 / ArduPilot fixed-wing autopilots.

The exported controllers replace the attitude + rate loops of the autopilot (the SAS and the
attitude-hold autopilot together). They only use what a typical fixed-wing autopilot measures:
body rates, Euler angles and airspeed - never alpha or beta.

Axes (PX4 / ArduPilot FRD body frame):
    pitch  - tracks theta with the elevator, measured [V (airspeed), q, theta]
    roll   - tracks phi with aileron + rudder, measured [p, r, phi]
Throttle stays with the autopilot's energy controller (TECS).

Design structures
    "PID"  - the firmware's own structure: attitude P (time constant) -> rate PI + feed-forward
             (+ yaw damper with turn coordination). Exported as parameters (.params / .param).
    "LQG"  - LQI gains + Kalman estimator on the sensors (output feedback).
    "Hinf" - H-infinity output feedback on the sensors.
    LQG and Hinf are exported as C code + a PX4 module (fw_ss_control).

Firmware output convention: normalized torque in [-1, 1], positive = positive body moment
(nose up, right wing down, nose right). With a surface of maximum deflection dmax and moment
derivative sign s:  delta = s * dmax * u_norm.
"""
from dataclasses import dataclass, field

import numpy as np

from . import autopilot as ap
from . import digital

DEG = np.pi / 180
G0 = 9.81
RHO0 = 1.225

LONG_STATES = ["V", "alpha", "q", "theta"]
LAT_STATES = ["beta", "p", "r", "phi"]
AXES = {"pitch": {"label": "Pitch (elevator)", "track": "theta", "ref": 5.0, "T": 8.0},
        "roll": {"label": "Roll + yaw (aileron, rudder)", "track": "phi", "ref": 20.0, "T": 8.0}}
STRUCTURES = {"PID": "Firmware PID structure (parameters)", "LQG": "LQG: LQI + Kalman on sensors (C code)",
              "Hinf": "H-infinity output feedback (C code)"}
FIRMWARES = ["PX4", "ArduPilot"]

# default sensor noise (1 sigma): gyro, attitude, airspeed
NOISE = {"V": 0.5, "p": 0.5 * DEG, "q": 0.5 * DEG, "r": 0.5 * DEG, "theta": 0.2 * DEG, "phi": 0.2 * DEG}
ARDU_WASHOUT = 0.2      # rad/s, high-pass of the ArduPlane yaw damper (AP_YawController)
ARDU_SCALE = np.pi / 4  # ArduPlane runs AC_PID in rad and maps 45 deg of notional deflection to full scale


class FCError(ValueError):
    pass


# ---------------------------------------------------------------------------
# models and plants
# ---------------------------------------------------------------------------
def long_va(model):
    """Longitudinal (A, B) in [V alpha q theta]. Body-axis [u w q theta] models are taken as
    stability axes (V ~ u, alpha ~ w / V0)."""
    if model.convention == "Vaqt":
        return model.A.copy(), model.B.copy()
    T = np.diag([1.0, 1.0 / model.V0, 1.0, 1.0])
    return T @ model.A @ np.linalg.inv(T), T @ model.B


def _input(names, role):
    j = ap._find_input(role, names)
    return j


def surface_sign(axis, B, j):
    """Sign of the body moment produced by a positive deflection of input j."""
    row = {"de": 2, "da": 1, "dr": 2}[axis]
    v = B[row, j]
    if abs(v) < 1e-12:
        raise FCError("The control surface has no effect on its moment equation (B entry is zero).")
    return 1.0 if v > 0 else -1.0


def build_plant(axis, long_model=None, lat_model=None, actuator=None, airspeed=True, noise=None):
    """APPlant without SAS: the exported controller provides all the feedback."""
    noise = {**NOISE, **(noise or {})}
    if axis == "pitch":
        if long_model is None:
            raise FCError("Enter the longitudinal model first.")
        A, B = long_va(long_model)
        names_in = list(long_model.input_names)
        js = [_input(names_in, "de")]
        states, units = list(LONG_STATES), ["m/s", "rad", "rad/s", "rad"]
        ks = [3]
        meas = [0, 2, 3] if airspeed else [2, 3]
        V0 = long_model.V0
    else:
        if lat_model is None:
            raise FCError("Enter the lateral model first.")
        A, B = lat_model.A.copy(), lat_model.B.copy()
        names_in = list(lat_model.input_names)
        js = [_input(names_in, "da"), _input(names_in, "dr")]
        if js[0] == js[1]:
            raise FCError("The lateral model needs separate aileron and rudder inputs.")
        states, units = list(LAT_STATES), ["rad", "rad/s", "rad/s", "rad"]
        ks = [3]
        meas = [1, 2, 3]
        V0 = lat_model.V0
    m = B.shape[1]
    Ksas = np.zeros((m, 4))
    in_scale = np.array([1 / DEG if ap.is_angle_input(nm) else 1.0 for nm in names_in])
    umax_hint = np.array([0.1 / max(np.abs(B[:, j]).max(), 1e-12) for j in range(m)])
    mnoise = np.array([noise[states[k]] for k in meas])

    def make(Aol, Bm, K, nms, uns, ideal=None):
        scale = np.array([1 / DEG if u in ("rad", "rad/s") else 1.0 for u in uns])
        return ap.APPlant(f"fc_{axis}", Aol - Bm @ K, Aol, Bm, K, js, ks, nms, names_in, scale, in_scale,
                          ["deg"], 3 if axis == "roll" else None, umax_hint, ideal, list(meas), mnoise)

    ideal = make(A, B, Ksas, states, units)
    ideal.V0 = V0
    if actuator is None or not actuator.active:
        return ideal
    Aol, Bm, K, nms, uns = ap.add_actuators(A, B, Ksas, list(states), list(units), names_in, actuator)
    full = make(Aol, Bm, K, nms, uns, ideal)
    full.V0 = V0
    return full


def settings_for(axis, plant, dmax, prefilter=0.0, ref=None, T=None, specs=None):
    """APSettings with the surface limits (rad) of the plant's inputs. specs = (GM dB, PM deg, DM s)."""
    info = AXES[axis]
    st = ap.APSettings(ref=[ref if ref is not None else info["ref"]], T=T or info["T"], prefilter=prefilter,
                       noise_std=[0.2])
    if specs is not None:
        st.gm_spec, st.pm_spec, st.dm_spec = (float(v) for v in specs)
    lim = np.full(plant.B.shape[1], np.inf)
    rates = np.full(plant.B.shape[1], np.inf)
    for j in plant.js:
        lim[j] = dmax[plant.input_names[j]]
        rates[j] = 200 * DEG
    st.limits, st.rates = lim, rates
    st.bank_limit = 0.0
    return st.ensure(plant)


# ---------------------------------------------------------------------------
# firmware PID structure
# ---------------------------------------------------------------------------
PID_KEYS = {"pitch": ["P", "I", "FF", "TC"], "roll": ["P", "I", "FF", "TC", "YAW_P"]}


def make_pid(plant, params, dmax, firmware="PX4"):
    """Firmware structure as a Controller in the common form (inputs [r; x]).
    params in normalized firmware units (per rad/s) and seconds."""
    n = plant.design.n
    if plant.mode == "fc_pitch":
        P, I, FF, TC = (float(params[k]) for k in PID_KEYS["pitch"])
        j = plant.js[0]
        k = surface_sign("de", plant.design.B, j) * dmax[plant.input_names[j]]
        iq, it = 2, 3
        Ac = np.zeros((1, 1))
        Bc = np.zeros((1, 1 + n)); Bc[0, 0] = 1 / TC; Bc[0, 1 + it] = -1 / TC; Bc[0, 1 + iq] = -1.0
        Cc = np.array([[k * I]])
        Dc = np.zeros((1, 1 + n))
        Dc[0, 0] = k * (P + FF) / TC; Dc[0, 1 + it] = -k * (P + FF) / TC; Dc[0, 1 + iq] = -k * P
        ints = [0]
    else:
        P, I, FF, TC, Py = (float(params[k]) for k in PID_KEYS["roll"])
        ja, jr = plant.js
        ka = surface_sign("da", plant.design.B, ja) * dmax[plant.input_names[ja]]
        kr = surface_sign("dr", plant.design.B, jr) * dmax[plant.input_names[jr]]
        ip, ir, iphi = 1, 2, 3
        c = G0 / plant.design.V0            # coordinated-turn yaw rate per rad of bank
        w = ARDU_WASHOUT if firmware == "ArduPilot" else 0.0
        nc = 2 if w > 0 else 1
        Ac = np.zeros((nc, nc))
        Bc = np.zeros((nc, 1 + n)); Cc = np.zeros((2, nc)); Dc = np.zeros((2, 1 + n))
        # roll: xi' = (r - phi)/TC - p ;  u_a = (P+FF)(r - phi)/TC - P p + I xi
        Bc[0, 0] = 1 / TC; Bc[0, 1 + iphi] = -1 / TC; Bc[0, 1 + ip] = -1.0
        Cc[0, 0] = ka * I
        Dc[0, 0] = ka * (P + FF) / TC; Dc[0, 1 + iphi] = -ka * (P + FF) / TC; Dc[0, 1 + ip] = -ka * P
        # yaw: e = c*phi - r ; u_r = Py * e (PX4) or Py * HP(e) (ArduPilot, HP = s/(s+w))
        if w > 0:
            Ac[1, 1] = -w
            Bc[1, 1 + iphi] = c; Bc[1, 1 + ir] = -1.0
            Cc[1, 1] = -kr * Py * w
        Dc[1, 1 + iphi] = kr * Py * c; Dc[1, 1 + ir] = -kr * Py
        ints = [0]
    keys = PID_KEYS["pitch" if plant.mode == "fc_pitch" else "roll"]
    prm = {kk: float(params[kk]) for kk in keys}
    c = ap.Controller("PID", Ac, Bc, Cc, Dc, prm, dict(prm), {"firmware": firmware}, int_states=ints)
    return ap._embed(c, plant)


TC_RANGE = {"PX4": (0.2, 1.0), "ArduPilot": (0.4, 1.0)}   # documented ranges of FW_*_TC / *2SRV_TCONST
MIN_DAMPING = 0.3         # of every oscillatory closed-loop mode faster than the phugoid
PID_WN_MIN = 0.5          # rad/s


def pid_search_space(plant, dmax, firmware):
    """Physical ranges of the PID search, from the moment each surface can produce at full deflection
    (a = |moment derivative| * dmax): rate-loop bandwidth P*a, integrator time P/I, feed-forward around
    its ideal value (rate damping / a), firmware range of the time constant."""
    Bd, Ad, names = plant.design.B, plant.design.A, plant.input_names
    if plant.mode == "fc_pitch":
        j = plant.js[0]
        row = 2
        ay = None
    else:
        j, jr = plant.js
        row = 1
        ay = abs(Bd[2, jr]) * dmax[names[jr]]
    a = abs(Bd[row, j]) * dmax[names[j]]
    ff_max = 2.0 * max(abs(Ad[row, row]), 1.0) / a
    # z = [log10 P, log10 Ti, FF, TC, (log10 YAW_P)]
    bounds = [(np.log10(1.0 / a), np.log10(100.0 / a)), (np.log10(0.2), np.log10(5.0)), (0.0, ff_max),
              TC_RANGE[firmware]]
    if ay is not None:
        bounds.append((np.log10(1.0 / ay), np.log10(30.0 / ay)))
    return bounds


def _pid_from_z(z, axis):
    P = 10 ** z[0]
    prm = {"P": P, "I": P / 10 ** z[1], "FF": float(z[2]), "TC": float(z[3])}
    if axis == "roll":
        prm["YAW_P"] = 10 ** z[4]
    return {k: float(v) for k, v in prm.items()}


def _min_damping(poles):
    osc = poles[(poles.imag > 1e-6) & (np.abs(poles) > PID_WN_MIN)]
    return float(np.min(-osc.real / np.abs(osc))) if len(osc) else 1.0


def tune_pid(plant, settings, dmax, firmware="PX4"):
    import copy

    import scipy.optimize
    axis = "pitch" if plant.mode == "fc_pitch" else "roll"
    bounds = pid_search_space(plant, dmax, firmware)
    best = {"J": np.inf}
    # tracking alone rewards feed-forward-only designs (open-loop rate commands): also ask the
    # loop to bring back an attitude offset, which only the feedback terms can do
    reg = copy.deepcopy(settings)
    reg.ref = np.zeros_like(settings.ref)
    reg.x0_track = np.asarray(settings.ref, float) / 2

    def f(z):
        c = make_pid(plant, _pid_from_z(z, axis), dmax, firmware)
        J, res, _ = ap._cost(plant, c, settings)
        if J < 1e5:
            J += ap._cost(plant, c, reg, margins_too=False)[0]
            # the attitude step barely excites the dutch roll: require damping explicitly, so the
            # yaw damper has a purpose in the search
            J += 10.0 * max(0.0, MIN_DAMPING - _min_damping(res.poles))
        if J < best["J"]:
            best.update(J=J, z=np.array(z, float))
        return J

    scipy.optimize.differential_evolution(f, bounds, maxiter=22, popsize=7, seed=2, tol=1e-4,
                                          polish=False, init="sobol")
    scipy.optimize.minimize(f, best["z"], method="Nelder-Mead", bounds=bounds,
                            options={"maxiter": 160, "xatol": 1e-3, "fatol": 1e-5})
    c = make_pid(plant, _pid_from_z(best["z"], axis), dmax, firmware)
    c.info["cost"] = best["J"]
    return c


# ---------------------------------------------------------------------------
# design entry point
# ---------------------------------------------------------------------------
@dataclass
class FCDesign:
    axis: str
    structure: str
    firmware: str
    plant: object
    settings: object
    ctrl: object
    dmax: dict
    result: object = None
    digital: dict = None
    Ts: float = 0.01
    extra: dict = field(default_factory=dict)


def design_actuator(actuator, Ts):
    """Actuator model used for the design: the servo lag / delay of the aircraft plus the delay of
    the digital implementation (zero-order hold Ts/2 + one sample of computation = 1.5 Ts), so that
    every design keeps its margins once it runs in the autopilot."""
    tau = actuator.tau if actuator is not None else 0.0
    delay = (actuator.delay if actuator is not None else 0.0) + 1.5 * Ts
    return ap.ActuatorSpec(tau, delay)


def design(axis, structure, long_model, lat_model, dmax, firmware="PX4", actuator=None, airspeed=True,
           Ts=0.01, prefilter=None, noise=None, specs=None, ref=None):
    plant = build_plant(axis, long_model, lat_model, design_actuator(actuator, Ts), airspeed, noise)
    if prefilter is None:
        prefilter = 0.0 if structure == "PID" else 2.0
    st = settings_for(axis, plant, dmax, prefilter, ref=ref, specs=specs)
    if structure == "PID":
        c = tune_pid(plant, st, dmax, firmware)
    elif structure == "LQG":
        lqi = ap.tune_lqi(plant, st)
        c = ap.tune_lqg(plant, st, lqi)
        c.method = "LQG"
    elif structure == "Hinf":
        c = ap.tune_hinf(plant, st)
    else:
        raise FCError(f"Unknown structure {structure}")
    if structure != "PID":        # the firmware loops have no reference scaling: they rely on the integrator
        ap.fix_dc(plant, c, st.prefilter)
    check_measured(plant, c)
    res = ap.evaluate(plant, c, st)
    dg = digital.digital_check(plant, c, st, Ts, "bilinear", True)
    return FCDesign(axis, structure, firmware, plant, st, c, dict(dmax), res, dg, Ts)


def check_measured(plant, ctrl, tol=1e-9):
    """The exported controller may only read the sensors."""
    p = plant.p
    used = np.abs(ctrl.Bc[:, p:]).sum(axis=0) + np.abs(ctrl.Dc[:, p:]).sum(axis=0)
    bad = [plant.state_names[k] for k in range(plant.n) if used[k] > tol * max(1.0, used.max())
           and k not in plant.meas]
    if bad:
        raise FCError(f"Controller uses unmeasured states: {', '.join(bad)}")


# ---------------------------------------------------------------------------
# firmware parameters
# ---------------------------------------------------------------------------
def cas(V_tas, h):
    """Calibrated (= equivalent, low speed) airspeed from TAS at altitude h (ISA)."""
    from .aero_lin import isa
    rho = isa(h)[0]
    return float(V_tas * np.sqrt(rho / RHO0))


def pid_parameters(designs, firmware, V_tas, h):
    """Firmware parameters from the PID-structure designs {'pitch': FCDesign, 'roll': FCDesign}."""
    out = []   # (name, value, comment)
    V_cas = cas(V_tas, h)
    pd = designs.get("pitch")
    rd = designs.get("roll")
    if firmware == "PX4":
        if pd is not None:
            p = pd.ctrl.params
            out += [("FW_P_TC", p["TC"], "pitch attitude time constant [s]"),
                    ("FW_PR_P", p["P"], "pitch rate P [normalized torque per rad/s]"),
                    ("FW_PR_I", p["I"], "pitch rate I"),
                    ("FW_PR_D", 0.0, "pitch rate D (not used)"),
                    ("FW_PR_FF", p["FF"], "pitch rate feed-forward")]
        if rd is not None:
            p = rd.ctrl.params
            out += [("FW_R_TC", p["TC"], "roll attitude time constant [s]"),
                    ("FW_RR_P", p["P"], "roll rate P"),
                    ("FW_RR_I", p["I"], "roll rate I"),
                    ("FW_RR_D", 0.0, "roll rate D (not used)"),
                    ("FW_RR_FF", p["FF"], "roll rate feed-forward"),
                    ("FW_YR_P", p["YAW_P"], "yaw rate P (turn-coordinated yaw damper)"),
                    ("FW_YR_I", 0.0, "yaw rate I"),
                    ("FW_YR_FF", 0.0, "yaw rate feed-forward")]
        out += [("FW_AIRSPD_TRIM", V_cas, "design airspeed (calibrated) [m/s]"),
                ("FW_ARSP_SCALE_EN", 1, "scale gains with airspeed around FW_AIRSPD_TRIM")]
    else:
        s = ARDU_SCALE
        if pd is not None:
            p = pd.ctrl.params
            out += [("PTCH2SRV_TCONST", p["TC"], "pitch time constant [s]"),
                    ("PTCH_RATE_P", p["P"] * s, "pitch rate P"),
                    ("PTCH_RATE_I", p["I"] * s, "pitch rate I"),
                    ("PTCH_RATE_D", 0.0, "pitch rate D (not used)"),
                    ("PTCH_RATE_FF", p["FF"] * s, "pitch rate feed-forward")]
        if rd is not None:
            p = rd.ctrl.params
            out += [("RLL2SRV_TCONST", p["TC"], "roll time constant [s]"),
                    ("RLL_RATE_P", p["P"] * s, "roll rate P"),
                    ("RLL_RATE_I", p["I"] * s, "roll rate I"),
                    ("RLL_RATE_D", 0.0, "roll rate D (not used)"),
                    ("RLL_RATE_FF", p["FF"] * s, "roll rate feed-forward"),
                    ("YAW2SRV_DAMP", p["YAW_P"] * s, "yaw damper (washout 0.2 rad/s)"),
                    ("YAW2SRV_RLL", 1.0, "turn coordination gain"),
                    ("YAW2SRV_INT", 0.0, "sideslip integrator (not used)"),
                    ("YAW2SRV_SLIP", 0.0, "sideslip gain (not used)")]
        out += [("SCALING_SPEED", V_cas, "speed at which the gains apply (EAS) [m/s]")]
    return out


def write_params(path, params, firmware, header_lines):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        for h in header_lines:
            f.write(f"# {h}\n")
        if firmware == "PX4":
            f.write("# QGroundControl parameter file: vehicle-id component-id name value type\n")
            for name, val, _ in params:
                typ = 6 if isinstance(val, int) else 9
                f.write(f"1\t1\t{name}\t{val:.6g}\t{typ}\n" if typ == 9 else f"1\t1\t{name}\t{val}\t{typ}\n")
        else:
            for name, val, cmt in params:
                f.write(f"{name},{val:.6g}\n")


# ---------------------------------------------------------------------------
# discrete controller for code generation
# ---------------------------------------------------------------------------
@dataclass
class DiscreteController:
    name: str
    Ts: float
    A: np.ndarray           # nc x nc
    Br: np.ndarray          # nc x 1 (prefiltered reference, Nbar included)
    By: np.ndarray          # nc x ny
    C: np.ndarray           # nu x nc   (normalized torque)
    Dr: np.ndarray          # nu x 1
    Dy: np.ndarray          # nu x ny
    a_pf: float             # prefilter pole: rf+ = a rf + (1-a) r  (a = 0: no prefilter)
    int_idx: list           # integrator states (anti-windup)
    y_names: list
    u_names: list
    r_name: str
    y_offsets: list         # subtracted from the raw measurements (trim values)
    r_offset: float
    trim: list = None       # normalized torque added to the outputs (trim deflection)


def to_discrete(d: FCDesign, theta0=0.0, trim_deflection=None):
    """Tustin controller with normalized-torque outputs, inputs = sensors only.
    trim_deflection: {input name: trim deflection [rad]} added as a constant torque."""
    plant, c, Ts = d.plant, d.ctrl, d.Ts
    p = plant.p
    Acd, Bcd, Ccd, Dcd = digital.discretize(c.Ac, c.Bc, c.Cc, c.Dc, Ts, "bilinear")
    Nb = float(ap._nbar(c, p)[0, 0])
    cols = [p + k for k in plant.meas]
    Br = Bcd[:, :p] * Nb
    Dr = Dcd[:, :p] * Nb
    By, Dy = Bcd[:, cols], Dcd[:, cols]
    # outputs: physical deflection [rad] -> normalized torque
    scale = []
    for j in plant.js:
        role = {"fc_pitch": ["de"], "fc_roll": ["da", "dr"]}[plant.mode][plant.js.index(j)]
        s = surface_sign(role, plant.design.B, j)
        scale.append(s / d.dmax[plant.input_names[j]])
    S = np.diag(scale)
    pf = d.settings.prefilter
    a = float(np.exp(-pf * Ts)) if pf > 0 else 0.0
    y_names = [plant.state_names[k] for k in plant.meas]
    offs = []
    for nm in y_names:
        offs.append(plant.design.V0 if nm == "V" else (theta0 if nm == "theta" else 0.0))
    u_names = ["torque_y (pitch)"] if plant.mode == "fc_pitch" else ["torque_x (roll)", "torque_z (yaw)"]
    trim = [scale[i] * float((trim_deflection or {}).get(plant.input_names[j], 0.0))
            for i, j in enumerate(plant.js)]
    return DiscreteController(d.axis, Ts, Acd, Br, By, S @ Ccd, S @ Dr, S @ Dy, a, list(c.int_states),
                              y_names, u_names, "theta_sp" if d.axis == "pitch" else "phi_sp", offs,
                              theta0 if d.axis == "pitch" else 0.0, trim)


def simulate_discrete(dc: DiscreteController, r, Y, umax=1.0, bumpless=True):
    """Reference implementation of the generated C step function (same order of operations).
    r: (N,) raw setpoints; Y: (N, ny) raw measurements. Returns (N, nu) outputs.
    bumpless=False starts the prefilter at 0 instead of at the first setpoint (as the
    application's step responses do)."""
    x = np.zeros(dc.A.shape[0]); rf = 0.0
    out = np.zeros((len(r), dc.C.shape[0]))
    yo = np.asarray(dc.y_offsets)
    trim = np.asarray(dc.trim if dc.trim is not None else np.zeros(dc.C.shape[0]))
    first = bumpless
    for k in range(len(r)):
        rk = r[k] - dc.r_offset
        if first:
            rf = rk if dc.a_pf > 0 else 0.0
            first = False
        rin = rf if dc.a_pf > 0 else rk
        y = Y[k] - yo
        u = dc.C @ x + dc.Dr[:, 0] * rin + dc.Dy @ y + trim
        us = np.clip(u, -umax, umax)
        sat = bool(np.any(np.abs(u) > umax))
        xn = dc.A @ x + dc.Br[:, 0] * rin + dc.By @ y
        if sat:
            for i in dc.int_idx:
                xn[i] = x[i]
        x = xn
        if dc.a_pf > 0:
            rf = dc.a_pf * rf + (1 - dc.a_pf) * rk
        out[k] = us
    return out
