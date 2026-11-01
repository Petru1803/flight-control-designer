"""Autopilot (outer-loop) design on top of the SAS-augmented aircraft.

Every autopilot mode is reduced to the same tracking problem (SISO or MIMO):
    x' = A x + B_ap u        (A already contains the SAS and, optionally, the actuators)
    y  = x[ks]               (tracked states: theta, h, V/u, phi, psi, beta, ...)
    delta_all = E u - Ksas x (total surface commands, used for effort)

Every controller is stored in one common state-space form
    xc' = Ac xc + Bc [r_f; x_meas]
    u   = Cc xc + Dc [r_f; x_meas]
where r_f (p) is the prefiltered reference and x_meas = x + E_k*noise.
"""
from dataclasses import dataclass, field

import numpy as np
import scipy.linalg

from . import cancel, lti
from .sas import lat_extended, long_extended

DEG = np.pi / 180

MODES = {
    "theta":   {"label": "Pitch attitude hold (theta)", "axis": "long", "tracks": ["theta"], "inputs": ["de"],
                "ref": [5.0], "T": 20.0, "units": ["deg"]},
    "alt":     {"label": "Altitude hold (h)", "axis": "long", "tracks": ["h"], "inputs": ["de"],
                "ref": [30.0], "T": 60.0, "units": ["m"]},
    "speed":   {"label": "Speed hold (u / V)", "axis": "long", "tracks": ["speed"], "inputs": ["dT"],
                "ref": [5.0], "T": 60.0, "units": ["m/s"]},
    "alt_speed": {"label": "Altitude hold + autothrottle (h, V)  [MIMO]", "axis": "long",
                  "tracks": ["h", "speed"], "inputs": ["de", "dT"], "ref": [30.0, 0.0], "T": 60.0,
                  "units": ["m", "m/s"]},
    "roll":    {"label": "Roll attitude hold (phi)", "axis": "lat", "tracks": ["phi"], "inputs": ["da"],
                "ref": [10.0], "T": 15.0, "units": ["deg"]},
    "heading": {"label": "Heading hold (psi)", "axis": "lat", "tracks": ["psi"], "inputs": ["da"],
                "ref": [10.0], "T": 60.0, "units": ["deg"]},
    "beta":    {"label": "Turn coordination (beta -> 0)", "axis": "lat", "tracks": ["beta"], "inputs": ["dr"],
                "ref": [0.0], "T": 20.0, "units": ["deg"], "x0": [5.0]},
    "heading_beta": {"label": "Heading hold + turn coordination (psi, beta)  [MIMO]", "axis": "lat",
                     "tracks": ["psi", "beta"], "inputs": ["da", "dr"], "ref": [10.0, 0.0], "T": 60.0,
                     "units": ["deg", "deg"]},
}
METHODS = ["PIDF", "LQI", "LQG", "Hinf", "LoopShaping", "Cascade"]
METHOD_LABELS = {"PIDF": "PIDF", "LQI": "LQI", "LQG": "LQG (LQI + Kalman)",
                 "Hinf": "H-infinity (hinfsyn)", "LoopShaping": "H-inf loop shaping (Glover-McFarlane)",
                 "Cascade": "Cascade PI(psi) -> LQI(phi)"}


def methods_for(mode):
    if len(MODES[mode]["tracks"]) > 1:
        return ["LQI", "LQG", "Hinf"]
    m = ["PIDF", "LQI", "LQG", "Hinf", "LoopShaping"]
    if mode == "heading":
        m.append("Cascade")
    return m


class APError(ValueError):
    pass


def _find_input(role, names):
    low = [n.lower() for n in names]
    cands = {"de": ["de", "delta_e", "elevator", "dele"], "dT": ["dt", "delta_t", "throttle", "t", "dth"],
             "da": ["da", "delta_a", "aileron", "dela"], "dr": ["dr", "delta_r", "rudder", "delr"]}[role]
    for c in cands:
        if c in low:
            return low.index(c)
    fallback = {"de": 0, "dT": 1, "da": 0, "dr": 1}[role]
    return fallback if fallback < len(names) else 0


def default_input(mode, input_names):
    return _find_input(MODES[mode]["inputs"][0], input_names)


def is_angle_input(name):
    return name.lower() not in ("dt", "delta_t", "t", "throttle", "thrust", "dth")


# ---------------------------------------------------------------------------
@dataclass
class ActuatorSpec:
    tau: float = 0.0     # first-order lag time constant [s], 0 = ideal
    delay: float = 0.0   # pure time delay [s] (1st-order Pade), 0 = none

    @property
    def active(self):
        return self.tau > 0 or self.delay > 0


def add_actuators(Aol, B, Ksas, names, units, input_names, spec):
    """Insert actuator dynamics between the commands and the aircraft inputs.
    Returns Aol, B (now from commands), Ksas, names, units."""
    if spec is None or not spec.active:
        return Aol, B, Ksas, names, units
    n0, m = Aol.shape[0], B.shape[1]
    blocks = []   # per input: (A_i, Bc_i (from command), Cx_i (to aircraft input), Dc_i)
    for i in range(m):
        Ai, Bi, Ci, Di, nm = np.zeros((0, 0)), np.zeros((0, 1)), np.zeros((1, 0)), np.array([[1.0]]), []
        if spec.delay > 0:
            T = spec.delay
            Ai = np.array([[-2.0 / T]]); Bi = np.array([[2.0 / T]]); Ci = np.array([[2.0]]); Di = np.array([[-1.0]])
            nm.append(f"{input_names[i]}_delay")
        if spec.tau > 0:
            t = spec.tau
            na = Ai.shape[0]
            Ai = np.block([[Ai, np.zeros((na, 1))], [Ci / t, np.array([[-1.0 / t]])]])
            Bi = np.vstack([Bi, Di / t])
            Ci = np.hstack([np.zeros((1, na)), [[1.0]]])
            Di = np.array([[0.0]])
            nm.append(f"{input_names[i]}_act")
        blocks.append((Ai, Bi, Ci, Di, nm))
    na_tot = sum(b[0].shape[0] for b in blocks)
    N = n0 + na_tot
    A = np.zeros((N, N)); Bn = np.zeros((N, m))
    A[:n0, :n0] = Aol
    o = n0
    for i, (Ai, Bi, Ci, Di, nm) in enumerate(blocks):
        k = Ai.shape[0]
        A[o:o + k, o:o + k] = Ai
        Bn[o:o + k, i] = Bi[:, 0]
        A[:n0, o:o + k] = B[:, [i]] @ Ci
        Bn[:n0, i] += B[:, i] * Di[0, 0]
        names = names + nm
        units = units + ["rad"] * k
        o += k
    Ks = np.hstack([Ksas, np.zeros((m, na_tot))])
    return A, Bn, Ks, names, units


@dataclass
class APPlant:
    mode: str
    A: np.ndarray            # SAS closed (and actuators)
    Aol: np.ndarray
    B: np.ndarray            # from surface commands (all inputs)
    Ksas: np.ndarray
    js: list                 # autopilot inputs
    ks: list                 # tracked states
    state_names: list
    input_names: list
    state_scale: np.ndarray  # internal -> display units
    input_scale: np.ndarray
    track_units: list
    phi_index: int = None    # bank angle index (lateral) for bank limit
    umax_hint: np.ndarray = None  # typical command size for inputs without a deflection limit
    ideal: object = None     # same plant without actuator dynamics (design model, measurable states)
    meas: list = None        # measured states (output feedback: LQG estimator / H-inf); None = see methods
    meas_noise: np.ndarray = None  # sensor noise std per measured state (internal units)

    @property
    def Cm(self):
        """Measurement matrix of the sensors (only when 'meas' is set)."""
        Cm = np.zeros((len(self.meas), self.n))
        for i, k in enumerate(self.meas):
            Cm[i, k] = 1
        return Cm

    @property
    def design(self):
        """Model used by the state-feedback syntheses: airframe (+SAS, washout, h/psi) with
        ideal actuators. Actuator and delay states are not measured, so controllers never
        feed them back; they are only part of the evaluation model."""
        return self.ideal if self.ideal is not None else self

    @property
    def n(self):
        return self.A.shape[0]

    @property
    def p(self):
        return len(self.ks)

    @property
    def q(self):
        return len(self.js)

    @property
    def j(self):
        return self.js[0]

    @property
    def k(self):
        return self.ks[0]

    @property
    def Bq(self):
        return self.B[:, self.js]

    @property
    def b(self):
        return self.B[:, [self.j]]

    @property
    def C(self):
        C = np.zeros((self.p, self.n))
        for i, k in enumerate(self.ks):
            C[i, k] = 1
        return C

    @property
    def c(self):
        return self.C[[0]]

    @property
    def track_unit(self):
        return self.track_units[0]

    @property
    def track_names(self):
        return [self.state_names[k] for k in self.ks]


def build_plant(mode, long_model=None, long_sas=None, lat_model=None, lat_sas=None, input_index=None,
                actuator=None):
    info = MODES[mode]
    if info["axis"] == "long":
        if long_model is None or long_sas is None:
            raise APError("Design the longitudinal SAS first.")
        model = long_model
        Aol, B, Ksas, names, units = long_extended(long_model, long_sas)
    else:
        if lat_model is None or lat_sas is None:
            raise APError("Design the lateral SAS first.")
        model = lat_model
        Aol, B, Ksas, names, units = lat_extended(lat_model, lat_sas)
    n0 = Aol.shape[0]
    tracks = info["tracks"]
    if "h" in tracks or "psi" in tracks:
        crow = np.zeros(n0)
        if "h" in tracks:
            crow[:4] = model.c_hdot
            nm = "h"; un = "m"
        else:
            crow[:4] = model.c_psidot
            nm = "psi"; un = "rad"
        Aol = np.block([[Aol, np.zeros((n0, 1))], [crow.reshape(1, n0), np.zeros((1, 1))]])
        B = np.vstack([B, np.zeros((1, B.shape[1]))])
        Ksas = np.hstack([Ksas, np.zeros((Ksas.shape[0], 1))])
        names.append(nm); units.append(un)
    idx = {"theta": 3, "speed": 0, "phi": 3, "beta": 0}
    ks = []
    for t in tracks:
        ks.append(names.index(t) if t in ("h", "psi") else idx[t])
    js = [_find_input(r, model.input_names) for r in info["inputs"]]
    if input_index is not None and len(js) == 1:
        js = [input_index]
    if len(set(js)) != len(js):
        raise APError("This mode needs two different control inputs (check the input names).")
    for j in js:
        if j >= B.shape[1]:
            raise APError(f"Input #{j+1} does not exist in B.")
        if np.allclose(B[:, j], 0):
            raise APError(f"Input '{model.input_names[j]}' has no effect (B column is zero).")
    # typical command size for non-angular inputs (e.g. thrust in N), from the aircraft B
    umax_hint = np.array([0.1 / max(np.abs(B[:, j]).max(), 1e-12) for j in range(B.shape[1])])
    in_scale = np.array([1 / DEG if is_angle_input(nm) else 1.0 for nm in model.input_names])

    def make(Aol_, B_, K_, names_, units_, ideal=None):
        scale = np.array([1 / DEG if u in ("rad", "rad/s") else 1.0 for u in units_])
        return APPlant(mode, Aol_ - B_ @ K_, Aol_, B_, K_, js, ks, names_, list(model.input_names), scale,
                       in_scale, list(info["units"]), 3 if info["axis"] == "lat" else None, umax_hint, ideal)

    ideal = make(Aol, B, Ksas, list(names), list(units))
    if actuator is None or not actuator.active:
        return ideal
    Aol, B, Ksas, names, units = add_actuators(Aol, B, Ksas, names, units, model.input_names, actuator)
    return make(Aol, B, Ksas, names, units, ideal)


def _embed(ctrl, plant):
    """Pad a controller designed on plant.design with zero gains on the unmeasured
    actuator/delay states of the full plant."""
    n_full, n_d, p = plant.n, plant.design.n, plant.p
    if n_full == n_d:
        return ctrl
    pad = n_full - n_d
    ctrl.Bc = np.hstack([ctrl.Bc, np.zeros((ctrl.Bc.shape[0], pad))])
    ctrl.Dc = np.hstack([ctrl.Dc, np.zeros((ctrl.Dc.shape[0], pad))])
    return ctrl


def _umax(plant, settings, j):
    lim = settings.limits[j]
    if np.isfinite(lim):
        return lim
    if plant.umax_hint is not None:
        return plant.umax_hint[j]
    return 0.1 / max(np.abs(plant.B[:, j]).max(), 1e-12)


# ---------------------------------------------------------------------------
@dataclass
class APSettings:
    ref: object = 5.0             # display units; scalar or list (one per tracked output)
    T: float = 20.0
    dt: float = 0.01
    prefilter: float = 2.0        # rad/s, 0 = none
    limits: np.ndarray = None     # per input, internal units (rad / raw)
    rates: np.ndarray = None      # per input, internal units / s
    gm_spec: float = 6.0
    pm_spec: float = 35.0
    dm_spec: float = 0.2
    x0_track: object = 0.0        # initial offset of tracked states (display units)
    noise_std: object = 0.5       # tracked-measurement noise (display units)
    noise_bw: float = 25.0        # rad/s first-order noise colouring
    use_saturation: bool = True
    antiwindup: bool = True
    bank_limit: float = 30.0      # deg, lateral modes (0 = none)

    def ensure(self, plant):
        m = plant.B.shape[1]
        if self.limits is None or len(self.limits) != m:
            self.limits = np.array([30 * DEG if is_angle_input(nm) else np.inf for nm in plant.input_names])
        if self.rates is None or len(self.rates) != m:
            self.rates = np.array([60 * DEG if is_angle_input(nm) else np.inf for nm in plant.input_names])
        p = plant.p
        self.ref = _vec(self.ref, p)
        self.x0_track = _vec(self.x0_track, p)
        self.noise_std = _vec(self.noise_std, p)
        return self


def _vec(v, p):
    a = np.atleast_1d(np.asarray(v, float)).ravel()
    if len(a) >= p:
        return a[:p].copy()
    return np.concatenate([a, np.full(p - len(a), a[-1] if len(a) else 0.0)])


DEFAULT_SCALE = {"deg": 1.0, "m": 5.0, "m/s": 1.0}


def _amp_internal(plant, settings):
    """Characteristic amplitude per tracked output (reference step, initial offset or default)."""
    settings.ensure(plant)
    out = []
    for i, k in enumerate(plant.ks):
        a = abs(settings.ref[i]) or abs(settings.x0_track[i]) or DEFAULT_SCALE.get(plant.track_units[i], 1.0)
        out.append(a / plant.state_scale[k])
    return np.array(out)


@dataclass
class Controller:
    method: str
    Ac: np.ndarray
    Bc: np.ndarray
    Cc: np.ndarray
    Dc: np.ndarray
    params: dict
    gains: dict = field(default_factory=dict)
    info: dict = field(default_factory=dict)
    Nbar: object = 1.0
    int_states: list = field(default_factory=list)  # integrator state indices (anti-windup)

    @property
    def nc(self):
        return self.Ac.shape[0]


def _nbar(ctrl, p):
    N = np.atleast_2d(np.asarray(ctrl.Nbar, float))
    return N if N.shape == (p, p) else N[0, 0] * np.eye(p)


def _error_ctrl(plant, Ak, Bk, Ck, Dk):
    """Wrap a SISO error-driven controller u = K(s)(r_f - y) into the common form."""
    n, k = plant.n, plant.k
    Ak = np.atleast_2d(Ak); nc = Ak.shape[0]
    Bk = np.asarray(Bk, float).reshape(nc, 1)
    Ck = np.asarray(Ck, float).reshape(1, nc)
    Dk = float(np.asarray(Dk).ravel()[0])
    ek = np.zeros((1, n)); ek[0, k] = 1
    Bc = np.hstack([Bk, -Bk @ ek])
    Dc = np.hstack([[[Dk]], -Dk * ek])
    return Ak, Bc, Ck, Dc


# ---------------------------------------------------------------------------
# closed loop assembly + evaluation
# ---------------------------------------------------------------------------
def closed_loop(plant, ctrl, prefilter):
    """Continuous closed loop. States [x; xc; xp]. Inputs [r (p); noise (p); d_u (q)].
    Outputs [x (n); u (q); delta_all (m)]."""
    n, m, nc, p, q = plant.n, plant.B.shape[1], ctrl.nc, plant.p, plant.q
    A, Bq = plant.A, plant.Bq
    Bcr, Bcx = ctrl.Bc[:, :p], ctrl.Bc[:, p:]
    Dcr, Dcx = ctrl.Dc[:, :p], ctrl.Dc[:, p:]
    Ek = plant.C.T                       # n x p
    Nb = _nbar(ctrl, p)
    npf = p if prefilter > 0 else 0
    N = n + nc + npf
    Acl = np.zeros((N, N)); Bcl = np.zeros((N, 2 * p + q))
    sx, sc, sp = slice(0, n), slice(n, n + nc), slice(n + nc, N)
    rr, rn, rd = slice(0, p), slice(p, 2 * p), slice(2 * p, 2 * p + q)
    Acl[sx, sx] = A + Bq @ Dcx
    Acl[sx, sc] = Bq @ ctrl.Cc
    Acl[sc, sx] = Bcx
    Acl[sc, sc] = ctrl.Ac
    Bcl[sx, rn] = Bq @ Dcx @ Ek
    Bcl[sc, rn] = Bcx @ Ek
    Bcl[sx, rd] = Bq
    Ccl = np.zeros((n + q + m, N)); Dcl = np.zeros((n + q + m, 2 * p + q))
    Ccl[:n, sx] = np.eye(n)
    Ccl[n:n + q, sx] = Dcx
    Ccl[n:n + q, sc] = ctrl.Cc
    Dcl[n:n + q, rn] = Dcx @ Ek
    if npf:
        Acl[sx, sp] = Bq @ Dcr @ Nb
        Acl[sc, sp] = Bcr @ Nb
        Acl[sp, sp] = -prefilter * np.eye(p)
        Bcl[sp, rr] = prefilter * np.eye(p)
        Ccl[n:n + q, sp] = Dcr @ Nb
    else:
        Bcl[sx, rr] = Bq @ Dcr @ Nb
        Bcl[sc, rr] = Bcr @ Nb
        Dcl[n:n + q, rr] = Dcr @ Nb
    E = np.zeros((m, q))
    for i, j in enumerate(plant.js):
        E[j, i] = 1
    Ccl[n + q:] = E @ Ccl[n:n + q] - np.hstack([plant.Ksas, np.zeros((m, N - n))])
    Dcl[n + q:] = E @ Dcl[n:n + q]
    return Acl, Bcl, Ccl, Dcl


def loop_matrices(plant, ctrl):
    """Loop transfer at the autopilot inputs: L = -K_x G_x (q x q), negative-feedback convention."""
    n, nc, p = plant.n, ctrl.nc, plant.p
    Bcx, Dcx = ctrl.Bc[:, p:], ctrl.Dc[:, p:]
    AL = np.block([[plant.A, np.zeros((n, nc))], [Bcx, ctrl.Ac]])
    BL = np.vstack([plant.Bq, np.zeros((nc, plant.q))])
    CL = -np.hstack([Dcx, ctrl.Cc])
    return AL, BL, CL


def loop_margins(plant, ctrl, wmax=1e4, npts=2500):
    """Loop-at-a-time margins at every autopilot input (others closed).
    Returns the worst case, with the per-channel results in 'channels'."""
    AL, BL, CL = loop_matrices(plant, ctrl)
    chans = []
    for i in range(plant.q):
        o = [x for x in range(plant.q) if x != i]
        Ai = AL - BL[:, o] @ CL[o, :] if o else AL
        chans.append(lti.margins(Ai, BL[:, [i]], CL[[i], :], 0.0, wmax=wmax, npts=npts))
    worst = dict(chans[0])
    for c in chans[1:]:
        if abs(c["gm_db"]) < abs(worst["gm_db"]):
            worst["gm_db"], worst["wcg"] = c["gm_db"], c["wcg"]
        if c["pm"] < worst["pm"]:
            worst["pm"], worst["wcp"] = c["pm"], c["wcp"]
        worst["dm"] = min(worst["dm"], c["dm"])
    worst["channels"] = chans
    return worst


def fix_dc(plant, ctrl, prefilter):
    """Set Nbar so the DC gain r -> y is identity (integral controllers already ~I)."""
    p = plant.p
    ctrl.Nbar = 1.0 if p == 1 else np.eye(p)
    Acl, Bcl, Ccl, Dcl = closed_loop(plant, ctrl, prefilter)
    if np.any(np.linalg.eigvals(Acl).real >= 0):
        return ctrl
    try:
        DC = Dcl[plant.ks][:, :p] - Ccl[plant.ks] @ np.linalg.solve(Acl, Bcl[:, :p])
    except np.linalg.LinAlgError:
        return ctrl
    if np.max(np.abs(DC - np.eye(p))) > 1e-4 and np.linalg.cond(DC) < 1e8:
        Nb = np.linalg.inv(DC)
        ctrl.Nbar = float(Nb[0, 0]) if p == 1 else Nb
    return ctrl


@dataclass
class APResult:
    method: str
    t: np.ndarray
    y: np.ndarray            # tracked outputs (N, p), display units
    x: np.ndarray            # states (display units)
    u: np.ndarray            # outer-loop commands (N, q) display units
    delta: np.ndarray        # all surfaces total command (display units) (N, m)
    margins: dict
    metrics: list            # per tracked output
    poles: np.ndarray
    stable: bool
    peak_effort: np.ndarray  # display units per input
    feasible: dict
    peak_bank: float = np.nan


def evaluate(plant, ctrl, settings, with_margins=True, quick=False):
    settings.ensure(plant)
    dt = settings.dt if not quick else max(settings.dt, settings.T / 600)
    N = int(round(settings.T / dt)) + 1
    t = np.linspace(0, dt * (N - 1), N)
    Acl, Bcl, Ccl, Dcl = closed_loop(plant, ctrl, settings.prefilter)
    poles = np.linalg.eigvals(Acl)
    stable = bool(np.all(poles.real < -1e-9))
    p, q, n = plant.p, plant.q, plant.n
    sc = plant.state_scale[plant.ks]
    u_in = np.zeros((N, Bcl.shape[1])); u_in[:, :p] = settings.ref / sc
    x0 = np.zeros(Acl.shape[0])
    x0[plant.ks] = settings.x0_track / sc
    if stable or quick:
        Y, _ = lti.simulate(Acl, Bcl, Ccl, Dcl, u_in, dt, x0)
    else:
        Y = np.full((N, Ccl.shape[0]), np.nan)
    xs = Y[:, :n] * plant.state_scale
    us = Y[:, n:n + q] * plant.input_scale[plant.js]
    ds = Y[:, n + q:] * plant.input_scale
    y = xs[:, plant.ks]
    mg = loop_margins(plant, ctrl) if with_margins else {"gm_db": np.nan, "pm": np.nan, "dm": np.nan}
    met = [lti.step_metrics(t, y[:, i], settings.ref[i]) if stable else {} for i in range(p)]
    peak = np.nanmax(np.abs(ds), axis=0) if stable else np.full(plant.B.shape[1], np.inf)
    lim_disp = settings.limits * plant.input_scale
    bank = float(np.nanmax(np.abs(xs[:, plant.phi_index]))) if plant.phi_index is not None and stable else np.nan
    feas = {
        "stable": stable,
        "effort": bool(np.all(peak <= lim_disp * 1.005 + 1e-9)),
        "gm": bool(abs(mg["gm_db"]) >= settings.gm_spec) if with_margins else True,
        "pm": bool(mg["pm"] >= settings.pm_spec) if with_margins else True,
        "dm": bool(mg["dm"] >= settings.dm_spec) if with_margins else True,
    }
    if plant.phi_index is not None and settings.bank_limit > 0 and "psi" in plant.track_names:
        feas["bank"] = bool(bank <= settings.bank_limit * 1.005)
    return APResult(ctrl.method, t, y, xs, us, ds, mg, met, poles, stable, peak, feas, bank)


def _cost(plant, ctrl, settings, margins_too=True):
    """Scalar tuning cost: tracking error + penalties for violated constraints."""
    cancel.check()
    res = evaluate(plant, ctrl, settings, with_margins=False, quick=True)
    if not res.stable:
        # graded, so a search that starts in the unstable region is led towards stability
        # (a flat value stops differential evolution after its first population)
        return 1e5 + 1e3 * min(max(float(res.poles.real.max()), 0.0), 100.0), res, None
    if not np.all(np.isfinite(res.y)):
        return 1e6, res, None
    amp = _amp_internal(plant, settings) * plant.state_scale[plant.ks]
    e = (settings.ref - res.y) / amp
    dt = res.t[1] - res.t[0]
    T = res.t[-1]
    J = float(np.sum(np.abs(e) * (1 + res.t / T)[:, None])) * dt / T
    J += 3 * float(np.sum(np.abs(e[-1])))
    for i, mt in enumerate(res.metrics):
        os_ = mt.get("overshoot", 0.0) or 0.0
        if np.isfinite(os_):
            J += 0.02 * max(0.0, os_ - 5.0)
    lim = settings.limits * plant.input_scale
    # effort the SAS alone already needs (e.g. reacting to an initial offset) is not
    # the outer loop's fault: only penalise what exceeds max(limit, SAS-only peak)
    lim = np.maximum(lim, 1.02 * _baseline_peak(plant, settings))
    viol = np.maximum(res.peak_effort / np.where(np.isfinite(lim), lim, np.inf) - 1, 0)
    J += 20 * float(np.sum(viol))
    if "bank" in res.feasible and np.isfinite(res.peak_bank):
        J += 20 * max(0.0, res.peak_bank / settings.bank_limit - 1)
    mg = None
    if margins_too:
        mg = loop_margins(plant, ctrl, npts=900)
        gm = abs(mg["gm_db"]) if np.isfinite(mg["gm_db"]) else 99.0
        dm = mg["dm"] if np.isfinite(mg["dm"]) else 99.0
        # a violated margin always costs more than any tracking gain (fixed step + proportional);
        # the 1 % guard keeps the optimum off the boundary, where the finer margin computation
        # of evaluate() could still report a violation
        for val, spec in ((gm, settings.gm_spec), (mg["pm"], settings.pm_spec), (dm, settings.dm_spec)):
            if val < 1.01 * spec:
                J += 1.0 + 10 * (spec - val) / spec
    return J, res, mg


def _baseline_peak(plant, settings):
    key = (tuple(settings.ref), tuple(settings.x0_track), settings.T, settings.prefilter)
    cache = plant.__dict__.setdefault("_baseline", {})
    if key not in cache:
        n, p, q = plant.n, plant.p, plant.q
        z = Controller("none", np.zeros((0, 0)), np.zeros((0, p + n)), np.zeros((q, 0)),
                       np.zeros((q, p + n)), {})
        res = evaluate(plant, z, settings, with_margins=False, quick=True)
        cache[key] = np.where(np.isfinite(res.peak_effort), res.peak_effort, 0.0)
    return cache[key]


def _need_siso(plant, method):
    if plant.p != 1 or plant.q != 1:
        raise APError(f"{method} is available only for single-loop modes.")


# ---------------------------------------------------------------------------
# PIDF
# ---------------------------------------------------------------------------
def make_pidf(plant, Kp, Ki, Kd, Tf):
    _need_siso(plant, "PIDF")
    Tf = max(Tf, 1e-4)
    Ak = np.array([[0.0, 0.0], [0.0, -1.0 / Tf]])
    Bk = np.array([[1.0], [1.0 / Tf]])
    Ck = np.array([[Ki, -Kd / Tf]])
    Dk = Kp + Kd / Tf
    Ac, Bc, Cc, Dc = _error_ctrl(plant, Ak, Bk, Ck, Dk)
    return Controller("PIDF", Ac, Bc, Cc, Dc, {"Kp": Kp, "Ki": Ki, "Kd": Kd, "Tf": Tf},
                      {"Kp": Kp, "Ki": Ki, "Kd": Kd, "Tf": Tf}, int_states=[0])


def plant_sign(plant):
    """Sign of y/u at low frequency (works for type-0 and integrating plants)."""
    s = 1e-4
    G = plant.c @ np.linalg.solve(s * np.eye(plant.n) - plant.A, plant.b)
    v = float(G.real.ravel()[0])
    return 1.0 if v >= 0 else -1.0


def tune_pidf(plant, settings, Tf=None):
    import scipy.optimize
    settings.ensure(plant)
    _need_siso(plant, "PIDF")
    sgn = plant_sign(plant)
    if Tf is None:
        Tf = max(0.02, settings.T / 400)
    best = {"J": np.inf}

    def f(pp, sg):
        Kp, Ki, Kd = sg * 10 ** pp[0], sg * 10 ** pp[1], sg * 10 ** pp[2]
        c = make_pidf(plant, Kp, Ki, Kd, Tf)
        J, _, _ = _cost(plant, c, settings)
        if J < best["J"]:
            best.update(J=J, c=c)
        return J

    for sg in (sgn, -sgn):
        scipy.optimize.differential_evolution(
            f, [(-4, 2.5), (-5, 2.0), (-5, 2.0)], args=(sg,), maxiter=18, popsize=7,
            seed=1, tol=1e-4, polish=False, init="sobol")
        if best["J"] < 1.0:
            break
    c = best["c"]
    p0 = np.log10(np.abs([c.params["Kp"], c.params["Ki"], c.params["Kd"]]) + 1e-12)
    sg = np.sign(c.params["Kp"]) or sgn
    scipy.optimize.minimize(f, p0, args=(sg,), method="Nelder-Mead",
                            options={"maxiter": 150, "xatol": 1e-3, "fatol": 1e-5})
    c = best["c"]
    c.info["cost"] = best["J"]
    return fix_dc(plant, c, settings.prefilter)


# ---------------------------------------------------------------------------
# LQI (MIMO)
# ---------------------------------------------------------------------------
def _aug(plant):
    n, p = plant.n, plant.p
    Aa = np.block([[plant.A, np.zeros((n, p))], [-plant.C, np.zeros((p, p))]])
    Ba = np.vstack([plant.Bq, np.zeros((p, plant.q))])
    return Aa, Ba


def lqr(A, B, Q, R):
    R = np.atleast_2d(R)
    P = scipy.linalg.solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    return K, P


def make_lqi(plant, Q, R):
    full, plant = plant, plant.design
    Aa, Ba = _aug(plant)
    R = np.atleast_2d(np.diag(np.atleast_1d(R))) if np.ndim(R) < 2 else np.atleast_2d(R)
    K, _ = lqr(Aa, Ba, Q, R)
    n, p, q = plant.n, plant.p, plant.q
    Kx, Ki = K[:, :n], K[:, n:]
    # xi' = r_f - C x ; u = -Kx x - Ki xi
    Ac = np.zeros((p, p))
    Bc = np.hstack([np.eye(p), -plant.C])
    Cc = -Ki
    Dc = np.hstack([np.zeros((q, p)), -Kx])
    g = {}
    for r, jn in enumerate(plant.js):
        pre = f"{plant.input_names[jn]}: " if q > 1 else ""
        for i, nm in enumerate(plant.state_names):
            g[f"{pre}K_{nm}"] = float(Kx[r, i])
        for i, tn in enumerate(plant.track_names):
            g[f"{pre}K_i({tn})"] = float(Ki[r, i])
    return _embed(Controller("LQI", Ac, Bc, Cc, Dc,
                             {"Q": np.diag(Q).tolist(), "R": np.diag(R).tolist()},
                             g, {"K": K}, int_states=list(range(p))), full)


XMAX = {"u": 10.0, "V": 10.0, "nz": 1.0, "alpha": 10 * DEG, "q": 20 * DEG, "theta": 10 * DEG,
        "h": 50.0, "beta": 5 * DEG, "p": 30 * DEG, "r": 20 * DEG, "phi": 30 * DEG, "psi": 20 * DEG,
        "r_washout": 20 * DEG}


def state_xmax(plant):
    out = []
    for nm in plant.state_names:
        if nm.endswith("_act") or nm.endswith("_delay"):
            out.append(30 * DEG)
        else:
            out.append(XMAX.get(nm, 1.0))
    return np.array(out)


def bryson_weights(plant, settings):
    """Bryson's rule: 1/x_max^2 per state, 1/u_max^2 per command, integrator scale per output."""
    q = 1.0 / state_xmax(plant.design) ** 2
    R = [1.0 / _umax(plant, settings, j) ** 2 for j in plant.js]
    return q, np.array(R), _amp_internal(plant, settings)


def tune_lqi(plant, settings):
    settings.ensure(plant)
    q, R, eint = bryson_weights(plant, settings)
    best = (np.inf, None, None)

    def trial(rho, mu, lam):
        nonlocal best
        qq = q.copy(); qq[plant.ks] *= mu
        Q = np.diag(np.concatenate([qq, lam / eint ** 2]))
        try:
            c = make_lqi(plant, Q, rho * R)
        except (np.linalg.LinAlgError, ValueError):
            return
        J, _, _ = _cost(plant, c, settings)
        if J < best[0]:
            best = (J, c, (rho, mu, lam))

    for rho in (1.0, 10.0, 100.0):                 # control weight (bandwidth / delay margin)
        for mu in np.logspace(-2, 3, 7):           # extra weight on the tracked states
            for lam in np.logspace(-3, 3, 9):      # integrator weight
                trial(rho, mu, lam)
    if best[1] is not None:                        # local refinement between the grid points
        for _ in range(2):
            r0, m0, l0 = best[2]
            for fr, fm, fl in ((1.5, 1, 1), (1 / 1.5, 1, 1), (1, 1.8, 1), (1, 1 / 1.8, 1), (1, 1, 1.8), (1, 1, 1 / 1.8)):
                trial(r0 * fr, m0 * fm, l0 * fl)
    if best[1] is None:
        raise APError("LQI: Riccati equation has no stabilizing solution.")
    c = best[1]
    c.info["cost"] = best[0]
    return c


# ---------------------------------------------------------------------------
# LQG = LQI gains + Kalman estimator on the tracked measurements
# ---------------------------------------------------------------------------
def make_lqg(plant, lqi_ctrl, Qn, Rn):
    full, plant = plant, plant.design
    n, p, q = plant.n, plant.p, plant.q
    A, Bq, C = plant.A, plant.Bq, plant.C
    # the estimator uses the sensors when they are declared, otherwise the tracked outputs
    Cy = plant.Cm if plant.meas else C
    Qn = np.asarray(Qn, float)
    if Qn.ndim == 1 and Qn.size == n * n:          # flattened matrix from the parameter table
        Qn = Qn.reshape(n, n)
    Qn = np.atleast_2d(Qn)
    if Qn.shape == (1, 1):
        Qn = Qn[0, 0] * np.eye(n)
    Rn = np.diag(np.atleast_1d(Rn)) if np.ndim(Rn) < 2 else np.atleast_2d(Rn)
    P = scipy.linalg.solve_continuous_are(A.T, Cy.T, Qn, Rn)
    L = P @ Cy.T @ np.linalg.inv(Rn)
    K = lqi_ctrl.info["K"]
    Kx, Ki = K[:, :n], K[:, n:]
    if plant.meas:
        # sensors: estimator corrected by every measurement, integrators on the measured tracked
        # outputs (xi' = r - y)
        Ac = np.block([[A - Bq @ Kx - L @ Cy, -Bq @ Ki], [np.zeros((p, n)), np.zeros((p, p))]])
        Bc = np.block([[np.zeros((n, p)), L @ Cy], [np.eye(p), -C]])
    else:
        # states xc = [x_hat (n); xi (p)]
        Ac = np.block([[A - Bq @ Kx - L @ C, -Bq @ Ki], [-C, np.zeros((p, p))]])
        Bc = np.block([[np.zeros((n, p)), L @ C], [np.eye(p), np.zeros((p, n))]])
    Cc = np.hstack([-Kx, -Ki])
    Dc = np.zeros((q, p + n))
    g = dict(lqi_ctrl.gains)
    ynames = [plant.state_names[k] for k in plant.meas] if plant.meas else plant.track_names
    for i, nm in enumerate(plant.state_names):
        for r, tn in enumerate(ynames):
            g[f"L_{nm}<-{tn} (Kalman)" if len(ynames) > 1 else f"L_{nm} (Kalman)"] = float(L[i, r])
    return _embed(Controller("LQG", Ac, Bc, Cc, Dc,
                             {"Q": lqi_ctrl.params["Q"], "R": lqi_ctrl.params["R"],
                              "Qn": Qn.tolist() if np.count_nonzero(Qn - np.diag(np.diag(Qn))) or
                              np.ptp(np.diag(Qn)) > 0 else float(Qn[0, 0]),
                              "Rn": np.diag(Rn).tolist()},
                             g, {"K": K, "L": L, "est_poles": np.linalg.eigvals(A - L @ Cy)},
                             int_states=list(range(n, n + p))), full)


def noise_h2(plant, ctrl, settings):
    """H2 gain from white measurement noise to the tracked outputs."""
    Acl, Bcl, Ccl, _ = closed_loop(plant, ctrl, settings.prefilter)
    p = plant.p
    return lti.h2_norm(Acl, Bcl[:, p:2 * p], Ccl[plant.ks])


def tune_lqg(plant, settings, lqi_ctrl=None):
    settings.ensure(plant)
    if lqi_ctrl is None:
        lqi_ctrl = tune_lqi(plant, settings)
    if plant.design.meas:
        Rn = np.maximum(np.asarray(plant.design.meas_noise, float), 1e-6) ** 2
    else:
        sig = settings.noise_std / plant.state_scale[plant.ks]
        Rn = np.maximum(sig, 1e-6) ** 2
    J_lqi, _, _ = _cost(plant, lqi_ctrl, settings)
    wc_lqi = loop_margins(plant, lqi_ctrl)["wcp"]
    Aa, Ba = _aug(plant.design)
    w_lqi = np.abs(np.linalg.eigvals(Aa - Ba @ lqi_ctrl.info["K"])).max()
    cands = []
    # two process-noise families: white on every state, and loop transfer recovery
    # (fictitious noise entering with the autopilot commands, q^2 B B', which drives the
    # loop at the plant input towards the LQI loop)
    Bq = plant.design.Bq
    n_d = plant.design.n
    noises = [qn for qn in np.logspace(-10, 4, 29)]
    noises += [qn * (Bq @ Bq.T) + 1e-10 * np.eye(n_d) for qn in np.logspace(-6, 8, 15)]
    for Qn in noises:
        try:
            c = make_lqg(plant, lqi_ctrl, Qn, Rn)
        except (np.linalg.LinAlgError, ValueError):
            continue
        J, _, _ = _cost(plant, c, settings)
        if J >= 1e5:
            continue
        mg = loop_margins(plant, c)
        # the estimator must really use the measurement: the LQG loop has to keep a
        # gain crossover close to the LQI one (loop-transfer recovery), otherwise the
        # "infinite" margins only mean the loop is open
        if np.isfinite(wc_lqi) and np.isfinite(mg["wcp"]):
            rec = abs(np.log10(mg["wcp"] / wc_lqi))
        else:
            rec = np.inf
        ok_m = (abs(mg["gm_db"]) >= settings.gm_spec and mg["pm"] >= settings.pm_spec
                and mg["dm"] >= settings.dm_spec)
        fast = np.abs(c.info["est_poles"]).max() <= 20 * w_lqi
        cands.append({"J": J, "h2": noise_h2(plant, c, settings), "c": c, "rec": rec,
                      "ok": ok_m and fast, "mg": mg})
    if not cands:
        raise APError("LQG: no stabilizing Kalman filter found.")
    good = [x for x in cands if x["ok"] and x["rec"] < 0.3 and x["J"] <= 1.25 * J_lqi + 0.05]
    if good:
        best = min(good, key=lambda x: x["h2"])
    else:
        # nothing meets every requirement: take the most robust candidate (largest worst-case
        # ratio margin / requirement) among those that track reasonably well
        def worst_ratio(x):
            mg = x["mg"]
            gm = abs(mg["gm_db"]) if np.isfinite(mg["gm_db"]) else 99.0
            dm = mg["dm"] if np.isfinite(mg["dm"]) else 99.0
            return min(gm / settings.gm_spec, mg["pm"] / settings.pm_spec, dm / settings.dm_spec)
        ok_track = [x for x in cands if x["J"] <= 2 * J_lqi + 0.1] or cands
        best = max(ok_track, key=worst_ratio)
    c = best["c"]
    c.info["cost"] = best["J"]
    c.info["noise_h2"] = best["h2"]
    return c


# ---------------------------------------------------------------------------
# H-infinity (hinfsyn) with integral action (MIMO)
# ---------------------------------------------------------------------------
def hinfsyn_fast(A, B, C, D, nmeas, ncon):
    """Optimal H-inf controller (SLICOT SB10AD, Glover-Doyle formulas), same result as
    control.hinfsyn but about 10x faster: plain bisection on gamma (job=1) started from a
    realistic gamma instead of bisection + scan from 1e100. If the start value is not
    admissible the routine fails and a larger one is tried."""
    from slycot import sb10ad
    A, B, C, D = (np.asarray(x, float) for x in (A, B, C, D))
    n, m, np_ = A.shape[0], B.shape[1], C.shape[0]
    last = None
    for g0 in (1e4, 1e7, 1e100):
        try:
            out = sb10ad(n, m, np_, ncon, nmeas, g0, A, B, C, D, job=1)
        except Exception as e:  # noqa: BLE001  (gamma too small or numerical failure)
            last = e
            continue
        gam, Ak, Bk, Ck, Dk = out[0], out[1], out[2], out[3], out[4]
        return Ak, Bk, Ck, Dk, float(gam)
    raise APError(f"H-infinity synthesis failed: {last}")


def make_hinf(plant, W1, W2, W3, W4, nu=1e-2):
    """H-inf synthesis on the plant augmented with eta' = -y (the reference is added
    to the integrators at run time, so tracking comes from integral action).
    w = [d_u (q); n (n+p)];  z = [W1*y; W2*u; W3*x; W4*eta];  y2 = [x; eta] + nu*n.
    W1, W4: per tracked output; W2: per autopilot input; nu scales typical state sizes."""
    full, plant = plant, plant.design
    n, p, q = plant.n, plant.p, plant.q
    W1 = _vec(W1, p); W2 = _vec(W2, q); W4 = _vec(W4, p)
    Aa, Ba = _aug(plant)
    na = n + p
    # measurements: every state, or only the sensors (output feedback) when declared
    Cy = plant.Cm if plant.meas else np.eye(n)
    ny = Cy.shape[0] + p
    C2 = np.block([[Cy, np.zeros((Cy.shape[0], p))], [np.zeros((p, n)), np.eye(p)]])
    nw = q + ny
    B1 = np.zeros((na, nw)); B1[:, :q] = Ba
    nz = p + q + n + p
    C1 = np.zeros((nz, na)); D12 = np.zeros((nz, q))
    C1[:p, :n] = np.diag(W1) @ plant.C
    D12[p:p + q, :] = np.diag(W2)
    C1[p + q:p + q + n, :n] = W3 * np.eye(n)
    C1[p + q + n:, n:] = np.diag(W4)
    D11 = np.zeros((nz, nw))
    D21 = np.zeros((ny, nw)); D21[:, q:] = nu * np.eye(ny)
    Ak, Bk, Ck, Dk, gam = hinfsyn_fast(Aa, np.hstack([B1, Ba]), np.vstack([C1, C2]),
                                       np.block([[D11, D12], [D21, np.zeros((ny, q))]]), ny, q)
    nk = Ak.shape[0]
    ns = Cy.shape[0]
    Bkx, Bke = Bk[:, :ns] @ Cy, Bk[:, ns:]
    Dkx, Dke = Dk[:, :ns] @ Cy, Dk[:, ns:]
    # controller states [eta (p); xk]; inputs [r_f (p); x_meas (n)]
    Ac = np.block([[np.zeros((p, p)), np.zeros((p, nk))], [Bke, Ak]])
    Bc = np.block([[np.eye(p), -plant.C], [np.zeros((nk, p)), Bkx]])
    Cc = np.hstack([Dke, Ck])
    Dc = np.hstack([np.zeros((q, p)), Dkx])
    return _embed(Controller("Hinf", Ac, Bc, Cc, Dc,
                             {"W1": W1.tolist(), "W2": W2.tolist(), "W3": float(W3), "W4": W4.tolist(),
                              "nu": float(nu)},
                             {"gamma": float(gam), "controller order": int(nk)}, {"gamma": float(gam)},
                             int_states=list(range(p))), full)


def tune_hinf(plant, settings):
    settings.ensure(plant)
    amp = _amp_internal(plant, settings)
    umax = np.array([_umax(plant, settings, j) for j in plant.js])
    W1 = 1.0 / amp
    r0 = amp[0]
    ev = np.abs(np.linalg.eigvals(plant.A))
    ev = ev[ev > 1e-6]
    wref = float(np.median(ev)) if len(ev) else 1.0
    best = (np.inf, None, None)

    def trial(a, bta, nu, m2=1.0):
        nonlocal best
        cancel.check()
        # m2: extra control weight on the 2nd..q-th input (MIMO modes)
        w2 = a / umax * np.concatenate([[1.0], np.full(plant.q - 1, m2)])
        try:
            c = make_hinf(plant, W1, w2, 1e-3 * W1[0], bta * wref * W1, nu * r0)
        except Exception:  # noqa: BLE001  (synthesis may fail for some weights)
            return
        fix_dc(plant, c, settings.prefilter)
        J, _, _ = _cost(plant, c, settings)
        if J < best[0]:
            best = (J, c, (a, bta, nu, m2))

    # coarse grid; the measurement-noise level nu limits the controller bandwidth
    # (and therefore protects the delay margin): with actuators or a slow airframe the
    # feasible designs often need nu of a few hundred
    m2s = (1.0, 10.0) if plant.q > 1 else (1.0,)
    for a in (0.3, 3.0, 30.0):
        for nu in (0.1, 1.0, 10.0, 100.0, 1000.0):
            for bta in (0.05, 0.5, 5.0):
                for m2 in m2s:
                    trial(a, bta, nu, m2)
    if best[1] is not None:                      # local refinement around the best point
        a0, b0, n0, m0 = best[2]
        for fa in (0.4, 2.5):
            trial(a0 * fa, b0, n0, m0)
        a0 = best[2][0]
        for fn in (0.4, 2.5):
            trial(a0, b0, n0 * fn, m0)
        n0 = best[2][2]
        for fb in (0.5, 2.0):
            trial(a0, b0 * fb, n0, m0)
    if best[1] is not None and not all(evaluate(plant, best[1], settings).feasible.values()):
        # still violating a requirement: try lower-bandwidth designs
        for a in (100.0, 300.0, 1000.0):
            for nu in (3.0, 10.0, 30.0, 300.0, 3000.0):
                for bta in (0.05, 0.5):
                    for m2 in m2s:
                        trial(a, bta, nu, m2)
    if best[1] is None:
        raise APError("H-infinity synthesis failed for all weight choices.")
    c = best[1]
    c.info["cost"] = best[0]
    return c


# ---------------------------------------------------------------------------
# H-infinity loop shaping (Glover-McFarlane, normalized coprime factors)
# ---------------------------------------------------------------------------
def ncfsyn(A, B, C, gamma_factor=1.1):
    """Glover-McFarlane controller for strictly proper (A,B,C). Positive-feedback
    controller K (u = K y). Returns (Ak, Bk, Ck, Dk, gamma, gamma_min)."""
    n = A.shape[0]
    m = B.shape[1]
    p = C.shape[0]
    X = scipy.linalg.solve_continuous_are(A, B, C.T @ C, np.eye(m))
    Z = scipy.linalg.solve_continuous_are(A.T, C.T, B @ B.T, np.eye(p))
    gmin = float(np.sqrt(1 + max(np.linalg.eigvals(X @ Z).real)))
    g = gamma_factor * gmin
    L = (1 - g ** 2) * np.eye(n) + X @ Z
    F = -B.T @ X
    LiT = np.linalg.inv(L.T)
    Ak = A + B @ F + g ** 2 * LiT @ Z @ C.T @ C
    Bk = g ** 2 * LiT @ Z @ C.T
    Ck = B.T @ X
    Dk = np.zeros((m, p))
    return Ak, Bk, Ck, Dk, g, gmin


def make_loopshaping(plant, wc, rho=0.2, gamma_factor=1.1):
    """Shaping W1 = kw (s + rho*wc)/s, kw sets |G W1|(j wc) = 1."""
    _need_siso(plant, "Loop shaping")
    a = rho * wc
    Gw = lti.freqresp(plant.A, plant.b, plant.c, 0.0, [wc])[0]
    kw = 1.0 / max(abs(Gw) * abs(1j * wc + a) / wc, 1e-12)
    n = plant.n
    As = np.block([[plant.A, plant.b * kw * a], [np.zeros((1, n)), np.zeros((1, 1))]])
    Bs = np.vstack([plant.b * kw, [[1.0]]])
    Cs = np.hstack([plant.c, [[0.0]]])
    Ak, Bk, Ck, Dk, g, gmin = ncfsyn(As, Bs, Cs, gamma_factor)
    # u = W1 K_inf y (positive feedback)  ->  error form u = K_neg (r - y), K_neg = -W1 K_inf
    nk = Ak.shape[0]
    A2 = np.block([[Ak, np.zeros((nk, 1))], [Ck, np.zeros((1, 1))]])
    B2 = np.vstack([Bk, Dk])
    C2 = np.hstack([kw * Ck, [[kw * a]]])
    D2 = kw * Dk
    Ac, Bc, Cc, Dc = _error_ctrl(plant, A2, -B2, C2, -D2)
    return Controller("LoopShaping", Ac, Bc, Cc, Dc,
                      {"wc": wc, "rho": rho, "gamma_factor": gamma_factor},
                      {"gamma": g, "epsilon (=1/gamma)": 1 / g, "W1 gain kw": kw, "W1 zero a": a},
                      {"gamma": g, "gamma_min": gmin}, int_states=[nk])


def tune_loopshaping(plant, settings):
    settings.ensure(plant)
    _need_siso(plant, "Loop shaping")
    ev = np.abs(np.linalg.eigvals(plant.A))
    ev = ev[ev > 1e-6]
    wref = float(np.median(ev)) if len(ev) else 1.0
    best = (np.inf, None)
    for wc in np.logspace(np.log10(wref / 30), np.log10(wref * 5), 16):
        for rho in (0.1, 0.25):
            try:
                c = make_loopshaping(plant, wc, rho)
            except (np.linalg.LinAlgError, ValueError):
                continue
            fix_dc(plant, c, settings.prefilter)
            J, _, _ = _cost(plant, c, settings)
            if c.info["gamma"] > 4:
                J += 0.5 * (c.info["gamma"] - 4)
            if J < best[0]:
                best = (J, c)
    if best[1] is None:
        raise APError("Loop-shaping synthesis failed.")
    c = best[1]
    c.info["cost"] = best[0]
    return c


# ---------------------------------------------------------------------------
# Cascade heading hold: PI on psi -> phi_ref -> LQI roll loop
# ---------------------------------------------------------------------------
def _roll_inner_plant(plant):
    """Heading plant without the psi state, tracking phi (psi does not feed back)."""
    ipsi = plant.k
    keep = [i for i in range(plant.n) if i != ipsi]
    A = plant.A[np.ix_(keep, keep)]
    Aol = plant.Aol[np.ix_(keep, keep)]
    return APPlant("roll", A, Aol, plant.B[keep], plant.Ksas[:, keep], [plant.j], [plant.phi_index],
                   [plant.state_names[i] for i in keep], plant.input_names, plant.state_scale[keep],
                   plant.input_scale, ["deg"], plant.phi_index), keep


def make_cascade(plant, Q_in, R_in, Kp_psi, Ki_psi):
    if plant.mode != "heading":
        raise APError("The cascade structure is available for the heading-hold mode only.")
    full, plant = plant, plant.design
    inner, keep = _roll_inner_plant(plant)
    ci = make_lqi(inner, np.diag(Q_in), R_in)
    K = ci.info["K"]
    Kx = np.zeros((1, plant.n)); Kx[0, keep] = K[0, :inner.n]
    Kiphi = K[0, inner.n]
    n = plant.n
    e_psi = np.zeros((1, n)); e_psi[0, plant.k] = 1
    e_phi = np.zeros((1, n)); e_phi[0, plant.phi_index] = 1
    # xc = [xi_psi, xi_phi]
    Ac = np.array([[0.0, 0.0], [Ki_psi, 0.0]])
    Bc = np.vstack([np.hstack([[[1.0]], -e_psi]),
                    np.hstack([[[Kp_psi]], -Kp_psi * e_psi - e_phi])])
    Cc = np.array([[0.0, -Kiphi]])
    Dc = np.hstack([[[0.0]], -Kx])
    g = {"Kp_psi (phi_ref per psi error)": Kp_psi, "Ki_psi": Ki_psi, "K_i(phi)": float(Kiphi)}
    for i, nm in enumerate(plant.state_names):
        g[f"K_{nm} (inner)"] = float(Kx[0, i])
    return _embed(Controller("Cascade", Ac, Bc, Cc, Dc,
                             {"Q_inner": np.diag(Q_in).tolist(), "R_inner": np.atleast_1d(R_in).tolist(),
                              "Kp_psi": float(Kp_psi), "Ki_psi": float(Ki_psi)},
                             g, {"K_inner": K}, int_states=[0, 1]), full)


def tune_cascade(plant, settings):
    settings.ensure(plant)
    inner, _ = _roll_inner_plant(plant.design)
    st_in = APSettings(ref=10.0, T=15.0, dt=settings.dt, prefilter=0.0, limits=settings.limits,
                       rates=settings.rates, gm_spec=settings.gm_spec, pm_spec=settings.pm_spec,
                       dm_spec=settings.dm_spec)
    ci = tune_lqi(inner, st_in)
    Q_in = np.array(ci.params["Q"])
    R_in = np.array(ci.params["R"])
    best = (np.inf, None)
    # rho slows the inner roll loop (robustness to actuator lag / delay, which the
    # inner design model does not contain)
    for rho in (1.0, 10.0, 100.0):
        for kp in np.logspace(-1.5, 1, 12):
            for ki_rel in (0.0, 0.02, 0.05, 0.1, 0.2):
                c = make_cascade(plant, Q_in, rho * R_in, kp, ki_rel * kp)
                J, _, _ = _cost(plant, c, settings)
                if J < best[0]:
                    best = (J, c)
    c = best[1]
    c.info["cost"] = best[0]
    return fix_dc(plant, c, settings.prefilter)


# ---------------------------------------------------------------------------
# rebuild from (edited) parameters
# ---------------------------------------------------------------------------
def build_from_params(method, plant, params, settings):
    settings.ensure(plant)
    if method == "PIDF":
        c = make_pidf(plant, params["Kp"], params["Ki"], params["Kd"], params["Tf"])
    elif method == "LQI":
        c = make_lqi(plant, np.diag(params["Q"]), params["R"])
    elif method == "LQG":
        base = make_lqi(plant, np.diag(params["Q"]), params["R"])
        c = make_lqg(plant, base, params["Qn"], params["Rn"])
    elif method == "Hinf":
        c = make_hinf(plant, params["W1"], params["W2"], params["W3"], params["W4"], params.get("nu", 1e-2))
    elif method == "LoopShaping":
        c = make_loopshaping(plant, params["wc"], params["rho"], params.get("gamma_factor", 1.1))
    elif method == "Cascade":
        c = make_cascade(plant, np.atleast_1d(params["Q_inner"]), params["R_inner"],
                         params["Kp_psi"], params["Ki_psi"])
    else:
        raise APError(method)
    return fix_dc(plant, c, settings.prefilter)


def tune(method, plant, settings, lqi_ctrl=None):
    cancel.check()
    if method == "PIDF":
        return tune_pidf(plant, settings)
    if method == "LQI":
        return tune_lqi(plant, settings)
    if method == "LQG":
        return tune_lqg(plant, settings, lqi_ctrl)
    if method == "Hinf":
        return tune_hinf(plant, settings)
    if method == "LoopShaping":
        return tune_loopshaping(plant, settings)
    if method == "Cascade":
        return tune_cascade(plant, settings)
    raise APError(method)
