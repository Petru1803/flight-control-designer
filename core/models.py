"""Aircraft linear models (longitudinal + lateral-directional) and mode analysis."""
from dataclasses import dataclass, field

import numpy as np

G0 = 9.81

LONG_CONVENTIONS = {
    "uwqt": "[u w q theta]  (body velocities, converted to [u nz q theta])",
    "Vaqt": "[V alpha q theta]  (wind-axis)",
}


class ModelError(ValueError):
    pass


def _check(A, B, n, name):
    A = np.atleast_2d(np.asarray(A, float))
    B = np.atleast_2d(np.asarray(B, float))
    if B.shape[0] == 1 and B.shape[1] == n:
        B = B.T
    if A.shape != (n, n):
        raise ModelError(f"{name}: A must be {n}x{n}, got {A.shape[0]}x{A.shape[1]}")
    if B.shape[0] != n:
        raise ModelError(f"{name}: B must have {n} rows, got {B.shape[0]}")
    if not (np.all(np.isfinite(A)) and np.all(np.isfinite(B))):
        raise ModelError(f"{name}: matrices contain NaN/Inf")
    return A, B


@dataclass
class LongModel:
    A: np.ndarray
    B: np.ndarray
    convention: str = "uwqt"
    V0: float = 100.0
    g: float = G0
    input_names: list = field(default_factory=lambda: ["de", "dT"])

    def __post_init__(self):
        self.A, self.B = _check(self.A, self.B, 4, "Longitudinal")
        if self.convention not in LONG_CONVENTIONS:
            raise ModelError(f"Unknown convention {self.convention}")
        if self.V0 <= 0:
            raise ModelError("Trim speed V0 must be > 0")
        m = self.B.shape[1]
        names = list(self.input_names)[:m]
        defaults = ["de", "dT", "dTV", "u4", "u5", "u6"]
        while len(names) < m:
            names.append(defaults[len(names)] if len(names) < len(defaults) else f"u{len(names)+1}")
        self.input_names = names
        g, V = self.g, self.V0
        if self.convention == "uwqt":
            a = self.A[1]
            # nz = (w_dot - V*q)/g  (user's convention; w_dot without control term)
            self.T = np.array([[1, 0, 0, 0],
                               [a[0] / g, a[1] / g, (a[2] - V) / g, a[3] / g],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]], float)
            if abs(np.linalg.det(self.T)) < 1e-12:
                raise ModelError("Transformation to nz is singular (check A(2,2))")
            self.state_names = ["u", "nz", "q", "theta"]
            self.state_units = ["m/s", "g", "rad/s", "rad"]
            # h_dot = V*theta - w  (physical coords)
            c_h_phys = np.array([0, -1, 0, V], float)
        else:
            self.T = np.eye(4)
            self.state_names = ["V", "alpha", "q", "theta"]
            self.state_units = ["m/s", "rad", "rad/s", "rad"]
            c_h_phys = np.array([0, -V, 0, V], float)
        Ti = np.linalg.inv(self.T)
        self.Ad = self.T @ self.A @ Ti
        self.Bd = self.T @ self.B
        self.c_hdot = c_h_phys @ Ti
        # upward load factor (for C*), in design coordinates: y = c x + d u
        if self.convention == "uwqt":
            self.c_nz = np.array([0, -1, 0, 0], float)
            self.d_nz = np.zeros(self.B.shape[1])
        else:
            self.c_nz = (V / g) * (np.array([0, 0, 1, 0]) - self.Ad[1])
            self.d_nz = -(V / g) * self.Bd[1]

    @property
    def n_inputs(self):
        return self.B.shape[1]

    def n_alpha(self):
        """Load factor per unit AoA: n/alpha = V / (g*T_theta2), 1/T_theta2 = -Z_w."""
        zw = self.A[1, 1]
        return max(self.V0 * (-zw) / self.g, 1e-6)

    def T_theta2(self):
        zw = self.A[1, 1]
        return -1.0 / zw if zw < 0 else np.inf

    def decoupled(self):
        """Short-period (states 2,3) and phugoid (Schur complement, states 1,4)."""
        A, B = self.Ad, self.Bd
        sp = [1, 2]
        ph = [0, 3]
        Asp = A[np.ix_(sp, sp)]
        Bsp = B[sp, :]
        try:
            Aspi = np.linalg.inv(Asp)
        except np.linalg.LinAlgError:
            Aspi = np.linalg.pinv(Asp)
        Aph = A[np.ix_(ph, ph)] - A[np.ix_(ph, sp)] @ Aspi @ A[np.ix_(sp, ph)]
        Bph = B[ph, :] - A[np.ix_(ph, sp)] @ Aspi @ B[sp, :]
        return Asp, Bsp, Aph, Bph


@dataclass
class LatModel:
    A: np.ndarray
    B: np.ndarray
    V0: float = 100.0
    theta0: float = 0.0  # rad
    g: float = G0
    input_names: list = field(default_factory=lambda: ["da", "dr"])

    def __post_init__(self):
        self.A, self.B = _check(self.A, self.B, 4, "Lateral")
        m = self.B.shape[1]
        names = list(self.input_names)[:m]
        defaults = ["da", "dr", "u3", "u4"]
        while len(names) < m:
            names.append(defaults[len(names)] if len(names) < len(defaults) else f"u{len(names)+1}")
        self.input_names = names
        self.state_names = ["beta", "p", "r", "phi"]
        self.state_units = ["rad", "rad/s", "rad/s", "rad"]
        self.Ad, self.Bd = self.A, self.B
        # psi_dot = r / cos(theta0)  (small bank)
        self.c_psidot = np.array([0, 0, 1.0 / np.cos(self.theta0), 0])

    @property
    def n_inputs(self):
        return self.B.shape[1]


# ----------------------------------------------------------------------------
# Mode identification
# ----------------------------------------------------------------------------
@dataclass
class Mode:
    name: str
    poles: list
    wn: float = np.nan
    zeta: float = np.nan
    tau: float = np.nan        # time constant for real modes (s), >0 if stable
    t2: float = np.nan         # time to double (s) if unstable real
    kind: str = "osc"          # 'osc' | 'real' | 'pair-real'

    def describe(self):
        if self.kind == "real":
            p = self.poles[0].real
            if p > 1e-9:
                return f"{p:+.4f}  divergent, T2={self.t2:.2f}s"
            if p < -1e-9:
                return f"{p:+.4f}  tau={self.tau:.3f}s"
            return f"{p:+.4f}  neutral"
        if np.isfinite(self.wn):
            return f"wn={self.wn:.3f} rad/s, zeta={self.zeta:+.3f}"
        return "divergent pair"


def _pair_mode(name, p1, p2):
    s = p1 + p2
    prod = (p1 * p2).real
    m = Mode(name, [p1, p2], kind="osc" if abs(p1.imag) > 1e-9 else "pair-real")
    if prod > 0:
        m.wn = np.sqrt(prod)
        m.zeta = -s.real / (2 * m.wn)
    else:
        m.wn, m.zeta = np.nan, np.nan
        lam = max(p1.real, p2.real)
        m.t2 = np.log(2) / lam if lam > 0 else np.inf
    return m


def _real_mode(name, p):
    m = Mode(name, [complex(p)], kind="real")
    r = complex(p).real
    if r < -1e-12:
        m.tau = -1.0 / r
        m.t2 = np.inf
    elif r > 1e-12:
        m.tau = -1.0 / r
        m.t2 = np.log(2) / r
    else:
        m.tau = np.inf
        m.t2 = np.inf
    return m


def long_modes(A):
    ev = np.linalg.eigvals(A)
    ev = ev[np.argsort(-np.abs(ev))]
    return {"sp": _pair_mode("Short period", ev[0], ev[1]),
            "ph": _pair_mode("Phugoid", ev[2], ev[3])}


def lat_modes(A):
    ev = np.linalg.eigvals(A)
    cpx = [e for e in ev if e.imag > 1e-6]
    real = sorted([e for e in ev if abs(e.imag) <= 1e-6], key=lambda e: -abs(e))
    modes = {}
    if len(cpx) == 1 and len(real) == 2:
        p = cpx[0]
        modes["dr"] = _pair_mode("Dutch roll", p, np.conj(p))
        modes["roll"] = _real_mode("Roll", real[0].real)
        modes["spiral"] = _real_mode("Spiral", real[1].real)
    elif len(cpx) == 2:
        # coupled roll-spiral ("lateral phugoid"): DR = higher-imag-ratio pair
        cpx = sorted(cpx, key=lambda e: -abs(e.imag) / max(abs(e), 1e-9))
        modes["dr"] = _pair_mode("Dutch roll", cpx[0], np.conj(cpx[0]))
        modes["roll"] = _pair_mode("Roll-spiral (coupled)", cpx[1], np.conj(cpx[1]))
        modes["roll"].kind = "coupled"
    else:
        # all real: largest = roll, smallest = spiral, middle two = DR
        ev_s = sorted(ev, key=lambda e: -abs(e))
        modes["roll"] = _real_mode("Roll", ev_s[0].real)
        modes["dr"] = _pair_mode("Dutch roll", ev_s[1], ev_s[2])
        modes["spiral"] = _real_mode("Spiral", ev_s[3].real)
    return modes


def second_order_poles(wn, zeta):
    if zeta >= 1:
        r = wn * np.sqrt(zeta ** 2 - 1)
        return [complex(-zeta * wn + r), complex(-zeta * wn - r)]
    w = wn * np.sqrt(1 - zeta ** 2)
    return [complex(-zeta * wn, w), complex(-zeta * wn, -w)]


def ctrb_rank(A, B, tol=1e-9):
    n = A.shape[0]
    M = np.hstack([np.linalg.matrix_power(A, k) @ B for k in range(n)])
    return np.linalg.matrix_rank(M, tol * max(1.0, np.abs(M).max()))


def obsv_rank(A, C, tol=1e-9):
    return ctrb_rank(A.T, np.atleast_2d(C).T, tol)


def pbh_report(A, B, C=None, tol=1e-7):
    """Stabilizability / detectability via PBH test on unstable eigenvalues."""
    n = A.shape[0]
    out = {"controllable": ctrb_rank(A, B) == n, "uncontrollable_unstable": []}
    for lam in np.linalg.eigvals(A):
        if lam.real >= -1e-9:
            M = np.hstack([lam * np.eye(n) - A, B])
            if np.linalg.matrix_rank(M, tol) < n:
                out["uncontrollable_unstable"].append(lam)
    out["stabilizable"] = not out["uncontrollable_unstable"]
    if C is not None:
        C = np.atleast_2d(C)
        out["observable"] = obsv_rank(A, C) == n
        out["undetectable_unstable"] = []
        for lam in np.linalg.eigvals(A):
            if lam.real >= -1e-9:
                M = np.vstack([lam * np.eye(n) - A, C])
                if np.linalg.matrix_rank(M, tol) < n:
                    out["undetectable_unstable"].append(lam)
        out["detectable"] = not out["undetectable_unstable"]
    return out
