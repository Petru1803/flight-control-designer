"""From aerodynamic coefficients to state-space matrices.

Same formulation as dinamica_longitudinala.m / dinamica_laterala.m:
  - nonlinear longitudinal dynamics in [V alpha q theta], inputs [delta_e, T]
    (lift / drag / moment from linear coefficient models, thrust with mount angle and arm);
  - trim (alpha, delta_e, T) for given V, gamma by Newton with a numerical Jacobian;
  - A, B by central differences around the trim;
  - lateral-directional [beta p r phi], inputs [delta_a, delta_r], with Ixz coupling,
    linearised at the same trim (alpha0, theta0).
All coefficients per radian; q, p, r derivatives either w.r.t. the AVL non-dimensional rates
(qhat = q c/2V, phat = p b/2V, rhat = r b/2V) or w.r.t. the dimensional rates [rad/s].
Optional compressibility scaling for a Mach sweep, as in aero_mach.m: 1/sqrt(1-(k M)^2)."""
from dataclasses import asdict, dataclass, field

import numpy as np

G = 9.81


# ------------------------------------------------------------------------------------
# ISA atmosphere (same constants as the MATLAB scripts)
def isa(h):
    T0, p0, R, g0, L = 288.15, 101325.0, 287.05287, 9.80665, 0.0065
    if h < 11000:
        T = T0 - L * h
        p = p0 * (T / T0) ** (g0 / (R * L))
    else:
        T = 216.65
        p11 = p0 * (216.65 / T0) ** (g0 / (R * L))
        p = p11 * np.exp(-g0 * (h - 11000) / (R * T))
    rho = p / (R * T)
    a = np.sqrt(1.4 * R * T)
    return rho, a, T, p


# ------------------------------------------------------------------------------------
@dataclass
class AeroData:
    # geometry / inertia
    m: float = 37000.0
    S: float = 77.6
    cbar: float = 5.76
    b: float = 14.74
    Ixx: float = 460000.0
    Iyy: float = 3000000.0
    Izz: float = 2400000.0
    Ixz: float = 5000.0
    # propulsion
    iT_deg: float = 0.0          # thrust line angle w.r.t. body x [deg]
    zT: float = 0.0              # thrust arm [m], + below CG (nose-up moment for +T)
    # longitudinal coefficients (per rad)
    CL0: float = 0.002
    CLa: float = 3.015
    CLq: float = 3.048
    CLde: float = 0.282
    CD0: float = 0.020
    CDa: float = 0.178
    CDq: float = 0.235
    CDde: float = 0.040
    Cm0: float = 0.001
    Cma: float = 0.214
    Cmq: float = -1.420
    Cmde: float = -0.344
    # lateral-directional coefficients (per rad)
    CYb: float = 0.0016
    CYp: float = 0.10445
    CYr: float = -0.010548
    CYda: float = 0.0
    CYdr: float = 0.001
    Clb: float = -0.043848
    Clp: float = -0.240547
    Clr: float = 0.054498
    Clda: float = -0.117
    Cldr: float = 0.0
    Cnb: float = -0.005484
    Cnp: float = -0.014202
    Cnr: float = 0.006431
    Cnda: float = 0.001
    Cndr: float = -0.010
    rates_nondim: bool = True    # True: derivatives w.r.t. qhat/phat/rhat (AVL); False: per rad/s
    # flight condition
    h: float = 4500.0
    V: float = 250.0
    gamma_deg: float = 0.0
    use_mach: bool = False       # if True V = Mach * a(h)
    mach: float = 0.7
    # Mach scaling for sweeps (aero_mach.m): coefficients given at mach_ref
    k_comp: float = 1.0          # 1 = Prandtl-Glauert; 0.636 = user's AVL fit
    mach_ref: float = 0.0
    g: float = G

    def to_dict(self):
        return asdict(self)

    @staticmethod
    def from_dict(d):
        a = AeroData()
        for k, v in d.items():
            if hasattr(a, k):
                setattr(a, k, type(getattr(a, k))(v))
        return a


SCALED = ("CLa", "Cma", "Cmq", "CLde", "Cmde", "CLq", "Cnb", "Clb", "Clp", "Cnr", "CYb")


def comp_factor(M, k):
    x = 1 - (k * M) ** 2
    return 1 / np.sqrt(x) if x > 0.05 else 1 / np.sqrt(0.05)


def at_mach(ad, M):
    """Copy of the data at Mach M: lift-curve-type derivatives scaled by the
    compressibility factor relative to mach_ref (subsonic model, as aero_mach.m)."""
    out = AeroData.from_dict(ad.to_dict())
    s = comp_factor(M, ad.k_comp) / comp_factor(ad.mach_ref, ad.k_comp)
    for k in SCALED:
        setattr(out, k, getattr(ad, k) * s)
    out.use_mach, out.mach = True, M
    return out


# ------------------------------------------------------------------------------------
def _rates(ad, V):
    """Factors converting dimensional rates into the variables the derivatives refer to."""
    if ad.rates_nondim:
        return ad.cbar / (2 * V), ad.b / (2 * V)
    return 1.0, 1.0


def long_dyn(x, u, ad, rho):
    V, al, q, th = x
    de, T = u
    qbar = 0.5 * rho * V ** 2
    kq, _ = _rates(ad, V)
    qh = q * kq
    CL = ad.CL0 + ad.CLa * al + ad.CLq * qh + ad.CLde * de
    CD = ad.CD0 + ad.CDa * al + ad.CDq * qh + ad.CDde * de
    Cm = ad.Cm0 + ad.Cma * al + ad.Cmq * qh + ad.Cmde * de
    L = qbar * ad.S * CL
    D = qbar * ad.S * CD
    M = qbar * ad.S * ad.cbar * Cm
    Xa = -D * np.cos(al) + L * np.sin(al)
    Za = -D * np.sin(al) - L * np.cos(al)
    iT = np.radians(ad.iT_deg)
    Xt, Zt, Mt = T * np.cos(iT), -T * np.sin(iT), T * ad.zT
    X, Z, Mtot = Xa + Xt, Za + Zt, M + Mt
    ub, wb = V * np.cos(al), V * np.sin(al)
    ud = X / ad.m - q * wb - ad.g * np.sin(th)
    wd = Z / ad.m + q * ub + ad.g * np.cos(th)
    return np.array([(ub * ud + wb * wd) / V, (ub * wd - wb * ud) / V ** 2, Mtot / ad.Iyy, q])


def lat_dyn(x, u, ad, rho, V, alpha0, theta0):
    be, p, r, ph = x
    da, dr = u
    qbar = 0.5 * rho * V ** 2
    _, kb = _rates(ad, V)
    ph_, rh_ = p * kb, r * kb
    CY = ad.CYb * be + ad.CYp * ph_ + ad.CYr * rh_ + ad.CYda * da + ad.CYdr * dr
    Cl = ad.Clb * be + ad.Clp * ph_ + ad.Clr * rh_ + ad.Clda * da + ad.Cldr * dr
    Cn = ad.Cnb * be + ad.Cnp * ph_ + ad.Cnr * rh_ + ad.Cnda * da + ad.Cndr * dr
    Y = qbar * ad.S * CY
    Lm = qbar * ad.S * ad.b * Cl
    Nm = qbar * ad.S * ad.b * Cn
    J = np.array([[ad.Ixx, -ad.Ixz], [-ad.Ixz, ad.Izz]])
    pr = np.linalg.solve(J, [Lm, Nm])
    bed = Y / (ad.m * V) + p * np.sin(alpha0) - r * np.cos(alpha0) + (ad.g / V) * np.cos(theta0) * ph
    phd = p + r * np.tan(theta0)
    return np.array([bed, pr[0], pr[1], phd])


def jac(f, x0, u0, clean=False):
    n, m = len(x0), len(u0)
    A, B = np.zeros((n, n)), np.zeros((n, m))
    for i in range(n):
        d = np.zeros(n); d[i] = 1e-6 * max(1.0, abs(x0[i]))
        A[:, i] = (f(x0 + d, u0) - f(x0 - d, u0)) / (2 * d[i])
    for i in range(m):
        d = np.zeros(m); d[i] = 1e-6 * max(1.0, abs(u0[i]))
        B[:, i] = (f(x0, u0 + d) - f(x0, u0 - d)) / (2 * d[i])
    if clean:
        A, B = _clean(A), _clean(B)
    return A, B


def _clean(M, rel=1e-10):
    """Zero the round-off left by the central differences (entries ~1e-10 smaller than
    the largest entry of their column), e.g. 7e-13 next to g = 9.81."""
    M = M.copy()
    for j in range(M.shape[1]):
        mx = np.abs(M[:, j]).max()
        M[np.abs(M[:, j]) < rel * mx, j] = 0.0
    return M


@dataclass
class LinResult:
    V: float
    h: float
    rho: float
    mach: float
    alpha0: float
    de0: float
    T0: float
    theta0: float
    trim_residual: float
    trim_iterations: int
    A_long: np.ndarray
    B_long: np.ndarray
    A_lat: np.ndarray
    B_lat: np.ndarray
    info: dict = field(default_factory=dict)
    warnings: list = field(default_factory=list)


def trim(ad, V, rho, gam):
    def res(z):
        al, de, T = z
        xd = long_dyn(np.array([V, al, 0.0, al + gam]), np.array([de, T]), ad, rho)
        return xd[:3]
    z = np.array([np.radians(3), 0.0, 0.15 * ad.m * ad.g])
    it = 0
    for it in range(1, 81):
        r = res(z)
        if np.linalg.norm(r) < 1e-10:
            break
        J = np.zeros((3, 3))
        for i in range(3):
            dz = np.zeros(3); dz[i] = 1e-7 * max(1.0, abs(z[i]))
            J[:, i] = (res(z + dz) - res(z - dz)) / (2 * dz[i])
        try:
            z = z - np.linalg.solve(J, r)
        except np.linalg.LinAlgError:
            break
    return z, float(np.linalg.norm(res(z))), it


def linearise(ad, de_limit_deg=30.0):
    rho, a, _, _ = isa(ad.h)
    V = ad.mach * a if ad.use_mach else ad.V
    M = V / a
    gam = np.radians(ad.gamma_deg)
    z, resid, it = trim(ad, V, rho, gam)
    al0, de0, T0 = z
    th0 = al0 + gam
    x0 = np.array([V, al0, 0.0, th0]); u0 = np.array([de0, T0])
    Al, Bl = jac(lambda x, u: long_dyn(x, u, ad, rho), x0, u0, clean=True)
    Aa, Ba = jac(lambda x, u: lat_dyn(x, u, ad, rho, V, al0, th0), np.zeros(4), np.zeros(2), clean=True)
    w = []
    if resid > 1e-6:
        w.append(f"Trim did not converge (residual {resid:.2e}): check CL0/Cm0/Cmde or the speed.")
    if abs(np.degrees(de0)) > de_limit_deg:
        w.append(f"Trim elevator {np.degrees(de0):.1f} deg exceeds +/-{de_limit_deg:g} deg.")
    if T0 < 0:
        w.append(f"Trim thrust is negative ({T0:.0f} N): the aircraft would need to descend (gamma < 0).")
    if abs(np.degrees(al0)) > 15:
        w.append(f"Trim angle of attack {np.degrees(al0):.1f} deg is outside the linear-aero range.")
    if ad.Cma > 0:
        w.append("Cm_alpha > 0: statically unstable in pitch (the SAS must stabilise it).")
    if ad.Cnb < 0:
        w.append("Cn_beta < 0: directionally unstable (weathercock instability).")
    if ad.Clb > 0:
        w.append("Cl_beta > 0: negative dihedral effect (spiral / roll coupling problems).")
    if M > 0.8:
        w.append(f"Mach {M:.2f}: linear subsonic aerodynamics and the compressibility factor are uncertain "
                 "above ~0.8 (transonic).")
    qbar = 0.5 * rho * V ** 2
    info = {"Mach": M, "rho [kg/m3]": rho, "qbar [Pa]": qbar, "CL trim": ad.m * ad.g * np.cos(gam) / (qbar * ad.S),
            "static margin [% MAC]": -100 * ad.Cma / ad.CLa if ad.CLa else np.nan,
            "n/alpha [g/rad]": qbar * ad.S * ad.CLa / (ad.m * ad.g),
            "Z_alpha / V  (A[2,2])": Al[1, 1], "M_alpha  (A[3,2])": Al[2, 1], "M_q  (A[3,3])": Al[2, 2],
            "M_de  (B[3,1])": Bl[2, 0], "X_V  (A[1,1])": Al[0, 0],
            "Y_beta/V  (A[1,1])": Aa[0, 0], "L_beta  (A[2,1])": Aa[1, 0], "L_p  (A[2,2])": Aa[1, 1],
            "N_beta  (A[3,1])": Aa[2, 0], "N_r  (A[3,3])": Aa[2, 2], "L_da  (B[2,1])": Ba[1, 0],
            "N_dr  (B[3,2])": Ba[2, 1]}
    return LinResult(V, ad.h, rho, M, al0, de0, T0, th0, resid, it, Al, Bl, Aa, Ba, info, w)


def mach_sweep(ad, machs):
    """Linearise at every Mach (same altitude), coefficients scaled with the compressibility
    factor relative to ad.mach_ref."""
    return [linearise(at_mach(ad, M)) for M in machs]
