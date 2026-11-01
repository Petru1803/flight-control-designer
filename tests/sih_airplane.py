"""Python replica of PX4's SIH fixed-wing simulator (src/modules/simulation/simulator_sih, PX4 v1.17.0,
airframe 10041_sihsim_airplane): flat-plate aerodynamic segments, propeller slipstream on the tail,
rigid body. Used to trim and linearise the SITL aircraft, so that controllers designed in the app can be
flown in PX4 SITL on exactly that aircraft.

Inputs follow SIH: roll_cmd, pitch_cmd, yaw_cmd in [-1, 1] (= normalized torque, the control allocation of
the airframe maps them 1:1 to the servos), throttle in [0, 1]."""
import numpy as np

FLAP_MAX = np.pi / 12          # 15 deg
SPAN, MAC, RP = 0.86, 0.21, 0.1
MASS, T_MAX, KDV, KDW = 0.3, 6.0, 0.2, 0.025
I = np.array([[0.00402, 0, 0.00046], [0, 0.0144, 0], [0.00046, 0, 0.0177]])
G = 9.80665
H0 = 489.4

AR_TAB = [0.1666, 0.333, 0.4, 0.5, 1.0, 1.25, 2.0, 3.0, 4.0, 6.0]
AFS_TAB = [49.00, 54.00, 56.00, 48.00, 40.00, 29.00, 27.00, 25.00, 24.00, 22.00, 22.00, 20.00]
ETA_POLY = [0.0535, -0.2688, 0.5817]
KV = np.pi
CD0, CD90 = 0.04, 1.98
ALPHA_BLEND = np.pi / 18


def _lkt(xt, yt, x):
    n = len(xt)            # SIH uses N_TAB = 12 but only 10 AR entries: the extra y values are never reached
    if x < xt[0]:
        return yt[0]
    if x > xt[n - 1]:
        return yt[n - 1]
    i = n - 2
    while xt[i] > x:
        i -= 1
    x0, x1, y0, y1 = xt[i], xt[i + 1], yt[i], yt[i + 1]
    return y0 + (y1 - y0) / (x1 - x0) * (x - x0)


def _rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


class Seg:
    def __init__(self, span, mac, alpha0_deg, p_B, dihedral_deg=0.0, AR=-1.0, cf=0.0, prop_radius=-1.0,
                 cl_alpha=2 * np.pi):
        self.span, self.mac = span, mac
        self.a0 = np.radians(alpha0_deg)
        self.p = np.asarray(p_B, float)
        self.ar = span / mac if AR <= 0 else AR
        self.kp = cl_alpha / (1 + 2 * (self.ar + 4) / (self.ar * (self.ar + 2)))
        self.kn = 0.41 * (1 - np.exp(-17 / self.ar))
        afs = np.radians(_lkt(AR_TAB, AFS_TAB, self.ar))
        self.amax, self.amin = afs, -afs
        self.cf = min(max(cf, 0.0), mac)
        self.C = _rot_x(np.radians(dihedral_deg))
        self.rp = prop_radius

    def fCL(self, a, fte=1.0, fle=1.0):
        return 0.25 * (1 + np.sqrt(fte)) ** 2 * (self.kp * np.sin(a) * np.cos(a) ** 2
                                                + fle ** 2 * KV * abs(np.sin(a)) * np.sin(a) * np.cos(a))

    def fCM(self, a, fte=1.0, fle=1.0):
        return (-0.25 * (1 + np.sqrt(fte)) ** 2 * 0.0625 * (-1 + 6 * np.sqrt(fte) - 5 * fte) * self.kp
                * np.sin(a) * np.cos(a) + 0.17 * fle ** 2 * KV * abs(np.sin(a)) * np.sin(a))

    @staticmethod
    def solve_alpha_eff(Kp, Kv, dCL, a0):
        a = a0
        for _ in range(3):
            s, c = np.sin(a), np.cos(a)
            a = a - (-Kp * s * c * c - Kv * abs(s) * s * c - dCL) / (
                Kv * abs(s) * s * s - Kv * abs(s) * c * c - Kp * c ** 3 + 2 * Kp * c * s * s
                - Kv * np.sign(s) * c * c * s)
        return a

    def high(self, a, d):
        mac_eff = np.sqrt((self.mac - self.cf) ** 2 + self.cf ** 2 + 2 * (self.mac - self.cf) * self.cf * np.cos(abs(d)))
        a = a + np.arcsin(self.cf / mac_eff * np.sin(d))
        cd90 = CD90 + 0.21 * d - 0.0426 * d * d
        CN = cd90 * np.sin(a) * (1 / (0.56 + 0.44 * np.sin(abs(a))) - self.kn)
        CT = 0.5 * CD0 * np.cos(a)
        return CN * np.cos(a) - CT * np.sin(a), CN * np.sin(a) + CT * np.cos(a), \
            -CN * (0.25 - 7 / 40 * (1 - 2 / np.pi * abs(a)))

    def coeffs(self, a, d):
        if self.cf / self.mac < 0.999:
            da = min(abs(d), np.radians(70))
            eta = da * da * ETA_POLY[0] + da * ETA_POLY[1] + ETA_POLY[2]
            th = np.arccos(2 * self.cf / self.mac - 1)
            tau = 1 - (th - np.sin(th)) / np.pi
            dCL = self.kp * tau * eta * d
            dCLmax = (1 - self.cf / self.mac) * dCL
            alf0eff = self.solve_alpha_eff(self.kp, KV, dCL, self.a0)
            ae = a - alf0eff
            CLmax = self.fCL(self.amax - self.a0) + dCLmax
            aemax = alf0eff - self.solve_alpha_eff(self.kp, KV, CLmax, self.amax - self.a0)
            CLmin = self.fCL(self.amin - self.a0) + dCLmax
            aemin = alf0eff - self.solve_alpha_eff(self.kp, KV, CLmin, self.amin - self.a0)
        else:
            ae = a + d
            aemax, aemin = self.amax, self.amin
        CLh, CDh, CMh = self.high(ae, d)
        CL_ = self.fCL(ae)
        CD_ = CD0 + abs(CLh * np.tan(ae))      # SIH uses the high-aoa _CL here (set just before)
        CM_ = -self.fCM(ae)
        if ae > 0:
            fb = 0.5 * (1 - np.tanh(4 * (ae - aemax) / ALPHA_BLEND))
        else:
            fb = 0.5 * (1 - np.tanh(-4 * (ae - aemin) / ALPHA_BLEND))
        return CL_ * fb + CLh * (1 - fb), CD_ * fb + CDh * (1 - fb), CM_ * fb + CMh * (1 - fb)

    def aero(self, v_B, w_B, rho, d=0.0, thrust=0.0):
        vS = self.C.T @ (v_B + np.cross(w_B, self.p))
        if self.rp > 1e-4:
            vS = vS.copy()
            vS[0] += np.sqrt(2 * thrust / (rho * np.pi * self.rp ** 2))
        vxz2 = vS[0] ** 2 + vS[2] ** 2
        if vxz2 < 0.01:
            return np.zeros(3), np.zeros(3)
        a = (np.arctan2(vS[2], vS[0]) - self.a0 + np.pi) % (2 * np.pi) - np.pi
        CL, CD, CM = self.coeffs(a, d)
        q = 0.5 * rho * vxz2 * self.span * self.mac
        F = self.C @ (q * np.array([CL * np.sin(a) - CD * np.cos(a), 0.0, -CL * np.cos(a) - CD * np.sin(a)]))
        M = self.C @ (q * self.mac * np.array([0.0, CM, 0.0])) + np.cross(self.p, F)
        return F, M


WING_L = Seg(SPAN / 2, MAC, -4.0, [0, -SPAN / 4, 0], 3.0, SPAN / MAC, MAC / 3)
WING_R = Seg(SPAN / 2, MAC, -4.0, [0, SPAN / 4, 0], -3.0, SPAN / MAC, MAC / 3)
TAIL = Seg(0.3, 0.1, 0.0, [-0.4, 0, 0], 0.0, -1.0, 0.05, RP)
FIN = Seg(0.25, 0.18, 0.0, [-0.45, 0, -0.1], -90.0, -1.0, 0.12, RP)
FUS = Seg(0.2, 0.8, 0.0, [0, 0, 0], -90.0)


def rho_at(alt):
    p = 101325 * (1 - 0.0065 * alt / 288.15) ** 5.2561
    T = 288.15 - 6.5e-3 * alt
    return p / 287.04 / T


def forces(v_B, w_B, u, alt=H0):
    """u = [roll_cmd, pitch_cmd, yaw_cmd, throttle] -> body force (aero + thrust), moment."""
    rho = rho_at(alt)
    r, pch, y, th = u
    T = T_MAX * th
    F, M = np.zeros(3), np.zeros(3)
    for seg, d, t in ((WING_L, r * FLAP_MAX, 0.0), (WING_R, -r * FLAP_MAX, 0.0), (TAIL, -pch * FLAP_MAX, T),
                      (FIN, y * FLAP_MAX, T), (FUS, 0.0, 0.0)):
        f, m = seg.aero(v_B, w_B, rho, d, t)
        F += f; M += m
    F += -KDV * v_B + np.array([T, 0, 0])
    M += -KDW * w_B
    return F, M


def f6(x, u):
    """Rigid body, flat earth. x = [u v w p q r phi theta psi]."""
    v, w = x[0:3], x[3:6]
    ph, th = x[6], x[7]
    F, M = forces(v, w, u)
    g_B = G * np.array([-np.sin(th), np.sin(ph) * np.cos(th), np.cos(ph) * np.cos(th)])
    vd = F / MASS + g_B - np.cross(w, v)
    wd = np.linalg.solve(I, M - np.cross(w, I @ w))
    p, q, r = w
    phd = p + (q * np.sin(ph) + r * np.cos(ph)) * np.tan(th)
    thd = q * np.cos(ph) - r * np.sin(ph)
    psd = (q * np.sin(ph) + r * np.cos(ph)) / np.cos(th)
    return np.concatenate([vd, wd, [phd, thd, psd]])


def trim(V):
    """Level flight at true airspeed V: unknowns alpha, pitch_cmd, throttle."""
    import scipy.optimize

    def res(z):
        a, pc, thr = z
        x = np.array([V * np.cos(a), 0, V * np.sin(a), 0, 0, 0, 0, a, 0])
        d = f6(x, [0, pc, 0, thr])
        return [d[0], d[2], d[4]]
    sol = scipy.optimize.fsolve(res, [0.05, 0.0, 0.3], full_output=True, xtol=1e-12)
    a, pc, thr = sol[0]
    return a, pc, thr, float(np.max(np.abs(res(sol[0]))))


def linearise(V):
    """Longitudinal [V alpha q theta] / [pitch_cmd*FLAP_MAX, throttle] and lateral [beta p r phi] /
    [roll_cmd*FLAP_MAX, yaw_cmd*FLAP_MAX] at level trim (inputs as equivalent deflections in rad, so
    that dmax = FLAP_MAX maps them back to normalized torque)."""
    a0, pc0, thr0, resid = trim(V)
    x0 = np.array([V * np.cos(a0), 0, V * np.sin(a0), 0, 0, 0, 0, a0, 0])
    u0 = np.array([0, pc0, 0, thr0])

    def long_f(z, uu):
        Vv, al, q, th = z
        x = x0.copy()
        x[0], x[2], x[4], x[7] = Vv * np.cos(al), Vv * np.sin(al), q, th
        uf = u0.copy(); uf[1] = uu[0] / FLAP_MAX; uf[3] = uu[1]
        d = f6(x, uf)
        ud, wd = d[0], d[2]
        Vd = (x[0] * ud + x[2] * wd) / Vv
        ald = (x[0] * wd - x[2] * ud) / (Vv ** 2)
        return np.array([Vd, ald, d[4], d[7]])

    def lat_f(z, uu):
        be, p, r, ph = z
        x = x0.copy()
        Vv = V
        x[0] = Vv * np.cos(a0) * np.cos(be); x[1] = Vv * np.sin(be); x[2] = Vv * np.sin(a0) * np.cos(be)
        x[3], x[5], x[6] = p, r, ph
        uf = u0.copy(); uf[0] = uu[0] / FLAP_MAX; uf[2] = uu[1] / FLAP_MAX
        d = f6(x, uf)
        vd = d[1]
        bed = vd / (Vv * np.cos(be))
        return np.array([bed, d[3], d[5], d[6]])

    def jac(f, z0, v0):
        n, m = len(z0), len(v0)
        A, B = np.zeros((n, n)), np.zeros((n, m))
        for i in range(n):
            h = 1e-6 * max(1.0, abs(z0[i]))
            e = np.zeros(n); e[i] = h
            A[:, i] = (f(z0 + e, v0) - f(z0 - e, v0)) / (2 * h)
        for j in range(m):
            h = 1e-6 * max(1.0, abs(v0[j]))
            e = np.zeros(m); e[j] = h
            B[:, j] = (f(z0, v0 + e) - f(z0, v0 - e)) / (2 * h)
        return A, B

    A_long, B_long = jac(long_f, np.array([V, a0, 0, a0]), np.array([pc0 * FLAP_MAX, thr0]))
    A_lat, B_lat = jac(lat_f, np.zeros(4), np.zeros(2))
    return {"V": V, "alpha0": a0, "pitch_cmd0": pc0, "throttle0": thr0, "residual": resid,
            "A_long": A_long, "B_long": B_long, "A_lat": A_lat, "B_lat": B_lat}


if __name__ == "__main__":
    np.set_printoptions(precision=5, suppress=True, linewidth=140)
    for V in (12.0, 15.0, 18.0):
        r = linearise(V)
        print(f"V={V}: alpha0={np.degrees(r['alpha0']):.2f} deg pitch_cmd0={r['pitch_cmd0']:.3f} "
              f"throttle0={r['throttle0']:.3f} residual={r['residual']:.1e}")
        print(" long eig", np.round(np.linalg.eigvals(r["A_long"]), 3))
        print(" lat eig ", np.round(np.linalg.eigvals(r["A_lat"]), 3))
    r = linearise(15.0)
    print("A_long\n", r["A_long"], "\nB_long\n", r["B_long"], "\nA_lat\n", r["A_lat"], "\nB_lat\n", r["B_lat"])
