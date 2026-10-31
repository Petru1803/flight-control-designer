"""Small LTI helpers: frequency response, margins, step simulation, H2 norm."""
import numpy as np
import scipy.linalg


def freqresp(A, B, C, D, w):
    """SISO frequency response L(jw) for arrays of w."""
    A = np.asarray(A, float)
    n = A.shape[0]
    B = np.asarray(B, float).reshape(n, -1)[:, 0]
    C = np.asarray(C, float).reshape(-1, n)[0]
    D = float(np.asarray(D).ravel()[0]) if np.size(D) else 0.0
    if n == 0:
        return np.full(len(w), D, complex)
    w = np.asarray(w, float)
    # fast path: modal decomposition (vectorised over frequency)
    lam, V = np.linalg.eig(A)
    if np.linalg.cond(V) < 1e8:
        r = (C @ V) * np.linalg.solve(V, B.astype(complex))
        return (r[None, :] / (1j * w[:, None] - lam[None, :])).sum(axis=1) + D
    # robust path: Hessenberg form keeps the solve well conditioned; all frequencies are
    # solved in one batched call (same maths as a per-frequency loop, ~50x faster)
    H, Q = scipy.linalg.hessenberg(A, calc_q=True)
    return _batched_resolvent(H, Q.T @ B, C @ Q, D, 1j * w)


def _batched_resolvent(A, b, c, D, s, chunk=4096):
    """c (s_k I - A)^-1 b + D for every s_k, solved in batches."""
    n = A.shape[0]
    out = np.empty(len(s), complex)
    I = np.eye(n)
    for k in range(0, len(s), chunk):
        sk = s[k:k + chunk]
        M = sk[:, None, None] * I[None] - A[None]
        x = np.linalg.solve(M, np.broadcast_to(b.astype(complex), (len(sk), n))[..., None])[..., 0]
        out[k:k + chunk] = x @ c + D
    return out


def freqresp_d(A, B, C, D, w, Ts):
    """SISO frequency response of a discrete system at z = exp(j w Ts)."""
    A = np.asarray(A, float)
    n = A.shape[0]
    B = np.asarray(B, float).reshape(n, -1)[:, 0]
    C = np.asarray(C, float).reshape(-1, n)[0]
    D = float(np.asarray(D).ravel()[0]) if np.size(D) else 0.0
    z = np.exp(1j * np.asarray(w, float) * Ts)
    lam, V = np.linalg.eig(A)
    if np.linalg.cond(V) < 1e8:
        r = (C @ V) * np.linalg.solve(V, B.astype(complex))
        return (r[None, :] / (z[:, None] - lam[None, :])).sum(axis=1) + D
    return _batched_resolvent(A, B, C, D, z)


def margins(A, B, C, D, wmin=1e-4, wmax=1e4, npts=3000):
    """Classical margins of a SISO loop L (negative feedback).
    Returns dict: gm_db (min |dB| over phase crossovers, inf if none), pm (deg, min),
    dm (s, min over gain crossovers), wcg, wcp."""
    w = np.logspace(np.log10(wmin), np.log10(wmax), npts)
    return margins_from_response(w, freqresp(A, B, C, D, w))


def margins_d(A, B, C, D, Ts, wmin=1e-4, npts=3000):
    """Margins of a discrete SISO loop, frequency grid up to the Nyquist frequency."""
    w = np.logspace(np.log10(wmin), np.log10(np.pi / Ts * 0.999), npts)
    return margins_from_response(w, freqresp_d(A, B, C, D, w, Ts))


def freqresp_mimo(A, B, C, w):
    """Frequency response G(jw) of (A, B, C) for an array of w -> (len(w), p, m)."""
    A = np.asarray(A, float)
    lam, V = np.linalg.eig(A)
    w = np.asarray(w, float)
    if np.linalg.cond(V) < 1e8:
        CV = C @ V
        ViB = np.linalg.solve(V, B.astype(complex))
        d = 1.0 / (1j * w[:, None] - lam[None, :])             # (N, n)
        return np.einsum("pn,kn,nm->kpm", CV, d, ViB)
    I = np.eye(A.shape[0])
    M = (1j * w)[:, None, None] * I[None] - A[None]
    X = np.linalg.solve(M, np.broadcast_to(B.astype(complex), (len(w),) + B.shape))
    return C[None] @ X


def mu_diag_upper(M, grid=41, sweeps=3):
    """Upper bound of the structured singular value for a diagonal complex perturbation
    (independent variation in every loop): min over positive diagonal D of
    sigma_max(D M D^-1), at every frequency (coordinate search on log d_i).
    M: (N, q, q). Returns (N,)."""
    N, q, _ = M.shape
    if q == 1:
        return np.abs(M[:, 0, 0])
    logd = np.zeros((N, q))
    steps = np.linspace(-3, 3, grid)
    best = np.linalg.norm(M, 2, axis=(1, 2))
    for sweep in range(sweeps):
        span = 3.0 / (4 ** sweep)
        for i in range(1, q):                  # d_0 = 1 fixes the scale
            cand = logd[:, i][:, None] + steps[None, :] * span / 3   # (N, grid)
            vals = np.empty((N, grid))
            for g in range(grid):
                ld = logd.copy(); ld[:, i] = cand[:, g]
                d = np.exp(ld)
                Ms = M * d[:, :, None] / d[:, None, :]
                vals[:, g] = np.linalg.norm(Ms, 2, axis=(1, 2))
            k = np.argmin(vals, axis=1)
            logd[:, i] = cand[np.arange(N), k]
            best = np.minimum(best, vals[np.arange(N), k])
    return best


def disk_margin(L):
    """Balanced (skew 0) disk margin from the loop response L (N, q, q), negative feedback.
    alpha = 1 / max_w sigma_max(S - I/2), S = (I + L)^-1.
    Returns alpha, disk gain margin [dB] (symmetric +/-) and disk phase margin [deg]."""
    q = L.shape[1]
    I = np.eye(q)
    try:
        M = np.linalg.inv(I[None] + L) - I[None] / 2          # (N, q, q)
    except np.linalg.LinAlgError:
        return 0.0, 0.0, 0.0
    peak = float(np.max(mu_diag_upper(M)))
    alpha = 1.0 / peak if peak > 0 else np.inf
    if alpha >= 2:
        dgm = np.inf
    else:
        dgm = 20 * np.log10((2 + alpha) / (2 - alpha))
    dpm = np.degrees(2 * np.arctan(alpha / 2)) if np.isfinite(alpha) else 180.0
    return float(alpha), float(dgm), float(dpm)


def margins_from_response(w, L):
    mag = np.abs(L)
    ph = np.unwrap(np.angle(L))
    res = {"gm_db": np.inf, "pm": np.inf, "dm": np.inf, "wcg": np.nan, "wcp": np.nan,
           "w": w, "L": L}
    # gain crossovers |L| = 1
    s = np.sign(mag - 1.0)
    idx = np.where(s[:-1] * s[1:] < 0)[0]
    pms, dms, wcs = [], [], []
    for i in idx:
        lw0, lw1 = np.log10(w[i]), np.log10(w[i + 1])
        m0, m1 = np.log10(mag[i]), np.log10(mag[i + 1])
        f = -m0 / (m1 - m0) if m1 != m0 else 0.5
        wc = 10 ** (lw0 + f * (lw1 - lw0))
        phc = ph[i] + f * (ph[i + 1] - ph[i])
        pm = (np.degrees(phc) + 180.0) % 360.0
        if pm > 180:
            pm -= 360
        pms.append(pm)
        wcs.append(wc)
        dms.append(np.radians(pm) / wc if pm > 0 else 0.0)
    if pms:
        k = int(np.argmin(pms))
        res["pm"], res["wcp"] = pms[k], wcs[k]
        res["dm"] = min(dms)
    # phase crossovers: Im(L)=0 with Re(L)<0
    im = L.imag
    s2 = np.sign(im)
    idx2 = np.where(s2[:-1] * s2[1:] < 0)[0]
    gms, wgs = [], []
    for i in idx2:
        f = im[i] / (im[i] - im[i + 1])
        re = L.real[i] + f * (L.real[i + 1] - L.real[i])
        if re < 0:
            gms.append(-20 * np.log10(abs(re)))
            wgs.append(w[i] + f * (w[i + 1] - w[i]))
    if gms:
        k = int(np.argmin(np.abs(gms)))
        res["gm_db"], res["wcg"] = gms[k], wgs[k]
    return res


def c2d(A, B, dt):
    n, m = A.shape[0], B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A * dt
    M[:n, n:] = B * dt
    E = scipy.linalg.expm(M)
    return E[:n, :n], E[:n, n:]


def simulate(A, B, C, D, u, dt, x0=None):
    """ZOH simulation; u is (N, m). Returns outputs (N, p) and states (N, n)."""
    Phi, Gam = c2d(A, B, dt)
    n = A.shape[0]
    N = u.shape[0]
    x = np.zeros(n) if x0 is None else np.asarray(x0, float).copy()
    X = np.empty((N, n))
    for k in range(N):
        X[k] = x
        x = Phi @ x + Gam @ u[k]
    Y = X @ C.T + u @ D.T
    return Y, X


def h2_norm(A, B, C):
    """H2 norm of a strictly proper stable system (inf if unstable)."""
    if np.any(np.linalg.eigvals(A).real >= 0):
        return np.inf
    P = scipy.linalg.solve_continuous_lyapunov(A, -B @ B.T)
    return float(np.sqrt(max(np.trace(C @ P @ C.T), 0.0)))


def step_metrics(t, y, r, tol=0.02):
    """Rise time (10-90 %), settling time (tol band), overshoot (%), final error."""
    out = {"rise": np.nan, "settle": np.nan, "overshoot": np.nan, "sse": np.nan}
    if abs(r) < 1e-12:
        out["sse"] = float(y[-1])
        band = tol * max(np.max(np.abs(y)), 1e-12)
        outside = np.where(np.abs(y) > band)[0]
        out["settle"] = float(t[outside[-1]]) if len(outside) and outside[-1] < len(t) - 1 else (np.nan if len(outside) else 0.0)
        return out
    yn = y / r
    out["sse"] = float(r - y[-1])
    out["overshoot"] = float(max(0.0, (np.max(yn) - 1.0) * 100))
    try:
        i10 = np.argmax(yn >= 0.1)
        i90 = np.argmax(yn >= 0.9)
        if yn[i90] >= 0.9 and yn[i10] >= 0.1:
            out["rise"] = float(t[i90] - t[i10])
    except ValueError:
        pass
    outside = np.where(np.abs(yn - 1.0) > tol)[0]
    if len(outside) == 0:
        out["settle"] = 0.0
    elif outside[-1] < len(t) - 1:
        out["settle"] = float(t[outside[-1] + 1])
    return out
