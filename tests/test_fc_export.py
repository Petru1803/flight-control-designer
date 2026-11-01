"""Flight-controller export test (Course Jet, generic data):
 1. design pitch (PID, LQG) and roll (PID, H-inf) controllers that only use the sensors;
 2. closed loop of the exported discrete controller (sensor inputs, normalized torque outputs,
    trim offsets) with the ZOH aircraft reproduces the application's digital step response;
 3. the generated C code, compiled with MSVC, gives the same outputs as the Python reference;
 4. writes the PX4 module and the parameter files.
Usage: test_fc_export.py <output folder>"""
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from core import autopilot as ap, fccode, fcexport as fx, lti  # noqa: E402
from tests.smoke import latm, lm  # noqa: E402

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "_fc_out")
os.makedirs(OUT, exist_ok=True)
VCVARS = r"C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
DEG = fx.DEG
dmax = {"de": 25 * DEG, "da": 20 * DEG, "dr": 25 * DEG, "dT": np.inf}
act = ap.ActuatorSpec(0.05, 0.02)
theta0 = -0.207 * DEG
fails = []


def check(cond, msg):
    print(("OK   " if cond else "FAIL ") + msg)
    if not cond:
        fails.append(msg)


def closed_loop_check(D, dc):
    """ZOH aircraft + one-sample delay + exported controller vs digital_check."""
    plant, Ts = D.plant, D.Ts
    Phi, Gam = lti.c2d(plant.Aol, plant.B, Ts)
    st = D.settings
    N = int(round(st.T / Ts)) + 1
    x = np.zeros(plant.n)
    ref = st.ref[0] * DEG
    sp = np.full(N, ref + dc.r_offset)
    u_prev = np.zeros(plant.B.shape[1])
    ys = np.zeros(N)
    # controller state as in the generated code (engaged, prefilter at rest)
    xs, rf = np.zeros(dc.A.shape[0]), 0.0
    inv = []
    for i, j in enumerate(plant.js):
        role = {"fc_pitch": ["de"], "fc_roll": ["da", "dr"]}[plant.mode][i]
        inv.append(fx.surface_sign(role, plant.design.B, j) * D.dmax[plant.input_names[j]])
    for k in range(N):
        ys[k] = x[plant.ks[0]]
        yraw = np.array([x[m] for m in plant.meas]) + np.asarray(dc.y_offsets)
        rk = sp[k] - dc.r_offset
        rin = rf if dc.a_pf > 0 else rk
        u = dc.C @ xs + dc.Dr[:, 0] * rin + dc.Dy @ (yraw - np.asarray(dc.y_offsets)) + np.asarray(dc.trim)
        xs = dc.A @ xs + dc.Br[:, 0] * rin + dc.By @ (yraw - np.asarray(dc.y_offsets))
        if dc.a_pf > 0:
            rf = dc.a_pf * rf + (1 - dc.a_pf) * rk
        delta = np.zeros(plant.B.shape[1])
        for i, j in enumerate(plant.js):
            delta[j] = inv[i] * (u[i] - dc.trim[i])
        x = Phi @ x + Gam @ u_prev          # one-sample computational delay
        u_prev = delta
    yd = D.digital["y"][:, 0] * DEG
    err = np.max(np.abs(ys - yd)) / abs(ref)
    return err


designs = {}
for axis, struct in (("pitch", "PID"), ("pitch", "LQG"), ("roll", "PID"), ("roll", "Hinf")):
    D = fx.design(axis, struct, lm, latm, dmax, "PX4", act, Ts=0.01)
    r = D.result
    check(all(r.feasible.values()) and D.digital["stable"],
          f"{axis} {struct}: feasible {r.feasible}, PM {r.margins['pm']:.1f}, digital PM {D.digital['margins']['pm']:.1f}")
    fx.check_measured(D.plant, D.ctrl)
    if struct == "PID":
        # the parameters must make sense on the firmware: inside the search ranges, integrator and
        # yaw damper really present, no reference scaling (the firmware has none)
        pr = D.ctrl.params
        lo, hi = fx.TC_RANGE["PX4"]
        check(lo <= pr["TC"] <= hi and pr["P"] > 1e-4 and pr["I"] > 1e-4 and pr["FF"] >= 0
              and pr.get("YAW_P", 1.0) > 1e-3 and float(np.ravel(D.ctrl.Nbar)[0]) == 1.0,
              f"{axis} PID parameters are physical: { {k: float(f'{v:.3g}') for k, v in pr.items()} }")
    dc = fx.to_discrete(D, theta0 if axis == "pitch" else 0.0)
    err = closed_loop_check(D, dc)
    check(err < 1e-6, f"{axis} {struct}: exported controller reproduces the digital step response (rel err {err:.1e})")
    designs[(axis, struct)] = (D, dc)

# ---- generated C vs Python reference
dcs = [designs[("pitch", "LQG")][1], designs[("roll", "Hinf")][1]]
dcs[0].trim = [0.05]
meta = {"name": "Course Jet test", "V0": lm.V0, "dmax": dmax, "structures": {"pitch": "LQG", "roll": "Hinf"}}
files = fccode.px4_module(dcs, meta)
mod_dir = os.path.join(OUT, "fw_ss_control")
fccode.write_files(mod_dir, files)
N = 400
t = np.arange(N) * 0.01
rng = np.random.default_rng(3)
sp_p = theta0 + 2 * DEG * (t > 0.5) - 1 * DEG * (t > 2.5)
Yp = np.column_stack([lm.V0 + 2 * np.sin(0.7 * t), 0.05 * np.sin(3 * t) + 0.002 * rng.standard_normal(N),
                      theta0 + 0.03 * np.sin(1.1 * t)])
sp_r = 10 * DEG * (t > 0.3)
Yr = np.column_stack([0.1 * np.sin(2 * t), 0.02 * np.cos(1.3 * t), 0.15 * np.sin(0.9 * t)])
ref_p = fx.simulate_discrete(dcs[0], sp_p, Yp)
ref_r = fx.simulate_discrete(dcs[1], sp_r, Yr)


def carr(name, M):
    M = np.atleast_2d(M)
    return f"static const float {name}[{M.shape[0]}][{M.shape[1]}] = {{" + ",".join(
        "{" + ",".join(f"{v:.9e}f" for v in row) + "}" for row in M) + "};\n"


main_c = ('#include <stdio.h>\n#include "fc_controller.h"\n'
          + carr("SPP", sp_p.reshape(-1, 1)) + carr("YP", Yp) + carr("SPR", sp_r.reshape(-1, 1)) + carr("YR", Yr)
          + f"int main(void) {{\n fc_pitch_t p; fc_roll_t r; fc_pitch_reset(&p); fc_roll_reset(&r);\n"
          f" for (int k = 0; k < {N}; k++) {{ float up[1], ur[2];\n"
          "  fc_pitch_step(&p, SPP[k][0], YP[k], up); fc_roll_step(&r, SPR[k][0], YR[k], ur);\n"
          '  printf("%.9g %.9g %.9g\\n", up[0], ur[0], ur[1]); }\n return 0;\n}\n')
with open(os.path.join(mod_dir, "test_main.c"), "w", encoding="utf-8") as f:
    f.write(main_c)
if os.path.exists(VCVARS):
    bat = os.path.join(mod_dir, "build_test.bat")
    with open(bat, "w") as f:
        f.write(f'@echo off\ncall "{VCVARS}" >nul\ncd /d "{mod_dir}"\n'
                'cl /nologo /W4 /O2 /Fe:fc_test.exe test_main.c fc_controller.c >build.log 2>&1\n'
                'if errorlevel 1 exit /b 1\n"%~dp0fc_test.exe"\n')
    res = subprocess.run(["cmd", "/c", bat], capture_output=True, text=True)
    log = open(os.path.join(mod_dir, "build.log"), encoding="utf-8", errors="ignore").read()
    warn = [ln for ln in log.splitlines() if "warning" in ln.lower()]
    check(res.returncode == 0, f"C code compiles with MSVC /W4 ({len(warn)} warnings)")
    for ln in warn[:5]:
        print("     ", ln.strip())
    if res.returncode == 0:
        got = np.loadtxt(res.stdout.splitlines())
        exp = np.column_stack([ref_p[:, 0], ref_r[:, 0], ref_r[:, 1]])
        err = np.max(np.abs(got - exp))
        check(err < 2e-4, f"C outputs match the Python reference (max abs diff {err:.1e}, outputs up to "
                          f"{np.max(np.abs(exp)):.2f})")
else:
    print("SKIP C compilation (MSVC not found)")

# ---- parameter files
for fw in ("PX4", "ArduPilot"):
    lat_pid = fx.design("roll", "PID", lm, latm, dmax, fw, act, Ts=0.01) if fw == "ArduPilot" else designs[("roll", "PID")][0]
    prm = fx.pid_parameters({"pitch": designs[("pitch", "PID")][0], "roll": lat_pid}, fw, lm.V0, 9000)
    path = os.path.join(OUT, "course_jet." + ("params" if fw == "PX4" else "param"))
    fx.write_params(path, prm, fw, ["test"])
    check(os.path.getsize(path) > 100, f"{fw} parameter file written ({len(prm)} parameters)")
print("FC EXPORT TEST " + ("OK" if not fails else f"FAILED ({len(fails)})"))
