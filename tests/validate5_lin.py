"""Compares core.aero_lin with the user's dinamica_longitudinala.m / dinamica_laterala.m (run
non-interactively with their default values). Needs MATLAB. Usage: validate5_lin.py <work folder>"""
import os
import re
import subprocess
import sys

import numpy as np
import scipy.io

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from core import aero_lin as al  # noqa: E402

work = sys.argv[1]
os.makedirs(work, exist_ok=True)
root = os.path.dirname(os.path.dirname(HERE))
for src, name in (("dinamica_longitudinala.m", "lin_long_auto"), ("dinamica_laterala.m", "lin_lat_auto")):
    t = open(os.path.join(root, src), encoding="utf-8").read()
    t = re.sub(r"^function lin_(long|lat)", f"function {name}", t)
    t = t.replace("s = input(sprintf('  %-42s [%g]: ', prompt, def), 's');", "s = '';").replace("clc;", "")
    open(os.path.join(work, name + ".m"), "w", encoding="utf-8").write(t)
matlab = r"C:\Program Files\MATLAB\R2025a\bin\matlab.exe"
subprocess.run([matlab, "-batch", f"cd('{work}'); lin_long_auto; lin_lat_auto; "
                "save('ml_lin.mat','A_long','B_long','x0_long','u0_long','A_lat','B_lat');"], check=True,
               capture_output=True)
ml = scipy.io.loadmat(os.path.join(work, "ml_lin.mat"))
ad = al.AeroData()
r = al.linearise(ad)
rho = al.isa(ad.h)[0]
Aa, Ba = al.jac(lambda x, u: al.lat_dyn(x, u, ad, rho, 250.0, np.radians(3.4), np.radians(3.4)),
                np.zeros(4), np.zeros(2))


def rel(a, b):
    return float(np.max(np.abs(a - b)) / np.max(np.abs(b)))


print(f"trim alpha {np.degrees(r.alpha0):.4f} deg / MATLAB {np.degrees(ml['x0_long'][1, 0]):.4f} deg")
print(f"A_long rel diff {rel(r.A_long, ml['A_long']):.1e}   B_long(de) {rel(r.B_long[:, 0], ml['B_long'][:, 0]):.1e}"
      f"   B_long(T) {rel(r.B_long[:, 1], ml['B_long'][:, 1]):.1e}")
print(f"A_lat rel diff {rel(Aa, ml['A_lat']):.1e}   B_lat {rel(Ba, ml['B_lat']):.1e}")
