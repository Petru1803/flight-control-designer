"""Shared test inputs: the Course Jet, a generic business jet with invented data (9000 m, Mach 0.70),
trimmed and linearised with core/aero_lin.py. The longitudinal model is given in [u w q theta] to
exercise the nz conversion; the gain-schedule points are a Mach sweep in [V alpha q theta]."""
import numpy as np

V0 = 212.6553            # m/s
MACH = 0.700
ALT = 9000            # m
CONSTS = "alpha0 = -0.206655*pi/180"
CLS, CAT = "II-L", "B"

A_LONG = """[-0.008365087  -0.01483637  0.7609885  -g*cos(alpha0);
 -0.09605828  -1.05253  210.9852  -g*sin(alpha0);
 -0.0004083461  -0.04132018  -0.96847  0;
 0  0  1  0]"""
B_LONG = """[-0.05851164  7.692308e-05;
 -16.22244  0;
 -13.18082  2.5e-06;
 0  0]"""
A_LAT = """[-0.1334999  -0.00360681  -0.9971237  0.04613069;
 -20.61409  -3.580298  0.9248707  0;
 6.336297  -0.1350756  -0.3623205  0;
 0  1  -0.003606834  0]"""
B_LAT = """[0  0.02860711;
 25.32407  3.928504;
 0.2252906  -3.858101;
 0  0]"""

# Mach sweep at 9000 m: (Mach, V, A_long [V alpha q theta], elevator column of B_long)
SCHEDULE = [
    (0.50, 151.90,
     np.array([[-0.008856651, 3.578824, 0, -9.81], [-0.0008483165, -0.8695691, 0.9909388, 0], [-0.0002880178, -5.176836, -0.7987806, 0], [0, 0, 1, 0]]),
     np.array([[0], [-0.06291933], [-7.765254], [0]])),
    (0.60, 182.28,
     np.array([[-0.008211706, 0.8632704, 0, -9.81], [-0.0005900594, -1.127957, 0.990191, 0], [-0.0002668948, -8.069889, -1.037646, 0], [0, 0, 1, 0]]),
     np.array([[0], [-0.08173461], [-12.10483], [0]])),
    (0.70, 212.66,
     np.array([[-0.008096162, -2.354688, 0, -9.81], [-0.0004339552, -1.47284, 0.9890117, 0], [-0.0002631261, -12.30456, -1.356129, 0], [0, 0, 1, 0]]),
     np.array([[0], [-0.1068213], [-18.45684], [0]])),
]


def fill_course_jet(w, actuators=True):
    """Types the Course Jet into the Model tab of a MainWindow."""
    mt = w.model_tab
    mt.name.setText("Course Jet test")
    mt.cls.setCurrentText(CLS); mt.cat.setCurrentText(CAT)
    mt.V0.setValue(V0); mt.mach.setValue(MACH); mt.alt.setValue(ALT)
    mt.consts.setText(CONSTS)
    mt.Along.setPlainText(A_LONG)
    mt.Blong.setPlainText(B_LONG)
    mt.Alat.setPlainText(A_LAT)
    mt.Blat.setPlainText(B_LAT)
    if actuators:
        mt.act_tau.setValue(0.05); mt.act_delay.setValue(0.02)
