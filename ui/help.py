"""User guide shown in the Help panel (F1 opens the section of the current tab)."""
from ui import theme

TAB_ANCHORS = ["model", "sas", "hq", "autopilot", "robust", "schedule", "export", "fc"]


def html():
    g, a, c, d = theme.GREEN, theme.AMBER, theme.CYAN, theme.GREEN_DIM
    css = (f"<style>body{{color:{g};background:{theme.BG};}} h2{{color:{g};border-bottom:1px solid {d};"
           f"margin-top:18px;}} h3{{color:{c};margin-bottom:2px;}} b{{color:{g};}} .k{{color:{a};}}"
           f" td{{padding:2px 8px 2px 0;vertical-align:top;}} code{{color:{c};}}</style>")
    return css + f"""
<h1 style="color:{g}">SAS &amp; AUTOPILOT DESIGNER - USER GUIDE</h1>
<p>Work from left to right through the tabs: <b>Model &rarr; SAS &rarr; Handling qualities &rarr; Autopilot
&rarr; Gain schedule &rarr; Export</b>. Projects are saved with File &rarr; Save (<code>.sasproj</code>);
on opening, results are recomputed from the saved parameters (no re-tuning). Press <b>F1</b> anywhere for the
section of the current tab.</p>

<h2><a name="model"></a>1. MODEL</h2>
<p>Matrices are typed in MATLAB syntax: <code>[a b; c d]</code>, rows on separate lines allowed,
<code>%</code> comments, scientific notation (<code>3.88e-6</code>, <code>3.88*10^-6</code>) and expressions
(<code>-g*cos(alpha0)</code>, <code>pi/180</code>). Always available constants: <code>g</code>,
<code>V0</code> = <code>TAS</code> = <code>V</code> = <code>U0</code> = <code>Ue</code>, <code>Mach</code>,
<code>theta0</code> [rad]. More can be defined in <i>Constants</i>, e.g. <code>alpha0 = 4.6*pi/180</code>.</p>
<h3>State conventions</h3>
<table>
<tr><td class="k">Longitudinal [u w q &theta;]</td><td>body-axis velocities. Converted to
[u n<sub>z</sub> q &theta;] with n<sub>z</sub> = (w&#775; - V0 q)/g. Needs V0.</td></tr>
<tr><td class="k">Longitudinal [V &alpha; q &theta;]</td><td>wind-axis form (also what the aerodynamic-coefficient window produces).</td></tr>
<tr><td class="k">Lateral [&beta; p r &phi;]</td><td>inputs usually [&delta;a &delta;r].</td></tr>
<tr><td class="k">Units</td><td>rad, rad/s, m/s, m; thrust in N. Angles are displayed in degrees.</td></tr>
<tr><td class="k">Inputs</td><td>named in <i>Input names</i>; <code>dT</code>/throttle is treated as non-angular
(no 30 deg limit by default).</td></tr>
</table>
<h3>Aircraft class and flight-phase category (MIL-F-8785C)</h3>
<p>Class I small light, II-L / II-C medium (land / carrier), III large heavy, IV high-manoeuvrability.
Category A: rapid manoeuvring / precision tracking; B: gradual manoeuvres (climb, cruise); C: terminal
(take-off, approach, landing). They set every Level boundary below.</p>
<h3>Matrices from aerodynamic coefficients</h3>
<p><i>Build matrices from aerodynamic coefficients</i> opens a form for mass, inertias (Ixx, Iyy, Izz, Ixz),
geometry (S, c&#772;, b), thrust line, and the linear coefficient models (per rad):
C<sub>L</sub> = C<sub>L0</sub> + C<sub>L&alpha;</sub>&alpha; + C<sub>Lq</sub>q&#770; + C<sub>L&delta;e</sub>&delta;e,
idem C<sub>D</sub>, C<sub>m</sub>; C<sub>Y</sub>, C<sub>l</sub>, C<sub>n</sub> in &beta;, p&#770;, r&#770;,
&delta;a, &delta;r (rate derivatives w.r.t. the AVL non-dimensional rates or per rad/s). The program:</p>
<table>
<tr><td class="k">1. Atmosphere</td><td>ISA density and speed of sound at h (V or Mach).</td></tr>
<tr><td class="k">2. Trim</td><td>Newton on V&#775; = &alpha;&#775; = q&#775; = 0 for &alpha;, &delta;e, T at the given
V and flight-path angle &gamma;.</td></tr>
<tr><td class="k">3. Linearisation</td><td>central-difference Jacobian of the nonlinear equations around the trim:
A<sub>long</sub>, B<sub>long</sub> in [V &alpha; q &theta;] with inputs [&delta;e, T (N)];
A<sub>lat</sub>, B<sub>lat</sub> in [&beta; p r &phi;] at the trim &alpha;0, &theta;0 (Ixz coupling included).</td></tr>
<tr><td class="k">4. Checks</td><td>static margin, n/&alpha;, dimensional derivatives, open-loop modes with their
Level, warnings (C<sub>m&alpha;</sub> &gt; 0, C<sub>n&beta;</sub> &lt; 0, C<sub>l&beta;</sub> &gt; 0, trim
elevator / thrust / &alpha; out of range, Mach &gt; 0.8).</td></tr>
<tr><td class="k">5. Mach sweep</td><td>coefficients given at a reference Mach are scaled with 1/&radic;(1-(kM)&sup2;)
(k = 1 Prandtl-Glauert, 0.636 = a fit to AVL results); every Mach is trimmed and linearised and added as
a gain-schedule flight point.</td></tr>
</table>
<h3>Actuators</h3>
<p>First-order lag &tau; and a pure delay (1st-order Pad&eacute;) between every command and its surface. Used by
the autopilot design/evaluation, the margins, the 1797A criteria and the digital check. State-feedback
controllers never feed back actuator or Pad&eacute; states (they are not measured).</p>

<h2><a name="sas"></a>2. SAS</h2>
<p>Pole placement, <code>u = -K x</code>. <b>Auto targets</b> put the poles inside the Level 1 region. With
<i>Minimise control effort</i> (default) the whole Level 1 region (with a safety margin from its edges) is searched
for the targets that need the smallest surface command for a set of disturbances: 5&deg; &alpha; and 5&deg;/s q
(longitudinal), 5&deg; &beta;, 10&deg;/s p and 5&deg;/s r (lateral). Unticked, the classic rule is used: short
period &zeta; = 0.7 and &omega;n close to the natural value, dutch roll &zeta; &ge; 0.6. Targets can be edited.
Note: targets at the edge of the region need little effort but are less robust - check them in the Robustness tab.</p>
<table>
<tr><td class="k">Full state</td><td>feedback on all four states - also places the phugoid
(note: &theta; feedback makes the aircraft attitude-command).</td></tr>
<tr><td class="k">Short period only</td><td>feedback on n<sub>z</sub>/&alpha; and q,
classic C* response.</td></tr>
<tr><td class="k">Yaw-rate washout &tau;w</td><td>r is fed back through s/(s+1/&tau;w) so the yaw damper does
not oppose steady turns. Gains are re-optimised so the four airframe poles stay on target.</td></tr>
<tr><td class="k">C* and q/q<sub>ss</sub></td><td>step response of the SAS aircraft (normalised at 3 s) against
envelopes spanned by short-period responses with &zeta; 0.35-1.30, &omega;<sub>n</sub> 2-8 rad/s and
&omega;<sub>n</sub>T<sub>&theta;2</sub> 1.5-3.5 (&plusmn;0.05); C* = n<sub>z</sub> + 12.5 q (n<sub>z</sub> positive up).
Slow aircraft whose phugoid moves within 3 s can fall outside even with a good short period.</td></tr>
<tr><td class="k">SP gain domain</td><td>gains (K<sub>nz/&alpha;</sub>, K<sub>q</sub>) of the decoupled short
period along the Level 1 boundary (the gain allocation domain).</td></tr>
</table>
<h3>MIL-F-8785C Level 1 (summary)</h3>
<table>
<tr><td class="k">Short period</td><td>&zeta; 0.35-1.30 (cat A, C), 0.30-2.0 (B); CAP = &omega;n&sup2;/(n/&alpha;)
0.28-3.6 (A), 0.085-3.6 (B), 0.16-3.6 (C); &omega;n &ge; 1.0 (A).</td></tr>
<tr><td class="k">Phugoid</td><td>&zeta; &ge; 0.04 (Level 2: &ge; 0; Level 3: T2 &ge; 55 s).</td></tr>
<tr><td class="k">Dutch roll</td><td>&zeta;, &zeta;&omega;n, &omega;n minima per class/category (e.g. A, I/IV:
0.19, 0.35, 1.0 rad/s).</td></tr>
<tr><td class="k">Roll mode</td><td>&tau;r &le; 1.0 s (A/C, classes I, IV) or 1.4 s.</td></tr>
<tr><td class="k">Spiral</td><td>time to double &ge; 12 s (A, C) or 20 s (B).</td></tr>
</table>

<h2><a name="hq"></a>3. HANDLING QUALITIES (MIL-STD-1797A)</h2>
<table>
<tr><td class="k">CAP - &zeta;</td><td>control anticipation parameter vs short-period damping, Level regions.</td></tr>
<tr><td class="k">Bandwidth / phase delay</td><td>from &theta;/&delta; of the SAS aircraft with actuators:
&omega;BW = min(&omega; at -135 deg, &omega; where the gain is 6 dB above the gain at -180 deg);
&tau;p = -(&phi;(2&omega;180) + 180)/(57.3&middot;2&omega;180). A gain-limited bandwidth indicates PIO
tendency. Boundaries are editable (commonly quoted values - verify with your edition).</td></tr>
<tr><td class="k">Gibson dropback</td><td>pilot step held then released on the short-period model; DB/q<sub>ss</sub>
&asymp; T&theta;2 - 2&zeta;/&omega;n. Guidance box: 0 &le; DB/q<sub>ss</sub> &le; 0.25 s,
q<sub>pk</sub>/q<sub>ss</sub> &le; 3.</td></tr>
<tr><td class="k">Roll performance</td><td>time to the required bank angle with an aileron step (MIL-F-8785C
Table IX, Level 1), with SAS and actuators.</td></tr>
</table>

<h2><a name="autopilot"></a>4. AUTOPILOT</h2>
<p>The SAS stays engaged; the autopilot adds a command on top (<i>effort = autopilot + SAS</i>).
A reference prefilter pf/(s+pf) smooths the step. Margins are computed with the loop broken at the autopilot
command; for the coupled (MIMO) modes, loop-at-a-time on each input, worst case reported.</p>
<h3>Modes</h3>
<table>
<tr><td class="k">&theta;, h, V/u, &phi;, &psi;, &beta;</td><td>single loops (h and &psi; are added as states:
h&#775; = V0(&theta; - &alpha;) or V0&theta; - w, &psi;&#775; = r / cos&theta;0).</td></tr>
<tr><td class="k">h + V</td><td>altitude hold with autothrottle (&delta;e, &delta;T).</td></tr>
<tr><td class="k">&psi; + &beta;</td><td>heading hold with turn coordination (&delta;a, &delta;r).</td></tr>
</table>
<h3>Methods and their main knobs</h3>
<table>
<tr><td class="k">PIDF</td><td>Kp + Ki/s + Kd s/(Tf s+1) on the tracked error. Faster: larger Kp/Ki. More
margin: smaller Kp/Kd, larger Tf.</td></tr>
<tr><td class="k">LQI</td><td>state feedback + integrator on the error; Q (states + integrators, Bryson-scaled),
R (commands). Larger R = slower, more robust, less effort.</td></tr>
<tr><td class="k">LQG</td><td>LQI gains with a Kalman filter on the tracked measurement (Qn process noise,
Rn = noise variance). Smaller Qn = more filtering but less loop recovery.</td></tr>
<tr><td class="k">H&infin; (hinfsyn)</td><td>weights W1 (output), W2 (commands), W3 (states), W4 (integrators),
nu (measurement-noise level). Larger W2 or nu = lower bandwidth, better delay margin.</td></tr>
<tr><td class="k">H&infin; loop shaping</td><td>Glover-McFarlane with W1 = k(s+&rho;&omega;c)/s; &gamma; &lt; 4
(&epsilon; &gt; 0.25) means good robustness. &omega;c sets the bandwidth.</td></tr>
<tr><td class="k">Cascade (heading)</td><td>PI on &psi; gives the bank command to an LQI roll loop; the bank
limit is respected in tuning.</td></tr>
</table>
<h3>Requirements</h3>
<table>
<tr><td class="k">GM / PM / DM</td><td>gain, phase and delay margin (defaults 6 dB, 35 deg, 0.2 s as in the
project specification). DM = PM/&omega;c: the higher the crossover, the smaller the delay margin.</td></tr>
<tr><td class="k">Effort</td><td>peak total surface command must stay within the deflection limit; rate limits are
used in the saturation simulation.</td></tr>
<tr><td class="k">Anti-windup</td><td>conditional integration: integrators are frozen while a command is limited
and integrating would push it further into the limit. The saturation plot shows both.</td></tr>
<tr><td class="k">Noise</td><td>coloured noise bw/(s+bw) on the tracked measurement.</td></tr>
</table>
<h3>Digital implementation</h3>
<p>The airframe and actuators are discretised exactly (ZOH); the SAS gains and the controller run in the
computer at Ts (Tustin, ZOH or backward Euler), optionally with a one-sample computation delay.
<i>Suggest sample rate</i> uses &omega;s &asymp; 30&times;&omega;c. The table gives stability, discrete margins and
the deviation from the continuous response.</p>

<h2><a name="robust"></a>5. ROBUSTNESS</h2>
<p><b>Monte Carlo:</b> every force/moment derivative of A and B is scaled independently by a uniform factor
in [1-x, 1+x] (kinematic and gravity terms stay exact). The SAS gains and the autopilot controllers are kept at
their nominal values - as in flight, where the real aircraft differs from the model - and each sample is checked:
SAS stability and Level 1, autopilot stability, margins, effort and tracking. The pole cloud shows how far the
closed-loop poles move; the response spread shows the tracked output of the samples.</p>
<p><b>Disk margins</b> (balanced, skew 0): &alpha; = 1 / max<sub>&omega;</sub> &sigma;&#772;(S - I/2),
S = (I + L)<sup>-1</sup>; disk GM = &plusmn;20 log10((2+&alpha;)/(2-&alpha;)) dB and disk PM = 2 atan(&alpha;/2),
valid <i>simultaneously</i>. They are stricter than the classical margins; the multi-loop value allows gain/phase
changes at all autopilot inputs at the same time (important for the MIMO modes).</p>

<h2><a name="schedule"></a>6. GAIN SCHEDULE</h2>
<p>Add flight points (Mach, V0, matrices) - <i>Add from Model tab</i> copies the current matrices. Select the
state convention of the points. At each point the SAS (and optionally the LQI autopilot) is designed with the
automatic Level 1 rules; gains are interpolated linearly in Mach (extrapolated outside). The lower table applies
each point's gains to its neighbours' plants (robustness between nodes).</p>

<h2><a name="export"></a>7. EXPORT</h2>
<p>PDF report (all tables and figures), PNG figures, Excel workbook and CSV tables, including the discrete
controller matrices. Figures are exported on a white background for printing unless <i>HUD style</i> is
selected.</p>

<h2><a name="fc"></a>8. PX4 / ARDUPILOT</h2>
<p>Designs the attitude controller that runs on a fixed-wing autopilot (it replaces the SAS + attitude loops
of the firmware) using only what the autopilot measures: body rates p, q, r, roll and pitch angles and, if
fitted, airspeed. Angle of attack and sideslip are never fed back. Throttle stays with the firmware's energy
controller (TECS); attitude setpoints still come from the firmware (sticks in Stabilized, navigation in Auto).</p>
<table>
<tr><td class="k">Firmware PID structure</td><td>the firmware's own loops: attitude error / time constant
&rarr; rate setpoint; rate PI + feed-forward; yaw damper on r - (g/V)&phi; (ArduPlane: with its 0.2 rad/s
washout). Tuned automatically on the same requirements as the Autopilot tab, plus the return from an attitude
offset (so feed-forward alone cannot win) and a damping of at least 0.3 on every oscillatory mode. The search
stays in physical ranges: rate-loop bandwidth 1-100 rad/s, integrator time 0.2-5 s, feed-forward up to twice its
ideal value, time constant inside the firmware range. Exported as parameters: PX4 FW_P_TC, FW_PR_P/I/FF, FW_R_TC,
FW_RR_P/I/FF, FW_YR_P, FW_AIRSPD_TRIM; ArduPlane PTCH2SRV_TCONST, PTCH_RATE_*, RLL2SRV_TCONST, RLL_RATE_*,
YAW2SRV_DAMP, SCALING_SPEED (ArduPlane gains = normalized gains &times; &pi;/4).</td></tr>
<tr><td class="k">LQG</td><td>LQI gains + a Kalman filter driven by the sensors (estimates &alpha; and &beta;);
integral action on the measured attitude.</td></tr>
<tr><td class="k">H&infin;</td><td>output feedback on the sensors and the attitude-error integrator.</td></tr>
<tr><td class="k">Outputs</td><td>normalized torque in [-1, 1]: deflection / maximum deflection, signed so that
a positive value gives a nose-up / right-wing-down / nose-right moment (PX4 vehicle_torque_setpoint).
The maximum deflections must match the servo travel configured in the autopilot.</td></tr>
<tr><td class="k">Digital check</td><td>every design is discretised (Tustin) at the controller rate and checked
with a one-sample delay; the exported code is that discrete controller.</td></tr>
<tr><td class="k">Export</td><td>PX4: parameter file (.params, QGroundControl) for the PID structure, and for
any structure the module <b>fw_ss_control</b> (C code + C++ module, install script, README): stop
fw_rate_control and start fw_ss_control. ArduPilot: .param file (PID structure on both axes only; ArduPlane
cannot load an external controller).</td></tr>
<tr><td class="k">Before flight</td><td>fly the design in SITL with a model of the same aircraft, check surface
directions and limits, then fly conservatively. This is a design tool, not certified flight software.</td></tr>
</table>

<h2><a name="diag"></a>DIAGNOSTICS</h2>
<p>The SAS, Handling qualities and Autopilot tabs list every requirement that is not met, why, and what to change
(<span style="color:{theme.AMBER}">[CAUTION]</span> / <span style="color:{theme.RED}">[WARNING]</span>,
suggestions in <span style="color:{c}">cyan</span>). Typical cases:</p>
<table>
<tr><td class="k">Delay margin too small</td><td>crossover too high for the actuator lag/delay - lower the
bandwidth (larger R / W2 / nu, smaller Kp, smaller &omega;c).</td></tr>
<tr><td class="k">Effort over the limit</td><td>if the SAS alone already exceeds it, soften the SAS targets;
otherwise slow the response (prefilter, reference size, control weight).</td></tr>
<tr><td class="k">Not settled</td><td>a slow transmission zero (e.g. &theta;/&delta;e with a zero near s &asymp; -0.01)
limits output-only feedback; use LQI/LQG/H&infin;.</td></tr>
</table>
"""
