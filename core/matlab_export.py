"""Export for MATLAB / Simulink: a .mat file with every model, gain and controller, plus a
.m script that loads it, rebuilds the closed loops and can generate a Simulink model of any
autopilot (airframe + actuators, SAS, controller, prefilter, rate limit and saturation)."""
import datetime
import os
import re

import numpy as np
import scipy.io

from . import autopilot as ap


def _cell(strings):
    a = np.empty((len(strings),), dtype=object)
    for i, s in enumerate(strings):
        a[i] = str(s)
    return a


def _f(v):
    return float(v) if v is not None and np.isfinite(v) else (np.inf if v is not None and v > 0 else -np.inf)


def _safe(name):
    s = re.sub(r"[^A-Za-z0-9_]", "_", name)
    if not s or not s[0].isalpha():
        s = "m_" + s
    return s[:60]


def build_struct(project):
    p = project
    S = {"info": {"aircraft": p.name, "class": p.cls, "category": p.cat,
                  "created": datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
                  "units": "SI; angles in rad, rates in rad/s; u_sas = -K*x",
                  "actuator_tau": p.actuator.tau, "actuator_delay": p.actuator.delay}}
    for k, v in p.flight.items():
        S["info"][_safe(k)] = v if not isinstance(v, str) else str(v)
    if p.long_model is not None:
        m = p.long_model
        d = {"A_phys": m.A, "B_phys": m.B, "T": m.T, "A": m.Ad, "B": m.Bd, "convention": m.convention,
             "state_names": _cell(m.state_names), "input_names": _cell(m.input_names), "V0": m.V0}
        if p.long_sas is not None:
            r = p.long_sas
            d.update({"K": r.K, "Acl": r.Acl, "poles": r.poles,
                      "targets": {"wn_sp": r.targets.wn_sp, "zeta_sp": r.targets.zeta_sp, "wn_ph": r.targets.wn_ph,
                                  "zeta_ph": r.targets.zeta_ph, "structure": r.targets.structure}})
        S["long"] = d
    if p.lat_model is not None:
        m = p.lat_model
        d = {"A": m.Ad, "B": m.Bd, "state_names": _cell(m.state_names), "input_names": _cell(m.input_names),
             "theta0": m.theta0, "V0": m.V0}
        if p.lat_sas is not None:
            from .sas import lat_extended
            r = p.lat_sas
            Ae, Be, Ke, names, _ = lat_extended(m, r)
            d.update({"K": r.K, "washout_tau": r.targets.washout_tau, "A_ext": Ae, "B_ext": Be, "K_ext": Ke,
                      "ext_state_names": _cell(names), "poles": r.poles,
                      "targets": {"wn_dr": r.targets.wn_dr, "zeta_dr": r.targets.zeta_dr,
                                  "roll_pole": r.targets.roll_pole, "spiral_pole": r.targets.spiral_pole}})
        S["lat"] = d
    if p.ap:
        A = {}
        for mode, e in p.ap.items():
            pl, st = e["plant"], e["settings"]
            md = {"label": ap.MODES[mode]["label"],
                  "plant": {"Aol": pl.Aol, "B": pl.B, "Ksas": pl.Ksas, "A": pl.A,
                            "state_names": _cell(pl.state_names), "input_names": _cell(pl.input_names),
                            "ap_inputs": np.array(pl.js, float) + 1, "tracked": np.array(pl.ks, float) + 1,
                            "tracked_names": _cell(pl.track_names), "display_scale": pl.state_scale},
                  "settings": {"ref": np.asarray(st.ref, float) / pl.state_scale[pl.ks],
                               "ref_display": np.asarray(st.ref, float), "T": st.T, "prefilter": st.prefilter,
                               "limits": np.asarray(st.limits, float), "rates": np.asarray(st.rates, float),
                               "x0_tracked": np.asarray(st.x0_track, float) / pl.state_scale[pl.ks]}}
            for meth, c in e["ctrls"].items():
                r = e["results"].get(meth)
                cd = {"Ac": c.Ac, "Bc": c.Bc, "Cc": c.Cc, "Dc": c.Dc,
                      "Nbar": np.atleast_2d(np.asarray(c.Nbar, float)) if np.ndim(c.Nbar) else
                      float(c.Nbar) * np.eye(pl.p),
                      "params": {_safe(k): (np.asarray(v, float) if isinstance(v, (list, tuple, np.ndarray))
                                            else (float(v) if isinstance(v, (int, float, np.floating)) else str(v)))
                                 for k, v in c.params.items()},
                      "integrator_states": np.array(c.int_states, float) + 1}
                if r is not None:
                    cd["margins"] = {"GM_dB": _f(r.margins["gm_db"]), "PM_deg": _f(r.margins["pm"]),
                                     "DM_s": _f(r.margins["dm"])}
                    cd["response"] = {"t": r.t, "y": r.y / 1.0, "y_internal": r.y / pl.state_scale[pl.ks]}
                dg = (e.get("digital") or {}).get(meth)
                if dg is not None:
                    cd["digital"] = {"Ts": dg["Ts"], "method": dg["method"], "one_sample_delay": float(dg["delay"]),
                                     "Ad": dg["Ad"], "Bd": dg["Bd"], "Cd": dg["Cd"], "Dd": dg["Dd"]}
                md[_safe(meth)] = cd
            A[_safe(mode)] = md
        S["ap"] = A
    if p.schedule is not None:
        s = p.schedule
        d = {"Mach": s.mach, "K_long": s.Klong, "long_input_names": _cell(s.long_inputs),
             "long_state_names": _cell(s.long_names or [])}
        if s.Klat is not None:
            d.update({"K_lat": s.Klat, "lat_input_names": _cell(s.lat_inputs), "lat_state_names": _cell(s.lat_names)})
        if s.Kap is not None:
            d.update({"K_ap": s.Kap, "ap_gain_names": _cell(s.ap_names or [])})
        S["schedule"] = d
    return S


def export(project, folder, base):
    os.makedirs(folder, exist_ok=True)
    base = _safe(base)
    mat = os.path.join(folder, base + ".mat")
    scipy.io.savemat(mat, build_struct(project), long_field_names=True, do_compression=True,
                     oned_as="column")
    script = os.path.join(folder, base + "_load.m")
    with open(script, "w", encoding="utf-8") as fh:
        fh.write(SCRIPT.replace("@BASE@", base).replace("@DATE@", datetime.datetime.now().strftime("%Y-%m-%d %H:%M")))
    return [mat, script]


SCRIPT = r"""%% @BASE@_load.m  -  generated by SAS & Autopilot Designer (@DATE@)
% Loads @BASE@.mat and rebuilds the designs in MATLAB.
%
% Contents of S (all SI, angles in rad):
%   S.info                      aircraft, class / category, actuator model
%   S.long, S.lat               models in design coordinates (A, B), SAS gains K (u_sas = -K*x),
%                               targets and closed-loop poles; S.lat.A_ext/B_ext/K_ext include the
%                               yaw-rate washout state when it is used
%   S.ap.<mode>.plant           Aol, B (from surface commands, actuators included), Ksas,
%                               ap_inputs / tracked (1-based indices), state_names
%   S.ap.<mode>.settings        ref (internal units), prefilter pole, limits, rates
%   S.ap.<mode>.<method>        Ac, Bc, Cc, Dc  - controller with input [r_f; x] and output u_ap
%                               Nbar, params, margins, response (app result), digital (Ad..Dd, Ts)
%   S.schedule                  Mach, K_long (N x m x 4), K_lat, K_ap
%
% Loop (identical to the application):
%   r_f   = Nbar * pf/(s+pf) * r
%   u_ap  = controller([r_f; x])
%   delta = E*u_ap - Ksas*x        (E puts u_ap on the autopilot inputs)
%   x'    = Aol*x + B*sat(ratelim(delta))
%
% Usage:
%   S = @BASE@_load();                              % load and print a summary
%   [sys, t, y] = @BASE@_load('closedloop', S, 'theta', 'LQI');   % linear closed loop + step
%   mdl = @BASE@_load('simulink', S, 'theta', 'LQI');             % build a Simulink model

function varargout = @BASE@_load(cmd, varargin)
if nargin == 0, cmd = 'load'; end
switch lower(cmd)
    case 'load'
        here = fileparts(mfilename('fullpath'));
        S = load(fullfile(here, '@BASE@.mat'));
        summary(S);
        varargout{1} = S;
    case 'closedloop'
        [varargout{1:nargout}] = closed_loop(varargin{:});
    case 'simulink'
        varargout{1} = build_simulink(varargin{:});
    case 'schedule'
        varargout{1} = schedule_gains(varargin{:});
    otherwise
        error('Unknown command %s', cmd);
end
end

% -------------------------------------------------------------------------------------------
function summary(S)
fprintf('\n%s  (class %s, category %s)\n', S.info.aircraft, S.info.class, S.info.category);
if isfield(S, 'long') && isfield(S.long, 'K')
    fprintf('Longitudinal SAS  K =\n'); disp(S.long.K);
end
if isfield(S, 'lat') && isfield(S.lat, 'K')
    fprintf('Lateral SAS  K =\n'); disp(S.lat.K);
end
if isfield(S, 'ap')
    modes = fieldnames(S.ap);
    for i = 1:numel(modes)
        M = S.ap.(modes{i});
        meths = setdiff(fieldnames(M), {'label', 'plant', 'settings'});
        fprintf('Autopilot %-14s: %s\n', modes{i}, strjoin(meths', ', '));
    end
end
end

% -------------------------------------------------------------------------------------------
function [sys, t, y] = closed_loop(S, mode, method)
% Linear closed loop r -> [x; u_ap; delta], same structure as the application.
M = S.ap.(mode); P = M.plant; C = M.(method); st = M.settings;
n = size(P.Aol, 1); m = size(P.B, 2);
js = P.ap_inputs(:)'; ks = P.tracked(:)'; p = numel(ks); q = numel(js);
E = zeros(m, q); for i = 1:q, E(js(i), i) = 1; end
G = ss(P.Aol - P.B*P.Ksas, P.B*E, eye(n), zeros(n, q));          % airframe + actuators + SAS
K = ss(C.Ac, C.Bc, C.Cc, C.Dc);                                   % [r_f; x] -> u_ap
if st.prefilter > 0
    pf = ss(-st.prefilter*eye(p), st.prefilter*eye(p), C.Nbar, zeros(p));
else
    pf = ss(C.Nbar);
end
% close the loop: u_ap = Kr*r_f + Kx*x, x = G*u_ap
Kr = K(:, 1:p); Kx = K(:, p+1:end);
Lcl = feedback(G, Kx, +1);               % x = G*(v + Kx*x)  (v = Kr*r_f)
sys = Lcl * Kr * pf;                     % r -> x
sys = [sys; Kr*pf + Kx*sys];             % append u_ap
t = linspace(0, st.T, 2001)';
y = step(sys * diag(st.ref), t);         % simultaneous steps of amplitude ref on every reference
y = sum(y, 3);                           % (initial offsets of the tracked states are not included)
if nargout == 0
    figure; plot(t, y(:, ks) .* P.display_scale(ks)'); grid on
    xlabel('t [s]'); legend(cellstr(P.tracked_names)); title([M.label ' - ' method]);
end
end

% -------------------------------------------------------------------------------------------
function mdl = build_simulink(S, mode, method, use_limits)
% Builds <mode>_<method>.slx: Step -> prefilter -> controller -> (+ SAS) -> rate limit ->
% saturation -> airframe+actuators; logs x (states) and delta (applied surface commands).
if nargin < 4, use_limits = true; end
M = S.ap.(mode); P = M.plant; C = M.(method); st = M.settings;
n = size(P.Aol, 1); m = size(P.B, 2);
js = P.ap_inputs(:)'; ks = P.tracked(:)'; p = numel(ks); q = numel(js);
E = zeros(m, q); for i = 1:q, E(js(i), i) = 1; end
mdl = sprintf('%s_%s', mode, method);
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl); open_system(mdl);
lim = st.limits(:); rate = st.rates(:);
if ~use_limits, lim(:) = inf; rate(:) = inf; end
x0 = zeros(n, 1); x0(ks) = st.x0_tracked;
blk = @(name) [mdl '/' name];
add_block('simulink/Sources/Step', blk('ref'), 'Time', '0', 'Before', mat2str(zeros(p,1)), ...
    'After', mat2str(st.ref(:)), 'Position', [30 40 60 70]);
if st.prefilter > 0
    add_block('simulink/Continuous/State-Space', blk('prefilter'), 'A', mat2str(-st.prefilter*eye(p)), ...
        'B', mat2str(st.prefilter*eye(p)), 'C', mat2str(C.Nbar), 'D', mat2str(zeros(p)), 'Position', [100 30 180 80]);
else
    add_block('simulink/Math Operations/Gain', blk('prefilter'), 'Gain', mat2str(C.Nbar), ...
        'Multiplication', 'Matrix(K*u)', 'Position', [100 30 180 80]);
end
add_block('simulink/Signal Routing/Mux', blk('mux'), 'Inputs', mat2str([p n]), 'Position', [220 30 225 110]);
add_block('simulink/Continuous/State-Space', blk('controller'), 'A', mat2str(C.Ac), 'B', mat2str(C.Bc), ...
    'C', mat2str(C.Cc), 'D', mat2str(C.Dc), 'Position', [260 40 360 100]);
add_block('simulink/Math Operations/Gain', blk('E'), 'Gain', mat2str(E), 'Multiplication', 'Matrix(K*u)', ...
    'Position', [390 50 430 90]);
add_block('simulink/Math Operations/Sum', blk('sum'), 'Inputs', '+-', 'Position', [460 50 480 90]);
add_block('simulink/Discontinuities/Rate Limiter', blk('rate'), 'RisingSlewLimit', mat2str(rate), ...
    'FallingSlewLimit', mat2str(-rate), 'InitialCondition', '0', 'Position', [510 50 560 90]);
add_block('simulink/Discontinuities/Saturation', blk('sat'), 'UpperLimit', mat2str(lim), ...
    'LowerLimit', mat2str(-lim), 'Position', [590 50 640 90]);
add_block('simulink/Continuous/State-Space', blk('airframe'), 'A', mat2str(P.Aol), 'B', mat2str(P.B), ...
    'C', mat2str(eye(n)), 'D', mat2str(zeros(n, m)), 'X0', mat2str(x0), 'Position', [670 40 770 100]);
add_block('simulink/Math Operations/Gain', blk('Ksas'), 'Gain', mat2str(P.Ksas), 'Multiplication', 'Matrix(K*u)', ...
    'Orientation', 'left', 'Position', [520 150 580 190]);
add_block('simulink/Sinks/To Workspace', blk('x_log'), 'VariableName', 'x_sim', 'SaveFormat', 'Timeseries', ...
    'Position', [820 50 880 80]);
add_block('simulink/Sinks/To Workspace', blk('delta_log'), 'VariableName', 'delta_sim', 'SaveFormat', 'Timeseries', ...
    'Position', [680 130 740 160]);
add_line(mdl, 'ref/1', 'prefilter/1');
add_line(mdl, 'prefilter/1', 'mux/1');
add_line(mdl, 'mux/1', 'controller/1');
add_line(mdl, 'controller/1', 'E/1');
add_line(mdl, 'E/1', 'sum/1');
add_line(mdl, 'sum/1', 'rate/1');
add_line(mdl, 'rate/1', 'sat/1');
add_line(mdl, 'sat/1', 'airframe/1');
add_line(mdl, 'sat/1', 'delta_log/1', 'autorouting', 'on');
add_line(mdl, 'airframe/1', 'x_log/1');
add_line(mdl, 'airframe/1', 'Ksas/1', 'autorouting', 'on');
add_line(mdl, 'Ksas/1', 'sum/2', 'autorouting', 'on');
add_line(mdl, 'airframe/1', 'mux/2', 'autorouting', 'on');
set_param(mdl, 'StopTime', num2str(st.T), 'Solver', 'ode45', 'MaxStep', '0.005', 'RelTol', '1e-6');
save_system(mdl);
fprintf('Simulink model %s.slx created. Run: out = sim(''%s''); x = out.x_sim;\n', mdl, mdl);
end

% -------------------------------------------------------------------------------------------
function K = schedule_gains(S, mach, which)
% Linear interpolation of the scheduled gains (extrapolation outside the Mach range).
% which: 'long' | 'lat' | 'ap'
if nargin < 3, which = 'long'; end
switch which
    case 'long', T = S.schedule.K_long;
    case 'lat',  T = S.schedule.K_lat;
    case 'ap',   T = S.schedule.K_ap;
end
sz = size(T);
K = reshape(interp1(S.schedule.Mach(:), reshape(T, sz(1), []), mach, 'linear', 'extrap'), [sz(2:end) 1]);
K = squeeze(K);
end
"""
