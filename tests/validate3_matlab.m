function validate3_matlab(folder)
% Checks the exported .mat / loader script: closed loop rebuilt with Control System Toolbox and
% the generated Simulink model reproduce the application's responses.
addpath(folder); cd(folder);
S = cjtest_load('load');
cases = {'theta', 'PIDF'; 'theta', 'LQI'; 'theta', 'LQG'; 'theta', 'Hinf'; 'heading_beta', 'LQI'};
for i = 1:size(cases, 1)
    mode = cases{i, 1}; meth = cases{i, 2};
    M = S.ap.(mode); C = M.(meth); P = M.plant;
    ks = P.tracked(:)';
    % 1) Control System Toolbox closed loop
    [sys, t, y] = cjtest_load('closedloop', S, mode, meth); %#ok<ASGLU>
    ya = C.response.y_internal; ta = C.response.t(:);
    yi = interp1(t, y(:, ks), ta);
    e1 = max(abs(yi(:) - ya(:))) / max(abs(M.settings.ref));
    % 2) Simulink, without limits (linear) -> must match too
    mdl = cjtest_load('simulink', S, mode, meth, false);
    out = sim(mdl);
    xs = out.x_sim; ysl = interp1(xs.Time, squeeze(xs.Data(:, ks)), ta);
    if size(ysl, 2) ~= numel(ks), ysl = ysl'; end
    e2 = max(abs(ysl(:) - ya(:))) / max(abs(M.settings.ref));
    close_system(mdl, 0);
    % 3) Simulink with rate limits and saturation: just has to run and stay bounded
    mdl = cjtest_load('simulink', S, mode, meth, true);
    out = sim(mdl); xs = out.x_sim;
    fin = squeeze(xs.Data(end, ks)) ./ M.settings.ref(:)';
    close_system(mdl, 0);
    fprintf('CHECK %-13s %-5s  closed-loop err %.2e   simulink err %.2e   limited final/ref %s\n', ...
        mode, meth, e1, e2, mat2str(fin, 4));
end
K = cjtest_load('schedule', S, 0.55, 'long');
Kpy = load('sched_055.txt');
fprintf('CHECK schedule interp max diff %.2e\n', max(abs(K(:) - Kpy(:))));
end
