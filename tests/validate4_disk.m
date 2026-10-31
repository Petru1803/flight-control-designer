function validate4_disk(folder)
% Disk margins of the exported autopilots with MATLAB diskmargin (loop broken at the
% autopilot inputs, negative feedback), compared with the values written by Python.
cd(folder); S = cjtest_load('load');
ref = readmatrix('disk_py.txt');
cases = {'theta', 'PIDF'; 'theta', 'LQI'; 'theta', 'LQG'; 'theta', 'Hinf'; 'heading_beta', 'LQI'};
for i = 1:size(cases, 1)
    M = S.ap.(cases{i, 1}); C = M.(cases{i, 2}); P = M.plant;
    n = size(P.Aol, 1); m = size(P.B, 2); js = P.ap_inputs(:)'; ks = P.tracked(:)'; p = numel(ks); q = numel(js);
    E = zeros(m, q); for k = 1:q, E(js(k), k) = 1; end
    G = ss(P.Aol - P.B*P.Ksas, P.B*E, eye(n), zeros(n, q));
    K = ss(C.Ac, C.Bc, C.Cc, C.Dc); Kx = K(:, p+1:end);
    L = -Kx*G;
    [DM, MM] = diskmargin(L);
    gm = 20*log10(MM.GainMargin(2)); pm = MM.PhaseMargin(2);
    fprintf('CHECK %-13s %-5s multi-loop disk GM %.3f dB (py %.3f)  PM %.3f deg (py %.3f)\n', ...
        cases{i,1}, cases{i,2}, gm, ref(i,1), pm, ref(i,2));
end
end
