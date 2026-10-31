function validate2_matlab(fin, fout)
% Independent MATLAB construction of: actuators (lag + pade(T,1)) in series with every input,
% SAS feedback, theta/de frequency response, Tustin controller, discrete loop margins.
d = load(fin);
r = struct();
m = size(d.Bs,2);
s = tf('s');
act = ss(tf(1,[d.tau 1])*pade(exp(-d.delay*s),1));
Gair = ss(d.As, d.Bs, eye(4), zeros(4,m));
Gext = Gair*append(act, act);                 % commands -> actuators -> airframe
CL = feedback(Gext, d.Ksas);                   % u = v - Ksas*x
r.poles_act_ml = esort(eig(CL));
G = freqresp(CL(4,1), d.w);
r.G_ml = squeeze(G);
% Tustin of the LQI controller
K = ss(d.Ac, d.Bc, d.Cc, d.Dc);
Kd = c2d(K, d.Ts, 'tustin');
r.Acd_ml = Kd.A; r.Bcd_ml = Kd.B; r.Ccd_ml = Kd.C; r.Dcd_ml = Kd.D;
% discrete loop: ZOH airframe, one-sample delay on all commands, SAS + LQI in the computer
n = size(d.Aol,1);
Gd = c2d(ss(d.Aol, d.B, eye(n), zeros(n,m)), d.Ts, 'zoh');
Dl = ss(zeros(m), eye(m), eye(m), zeros(m), d.Ts);   % one-sample delay on every command
Gdd = Gd*Dl;                                   % delayed commands
Gs = feedback(Gdd, d.Kext);                    % SAS closed inside the computer
E = zeros(m,1); E(1) = 1;
p = 1;
Kx = Kd(:, p+1:end);
L = -Kx*Gs*E;
mg = allmargin(L);
gm = 20*log10(mg.GainMargin); if isempty(gm), gm = Inf; end
[~,i] = min(abs(gm)); gm = gm(i);
r.dig_margins_ml = [gm min(mg.PhaseMargin) min(mg.DelayMargin)*d.Ts];
save(fout, '-struct', 'r');
end

function e = esort(e)
[~,i] = sort(real(e)+1e-6*imag(e)); e = e(i);
end
