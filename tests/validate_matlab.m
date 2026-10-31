function validate_matlab(fin, fout)
% Cross-check of the Python app against MATLAB Control/Robust Control toolboxes.
d = load(fin);
r = struct();
% 1. acker (SAS)
r.K_long_ml = acker(d.As, d.Bs(:,1), d.poles_long);
% 2. LQI
n = size(d.A_sas,1);
Aa = [d.A_sas zeros(n,1); -d.c 0]; Ba = [d.b; 0];
[K,~,~] = lqr(Aa, Ba, d.Q, d.R);
r.K_lqi_ml = K;
G = ss(Aa, Ba, eye(n+1), zeros(n+1,1));
m = allmargin(K*G);
r.lqi_margins_ml = mm(m);
% 3. lqe
[L,~,~] = lqe(d.A_sas, eye(n), d.c, d.Qn*eye(n), d.Rn);
r.L_ml = L;
Kx = K(1:n); Ki = K(n+1);
Ak = [d.A_sas - d.b*Kx - L*d.c, -d.b*Ki; -d.c, 0];
Klqg = ss(Ak, [L; 0], [-Kx -Ki], 0);
Gy = ss(d.A_sas, d.b, d.c, 0);
m = allmargin(-Klqg*Gy);
r.lqg_margins_ml = mm(m);
% 4. PIDF in the loop u = C(s)(r - theta)
s = tf('s');
C = d.pid(1) + d.pid(2)/s + d.pid(3)*s/(d.pid(4)*s+1);
m = allmargin(C*Gy);
r.pid_margins_ml = mm(m);
% 5. hinfsyn - same generalized plant as core/autopilot.make_hinf
W = d.hinf_w; W1=W(1); W2=W(2); W3=W(3); W4=W(4); nu=W(5);
na = n+1; nw = 1+na; nz = 2+n+1;
B1 = zeros(na,nw); B1(:,1) = Ba;
C1 = zeros(nz,na); C1(1,1:n) = W1*d.c; C1(3:2+n,1:n) = W3*eye(n); C1(3+n,n+1) = W4;
D12 = zeros(nz,1); D12(2) = W2;
D21 = zeros(na,nw); D21(:,2:end) = nu*eye(na);
P = ss(Aa, [B1 Ba], [C1; eye(na)], [zeros(nz,nw) D12; D21 zeros(na,1)]);
[~,~,gam] = hinfsyn(P, na, 1);
r.gamma_hinf_ml = gam;
% 6. ncfsyn on the same shaped plant (W1 = kw (s+a)/s)
a = d.ls_a;
W1s = d.ls_kw*(s+a)/s;
Gs = Gy*W1s;
[~,~,gls] = ncfsyn(ss(Gs));
r.ls_gamma_ml = gls;
% 7. lateral place
Kp = place(d.Alat, d.Blat, d.poles_lat);
r.K_lat_ml = Kp;
r.poles_lat_ml = eig(d.Alat - d.Blat*Kp);
r.poles_lat_py = eig(d.Alat - d.Blat*d.K_lat_py);
save(fout, '-struct', 'r');
end

function v = mm(m)
gm = 20*log10(m.GainMargin); if isempty(gm), gm = Inf; end
[~,i] = min(abs(gm)); gm = gm(i);
pm = min(m.PhaseMargin); if isempty(pm), pm = Inf; end
dm = min(m.DelayMargin); if isempty(dm), dm = Inf; end
v = [gm pm dm];
end
