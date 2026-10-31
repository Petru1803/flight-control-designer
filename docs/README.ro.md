# SAS & Autopilot Designer — documentație detaliată (RO)

> Pagina principală a proiectului este [README.md](../README.md). Căile de mai jos sunt relative la
> folderul aplicației (cel care conține `main.py`).

Aplicație desktop (Python + PySide6) construită pe baza scripturilor MATLAB din folderul părinte
(`proiectv2.m`, `pt_sedan.m`, `SAS_MPD27b.m`, `gain_schedule.m`, `lqi_zgomot.m`, ...).

## Pornire

- **Installer (recomandat):** `installer\Output\SASDesigner-<versiune>-setup.exe` (~78 MB). Instalare per
  utilizator, fără drepturi de administrator (opțional „pentru toți utilizatorii”); creează scurtături în
  Start Menu / pe Desktop, asociază fișierele `.sasproj` (dublu-click deschide proiectul) și apare la
  dezinstalare în „Aplicații instalate”. Reconstruire: `installer\build_installer.bat` (PyInstaller + Inno
  Setup 6; versiunea se ia din `version.py`).
- **Executabil:** `dist\SASDesigner\SASDesigner.exe` (nu cere Python; tot folderul `SASDesigner` trebuie păstrat împreună).
  Un proiect se poate deschide și direct: `SASDesigner.exe proiect.sasproj`.
- **Din sursă:** `run.bat`, sau `..\.venv\Scripts\python.exe main.py`.

## Flux de lucru

1. **Model**: clasa avionului (I, II-L, II-C, III, IV), categoria de zbor (A/B/C), V0/TAS, Mach, θ0.
   Matricile se lipesc în sintaxă MATLAB, cu constante (`g`, `TAS`/`V0`, `theta0`, `pi`, plus cele din câmpul
   *Constants*, ex. `alpha0 = 4.6*pi/180`). Convenție longitudinală `[u w q θ]` (convertită în `[u nz q θ]`)
   sau `[V α q θ]`; lateral `[β p r φ]`.
   **Actuatoare**: constantă de timp τ și întârziere pură (Padé de ordinul 1), aplicate tuturor comenzilor.
2. **SAS**: ținte automate în Level 1 (MIL-F-8785C), editabile; plasare de poli. Pe lateral se poate adăuga
   **washout** pe reacția după r (yaw damper care nu se opune virajelor stabilizate).
3. **Handling qualities (MIL-STD-1797A)**: CAP–ζ, bandwidth / phase delay al atitudinii (cu actuatoare),
   dropback Gibson, performanța de ruliu (MIL-F-8785C). Limitele bandwidth sunt editabile.
4. **Autopilot**:
   - moduri simple: θ, altitudine, viteză, ruliu, heading, coordonare β = 0;
   - moduri cuplate (MIMO): altitudine + autothrottle (h, V) și heading + coordonare (ψ, β);
   - metode: PIDF, LQI, LQG, H∞ (`hinfsyn`), H∞ loop shaping, cascadă PI(ψ) → LQI(φ) pentru heading;
   - tuning automat (GM/PM/DM, bracaj, limită de înclinare), parametri editabili, buton **Cancel**;
   - zgomot pe măsurare, saturare + rate limit cu și fără **anti-windup** (integrare condiționată);
   - **implementare digitală**: frecvență de eșantionare (cu sugestie automată), Tustin / ZOH / Euler,
     întârziere de calcul de un eșantion, stabilitate și margini în discret, abatere față de continuu.
5. **Robustețe**: Monte Carlo pe derivatele aerodinamice (±x% pe fiecare derivată din A și B; termenii
   cinematici și gravitaționali rămân exacți; SAS-ul și autopiloții rămân la valorile nominale) — procent stabil,
   procent Level 1 / procent care respectă cerințele, norul de poli, dispersia răspunsurilor; plus **disk margins**
   (loop-at-a-time și multi-loop cu limită superioară μ, identice cu `diskmargin` din MATLAB).
6. **Gain schedule** pe Mach, cu buton Cancel.
7. **Export**: PDF, PNG, Excel, CSV (inclusiv matricile controlerelor discrete) și **MATLAB / Simulink**:
   `<nume>.mat` + `<nume>_load.m`:
   - `S = <nume>_load();` încarcă tot (modele, câștiguri SAS, controlere A/B/C/D continue și discrete, gain schedule);
   - `<nume>_load('closedloop', S, 'theta', 'LQI')` reconstruiește bucla închisă și răspunsul la treaptă;
   - `<nume>_load('simulink', S, 'theta', 'LQI')` generează un model Simulink (avion + actuatoare, SAS, controler,
     prefiltru, rate limit, saturație);
   - `<nume>_load('schedule', S, 0.75, 'long')` interpolează câștigurile programate.

**SAS cu efort minim** (implicit): țintele automate se caută în toată regiunea Level 1 (cu o margine față de
limite) pentru cel mai mic bracaj la perturbațiile 5° α / 5°/s q (longitudinal) și 5° β / 10°/s p / 5°/s r
(lateral). Fără bifă se folosesc țintele clasice.

**Import .mat** în tab-ul Model: alegi ce variabile (4×n) devin A_long, B_long, A_lat, B_lat.

**Matrici din coeficienți aerodinamici** (tab-ul Model → *Build matrices from aerodynamic coefficients*):
masă, inerții, geometrie, tracțiune și coeficienții liniari (per rad, derivatele de rotație față de ratele
adimensionale AVL sau per rad/s) → atmosferă ISA → trim (α, δe, T) prin Newton → jacobian numeric →
A/B longitudinal [V α q θ] cu intrări [δe, T] și lateral [β p r φ] cu [δa, δr], plus marja statică, n/α,
derivatele dimensionale, modurile cu nivelul Level și avertizări. *Use in Model tab* trimite matricile în
model; *Create gain-schedule points* face un sweep de Mach (corecție 1/√(1−(kM)²), ca în `aero_mach.m`) și
adaugă punctele de zbor în tab-ul Gain schedule. Aceeași formulare ca `dinamica_longitudinala.m` /
`dinamica_laterala.m`; `tests\validate5_lin.py` verifică identitatea cu scripturile MATLAB (diferență ≤ 1e-13).

**Interfață**: stil de cabină militară (MFD/HUD) — fundal negru, simbologie verde fosfor în font monospace
(Consolas), chihlimbar pentru atenționări și roșu pentru avertizări; graficele sunt recolorate în stil HUD pe
ecran, iar la export rămân pe alb (opțional HUD).

**Ajutor și diagnostic**: Help → User guide (**F1**) deschide ghidul la secțiunea tab-ului curent (convenții,
criterii, metode, ce parametru se ajustează). Tab-urile SAS, Handling qualities și Autopilot au un panou
*Diagnostics* care spune ce cerință nu e îndeplinită, de ce, și ce să schimbi.

**Proiecte**: File → Save / Open (`.sasproj`, JSON). Se salvează tot ce e introdus și parametrii tunați;
la deschidere rezultatele se recalculează fără să se repete tuning-ul.

## Convenții

- SAS: `u = -K x`. Efortul raportat = comanda totală (autopilot + SAS).
- Marginile sunt calculate cu bucla tăiată la comanda autopilotului; la modurile MIMO, „loop-at-a-time”
  pe fiecare intrare (celelalte închise), raportând cazul cel mai defavorabil.
- Cu actuatoare: controlerele cu reacție după stare (LQI, LQG, H∞, cascadă) sunt proiectate pe avion cu
  actuatoare ideale și **verificate** pe modelul cu actuatoare (stările actuatorului și ale aproximării Padé
  nu sunt măsurate). Marginile și răspunsurile includ deci întârzierea și lag-ul.
- PID-ul este în bucla directă `u = C(s)(r_f - y)`, cu prefiltrul pe referință
  (în `pt_sedan.m` prefiltrul era în buclă, deci marginile PID diferă de cele din script).
- C* folosește `nz` pozitiv în sus (`C* = nz + 12.5 q`).

## Validare (MATLAB R2025a)

- `tests\validate_make.py` + `tests\validate_matlab.m`: acker, lqr, lqe, allmargin identice (5 cifre);
  γ din hinfsyn la ~0,6%; γ_min din ncfsyn identic; `place` multi-intrare: aceiași poli.
- `tests\validate2_make.py` + `tests\validate2_matlab.m`: SAS + actuatoare (lag + `pade`) — poli identici,
  răspuns în frecvență θ/δe identic, discretizare Tustin identică cu `c2d`, margini discrete cu întârziere
  de un eșantion identice cu `allmargin` (4 cifre).

- `tests\validate3_make.py` + `tests\validate3_matlab.m`: exportul MATLAB — bucla închisă reconstruită de
  script și modelul Simulink generat reproduc răspunsul aplicației (eroare relativă 1e-5 … 1e-13).
- `tests\validate4_disk.m`: disk margins identice cu `diskmargin` (SISO și MIMO).

## Teste

- `tests\smoke.py [moduri...]`: toate modurile × metodele, cu și fără actuatoare, + 1797A și digital
  (date de test: Course Jet, avion generic cu date inventate, în `tests\test_data.py`).
- `tests\ui_test.py <folder>`: test end-to-end al interfeței (headless), export, salvare/încărcare proiect.

## Reconstruire .exe

```
..\.venv\Scripts\pyinstaller.exe --noconfirm SASDesigner.spec
```
