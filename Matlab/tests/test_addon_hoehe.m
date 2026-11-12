function test_addon_hoehe(Z1, MONT, SIM, REG)
% TEST_ADDON_HOEHE  Prüft Add-on 2 (Höhe im Nullraum) ohne Simulink.
% 1) Vorrang:        J_w*(dq - dq1) = 0          (Höhe stört die Orientierung nicht)
% 2) Höhenaufgabe:   j_h*dq = v_soll             (bis auf Dämpfung lambda)
% 3) Formeln:        numerische Ableitung von ee_hoehe = v_rumpf + j_h*dq
%                    (prüft Vorzeichen von Hebelarm, Gyro-Frame und Jacobi-Zeile)
% 4) Aufschaltung:   mit v_soll = -v_rumpf steht die Höhe still
rng(7);                                              % reproduzierbar
q    = SIM.q0 + deg2rad(5) * randn(6,1);             % Pose nahe der Grundpose
R_WB = rot_achse([1;0;0], 0.05) * rot_achse([0;1;0], -0.04);
w_B  = deg2rad([10; -15; 5]);                        % [rad/s] im Rumpf-Frame
z_B  = 0.01;   dz_B = 0.2;                           % [m], [m/s]
dq1  = deg2rad([5; -10; 8; 3; -4; 6]);               % beliebige Primärlösung
v_soll = -0.15;                                      % [m/s]

J   = z1_jacobian(q, Z1, MONT.T_6C);
j_h = [0 0 1] * R_WB * MONT.R_B0 * J(1:3, :);
dq  = nullraum_hoehe(dq1, v_soll, q, R_WB, Z1, MONT, REG.lambda);

% 1) Vorrang der Orientierung
f1 = norm(J(4:6, :) * (dq - dq1));
assert(f1 < 1e-12, 'test_addon_hoehe: Höhe verändert die Orientierung (%.2e rad/s)', f1);

% 2) Höhenaufgabe erfüllt
f2 = abs(j_h*dq - v_soll) / abs(v_soll);
assert(f2 < 0.01, 'test_addon_hoehe: Höhenaufgabe nur bis auf %.2f %% erfüllt', 100*f2);

% 3) Formel gegen numerische Ableitung (zentrale Differenz)
[~, v_rumpf] = hoehe_aufschaltung(q, R_WB, w_B, z_B, dz_B, Z1, MONT);
dt = 1e-6;   a = w_B / norm(w_B);   wb = norm(w_B);
h_p = ee_hoehe(q + dq*dt, R_WB * rot_achse(a,  wb*dt), z_B + dz_B*dt, Z1, MONT);
h_m = ee_hoehe(q - dq*dt, R_WB * rot_achse(a, -wb*dt), z_B - dz_B*dt, Z1, MONT);
dh_num = (h_p - h_m) / (2*dt);
dh_ana = v_rumpf + j_h*dq;
f3 = abs(dh_num - dh_ana);
assert(f3 < 1e-7, 'test_addon_hoehe: Formel %.6f, numerisch %.6f m/s', dh_ana, dh_num);

% 4) Aufschaltung allein: Höhe steht
dq4  = nullraum_hoehe(dq1, -v_rumpf, q, R_WB, Z1, MONT, REG.lambda);
rest = abs(v_rumpf + j_h*dq4) / abs(v_rumpf);
assert(rest < 0.01, 'test_addon_hoehe: Aufschaltung lässt %.2f %% übrig', 100*rest);

fprintf(['test_addon_hoehe: OK  (Orientierung %.1e rad/s, Höhenaufgabe %.2f %%, ' ...
    'Formel vs. numerisch %.1e m/s, Rest Aufschaltung %.2f %%)\n'], ...
    f1, 100*f2, f3, 100*rest);
end