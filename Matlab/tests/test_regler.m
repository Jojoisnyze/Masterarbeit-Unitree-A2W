function test_regler(Z1, MONT, SIM, REG)
% TEST_REGLER  Prüft die Mathe-Bausteine des Reglers (ohne Zeit, ohne Regelkreis):
%   1. rot_log: Matrix-Logarithmus, auch bei theta ~ 0 und theta ~ pi
%   2. orient_fehler: Vorzeichen und Frame
%   3. dls_pinv: lambda = 0 entspricht pinv, Restfehler bei lambda = REG.lambda
%   4. Herleitung der Aufschaltung: dq = J#(-R_B0'*w_B) -> Kamera dreht nicht
%   5. Grundpose: innerhalb der Grenzen, J_w gut konditioniert
zufall = RandStream('mt19937ar', 'Seed', 11);
zrot = @() rot_achse(randn(zufall,3,1), 2*pi*rand(zufall));       % zufällige Drehung
zq   = @() Z1.q_min + rand(zufall,6,1) .* (Z1.q_max - Z1.q_min);   % zufällige Pose

%% 1. rot_log
f_log = 0;
for th = [0, 1e-9, 1e-3, 0.5, 2.0, pi - 1e-6, pi]
    for k = 1:50
        a = randn(zufall,3,1);  a = a / norm(a);
        R = rot_achse(a, th);
        e = rot_log(R);
        f_log = max(f_log, norm(rot_exp(e) - R));                    % ergibt wieder R?
        if th < pi - 1e-3
            f_log = max(f_log, norm(e - a*th));                      % Achse*Winkel?
        end
    end
end
fprintf('  rot_log: max. Abweichung = %.1e (inkl. theta = 0 und pi)\n', f_log);
assert(f_log < 1e-9, 'rot_log fehlerhaft');

%% 2. orient_fehler: Soll = Ist, um 0,1 rad um z_W weitergedreht
R_WC = zrot();  R_W0 = zrot();
[e_0, e_W] = orient_fehler(R_WC, rot_achse([0;0;1], 0.1) * R_WC, R_W0);
assert(norm(e_W - [0;0;0.1]) < 1e-12, 'orient_fehler: Vorzeichen falsch');
assert(norm(e_0 - R_W0'*e_W) < 1e-12, 'orient_fehler: Frame falsch');
disp('  orient_fehler: Vorzeichen und Frame korrekt');

%% 3. + 4. dls_pinv und Herleitung der Aufschaltung
f_pinv = 0;  rest_dls = 0;  f_ff = 0;
for k = 1:200
    q = zq();
    J = z1_jacobian(q, Z1, MONT.T_6C);  J_w = J(4:6,:);

    % 3. lambda = 0 -> identisch mit pinv;  lambda > 0 -> J*J# weicht von I ab
    f_pinv   = max(f_pinv, norm(dls_pinv(J_w, 0) - pinv(J_w)));
    rest_dls = max(rest_dls, norm(J_w*dls_pinv(J_w, REG.lambda) - eye(3)));

    % 4. w_W,C = w_W,B + R_W0*J_w*dq  muss mit dq = J#(-R_B0'*w_B) null werden
    w_B = randn(zufall,3,1);  R_WB = zrot();  R_B0 = zrot();
    dq  = dls_pinv(J_w, 0) * (-R_B0' * w_B);
    w_C = R_WB*w_B + R_WB*R_B0*J_w*dq;
    f_ff = max(f_ff, norm(w_C));
end
fprintf('  dls_pinv(lambda = 0) vs. pinv:          max. Abw. = %.1e\n', f_pinv);
fprintf('  dls_pinv(lambda = %.2f): Restfehler     max. %.2f %%\n', REG.lambda, 100*rest_dls);
fprintf('  Aufschaltung (Herleitung): max |w_C|  = %.1e rad/s\n', f_ff);
assert(f_pinv < 1e-10,  'dls_pinv weicht von pinv ab');
assert(rest_dls < 0.01, 'Dämpfung zu groß');
assert(f_ff < 1e-10,    'Herleitung der Aufschaltung stimmt nicht');

%% 5. Grundpose
assert(all(SIM.q0 >= Z1.q_min & SIM.q0 <= Z1.q_max), 'Grundpose außerhalb der Grenzen');
[J, T_0C] = z1_jacobian(SIM.q0, Z1, MONT.T_6C);
s = svd(J(4:6,:));
abstand = rad2deg(min(SIM.q0 - Z1.q_min, Z1.q_max - SIM.q0));
fprintf('  Grundpose "%s": p_C = [%.3f %.3f %.3f] m, Blick x_C = [%.2f %.2f %.2f]\n', ...
        SIM.pose, T_0C(1:3,4), T_0C(1:3,1));
fprintf('     sigma_min(J_w) = %.2f, Abstand zu den Grenzen [°]: %s\n', ...
        s(3), mat2str(round(abstand')));

%% 6. kamera_orientierung (wird in Simulink verwendet) gegen zweiten Rechenweg
%  z1_jacobian liefert die Kamerapose als ZWEITEN Ausgang -> unabhängige Prüfung
f_kam = 0;
for k = 1:200
    q = zq();  R_WB = zrot();  MONT_t = MONT;  MONT_t.R_B0 = zrot();
    [~, T_0C] = z1_jacobian(q, Z1, MONT_t.T_6C);
    R_WC = kamera_orientierung(q, R_WB, Z1, MONT_t);
    f_kam = max(f_kam, norm(R_WC - R_WB * MONT_t.R_B0 * T_0C(1:3,1:3)));
end
fprintf('  kamera_orientierung: max. Abweichung = %.1e\n', f_kam);
assert(f_kam < 1e-12, 'kamera_orientierung fehlerhaft');

disp('test_regler: OK');
end

% ---------------------------------------------------------------------------
function R = rot_exp(e)
% Umkehrung von rot_log: e = a*theta -> R = Rot(a, theta)
if norm(e) < 1e-15
    R = eye(3);
else
    R = rot_achse(e / norm(e), norm(e));
end
end