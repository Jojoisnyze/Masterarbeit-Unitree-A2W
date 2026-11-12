function test_ee_hoehe(Z1, MONT, SIM)
% TEST_EE_HOEHE  Prüft die Höhe des Endeffektors (Add-on 2, Schritt 1).
% Arm steif in der Grundpose, r = Lage des Endeffektors im Rumpf-Frame B:
%   1) reiner Hub z_B:   Delta h = z_B
%   2) Nicken um y:      Delta h = -r_x*sin(th) + r_z*(cos(th) - 1)
%      (th > 0 = Nase runter -> ein vorn liegender Endeffektor sinkt)
%   3) Rollen um x:      Delta h =  r_y*sin(ph) + r_z*(cos(ph) - 1)
% Quelle: Drehmatrizen Rot(y), Rot(x), Modern Robotics Kap. 3.2.1
tol  = 1e-12;
h0   = ee_hoehe(SIM.q0, eye(3), 0, Z1, MONT);
T_0C = z1_fk(SIM.q0, Z1, MONT.T_6C);
r    = MONT.p_B0 + MONT.R_B0 * T_0C(1:3, 4);

% 1) Hub
dh = ee_hoehe(SIM.q0, eye(3), 0.02, Z1, MONT) - h0;
assert(abs(dh - 0.02) < tol, 'test_ee_hoehe: Hub 2 cm ergibt %.6f m', dh);

% 2) Nicken 3°
th   = deg2rad(3);
dh_n = ee_hoehe(SIM.q0, rot_achse([0;1;0], th), 0, Z1, MONT) - h0;
soll = -r(1)*sin(th) + r(3)*(cos(th) - 1);
assert(abs(dh_n - soll) < tol, 'test_ee_hoehe: Nicken %.6f statt %.6f m', dh_n, soll);

% 3) Rollen 3°
ph   = deg2rad(3);
dh_r = ee_hoehe(SIM.q0, rot_achse([1;0;0], ph), 0, Z1, MONT) - h0;
soll = r(2)*sin(ph) + r(3)*(cos(ph) - 1);
assert(abs(dh_r - soll) < tol, 'test_ee_hoehe: Rollen %.6f statt %.6f m', dh_r, soll);

fprintf(['test_ee_hoehe bestanden: Endeffektor %.0f mm vor, %.0f mm über der Rumpfmitte.\n' ...
    '   3° Nicken -> %.1f mm, 3° Rollen -> %.1f mm Höhenänderung (Arm steif)\n'], ...
    1000*r(1), 1000*r(3), 1000*dh_n, 1000*dh_r);
end