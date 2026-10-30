function test_z1_jacobian(Z1, MONT, urdfDatei)
% TEST_Z1_JACOBIAN  Prüft die eigene Jacobi-Matrix auf vier Wegen:
%   1. Anschauung bei q = 0 (Matrix ausgeben)
%   2. Numerische Ableitung der eigenen Vorwärtskinematik (toolbox-frei)
%   3. Vergleich mit geometricJacobian der Toolbox (Achtung: liefert [w; v])
%   4. Singularität von J_w: innerhalb der Grenzen nie, bei q234=0 & q5=90° ja

%% 1. Anschauung bei q = 0
J0 = z1_jacobian(zeros(6,1), Z1, MONT.T_6C);
disp('  J(q=0), Zeilen [vx vy vz wx wy wz], Spalten Gelenk 1..6:');
disp(round(J0, 4));
% Erwartung: w-Zeilen = Achsen (z, y, y, y, z, x); Spalten 2-4 gleiche w

%% Toolbox-Modell mit Kamera (nur als Referenz)
robot  = importrobot(urdfDatei, 'DataFormat', 'column');
kamera = rigidBody('kamera');
setFixedTransform(kamera.Joint, MONT.T_6C);
addBody(robot, kamera, 'link06');

rng(2);
h = 1e-6;                         % Schrittweite für die numerische Ableitung
fehler_num = 0;  fehler_tb = 0;
for k = 1:200
    q = Z1.q_min + rand(6,1) .* (Z1.q_max - Z1.q_min);
    [J, T_0C] = z1_jacobian(q, Z1, MONT.T_6C);

    %% 2. Numerische Ableitung (zentraler Differenzenquotient)
    % v-Teil:  dp_C/dq_i
    % w-Teil:  dR/dq_i * R^T = [w_i]  (schiefsymmetrisch, Modern Robotics Kap. 3.2.2)
    J_num = zeros(6, 6);
    for i = 1:6
        dq = zeros(6,1); dq(i) = h;
        Tp = z1_fk(q + dq, Z1, MONT.T_6C);
        Tm = z1_fk(q - dq, Z1, MONT.T_6C);
        J_num(1:3, i) = (Tp(1:3,4) - Tm(1:3,4)) / (2*h);
        W = (Tp(1:3,1:3) - Tm(1:3,1:3)) / (2*h) * T_0C(1:3,1:3)';
        J_num(4:6, i) = [W(3,2); W(1,3); W(2,1)];
    end
    fehler_num = max(fehler_num, max(abs(J - J_num), [], 'all'));

    %% 3. Toolbox: geometricJacobian liefert [w; v] -> umsortieren in [v; w]
    J_tb = geometricJacobian(robot, q, 'kamera');
    J_tb = [J_tb(4:6, :); J_tb(1:3, :)];
    fehler_tb = max(fehler_tb, max(abs(J - J_tb), [], 'all'));
end
fprintf('  200 Zufallsposen: max. Abw. zur numerischen Ableitung = %.2e\n', fehler_num);
fprintf('  200 Zufallsposen: max. Abw. zu geometricJacobian      = %.2e\n', fehler_tb);
assert(fehler_num < 1e-6,  'Jacobi weicht von der numerischen Ableitung ab');
assert(fehler_tb  < 1e-12, 'Jacobi weicht von geometricJacobian ab');

%% 4. Singularität der Orientierungs-Jacobi J_w (Zeilen 4-6)
% 4a: innerhalb der Gelenkgrenzen -> kleinster Singulärwert muss > 0 sein
rng(3);
sigma_min = inf;
for k = 1:5000
    q = Z1.q_min + rand(6,1) .* (Z1.q_max - Z1.q_min);
    J = z1_jacobian(q, Z1, MONT.T_6C);
    s = svd(J(4:6, :));
    sigma_min = min(sigma_min, s(3));
end
fprintf('  J_w innerhalb der Grenzen: kleinster Singulärwert = %.3f\n', sigma_min);
assert(sigma_min > 1e-3, 'J_w wird innerhalb der Grenzen singulär');

% 4b: Gimbal Lock außerhalb der Grenzen: q2+q3+q4 = 0 und q5 = 90°
q_sing = [0; 0.5; -0.5; 0; pi/2; 0];
J = z1_jacobian(q_sing, Z1, MONT.T_6C);
s = svd(J(4:6, :));
fprintf('  J_w bei q234 = 0, q5 = 90°:    kleinster Singulärwert = %.1e (singulär)\n', s(3));
assert(s(3) < 1e-12, 'Erwartete Singularität wurde nicht gefunden');

disp('test_z1_jacobian: OK');
end