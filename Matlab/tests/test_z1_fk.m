function test_z1_fk(Z1, MONT, urdfDatei)
% TEST_Z1_FK  Prüft die eigene Vorwärtskinematik in drei Stufen:
%   1. Handrechnung bei q = 0
%   2. Eigenschaften der Drehmatrix (orthonormal, det = 1)
%   3. Vergleich mit der Robotics System Toolbox (nur als Referenz)
tol = 1e-10;

%% 1. Handrechnung bei q = 0
% Alle Drehungen sind null -> Position = Summe der Versätze aus der URDF
[T_0C, T_0i] = z1_fk(zeros(6,1), Z1, MONT.T_6C);
p6_hand = [0 - 0.35 + 0.218 + 0.07 + 0.0492;   % x
    0;                                  % y
    0.0585 + 0.045 + 0.057];            % z
assert(norm(T_0i(1:3,4,6) - p6_hand) < tol, 'q=0: Position link06 falsch');
assert(norm(T_0C(1:3,4) - (p6_hand + MONT.p_6C)) < tol, 'q=0: Kameraposition falsch');
assert(norm(T_0C(1:3,1:3) - MONT.R_6C) < tol, 'q=0: Kameraorientierung falsch');
fprintf('  q = 0 (Arm gefaltet): p_C = [%.4f  %.4f  %.4f] m\n', T_0C(1:3,4));

%% 2. + 3. Zufällige Posen innerhalb der Gelenkgrenzen
robot  = importrobot(urdfDatei, 'DataFormat', 'column');
kamera = rigidBody('kamera');
setFixedTransform(kamera.Joint, MONT.T_6C);
addBody(robot, kamera, 'link06');

rng(1);                         % reproduzierbare Zufallsposen
fehler_max = 0;
for k = 1:200
    q = Z1.q_min + rand(6,1) .* (Z1.q_max - Z1.q_min);
    [T_0C, T_0i] = z1_fk(q, Z1, MONT.T_6C);

    % 2. Drehmatrix muss orthonormal sein mit det = +1
    R = T_0C(1:3,1:3);
    assert(norm(R'*R - eye(3)) < tol && abs(det(R) - 1) < tol, ...
        'R_0C ist keine gültige Drehmatrix');

    % 3. Vergleich Kamera und alle Zwischenframes mit der Toolbox
    T_ref = getTransform(robot, q, 'kamera', 'link00');
    fehler_max = max(fehler_max, max(abs(T_0C - T_ref), [], 'all'));
    for i = 1:6
        T_ref_i = getTransform(robot, q, sprintf('link%02d', i), 'link00');
        fehler_max = max(fehler_max, max(abs(T_0i(:,:,i) - T_ref_i), [], 'all'));
    end
end
fprintf('  200 Zufallsposen: max. Abweichung zur Toolbox = %.2e\n', fehler_max);
assert(fehler_max < tol, 'Vorwärtskinematik weicht von der Toolbox ab');
disp('test_z1_fk: OK');
end