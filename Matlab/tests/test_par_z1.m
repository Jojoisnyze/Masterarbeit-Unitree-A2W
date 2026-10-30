function test_par_z1(Z1, info)
% TEST_PAR_Z1  Prüft die eingelesenen URDF-Werte gegen die Tabelle im
% Dokument "Modellgrundlagen", Abschnitt "Z1-Kinematik aus der URDF".
tol = 1e-9;

p_soll = [ 0      0  0.0585
    0      0  0.045
    -0.35   0  0
    0.218  0  0.057
    0.07   0  0
    0.0492 0  0     ]';
a_soll = [0 0 1; 0 1 0; 0 1 0; 0 1 0; 0 0 1; 1 0 0]';
lim_soll_grad = [-150 150; 0 170; -165 0; -87 87; -77 77; -160 160];

assert(Z1.n == 6, 'Es müssen 6 Drehgelenke sein');
assert(all(info.gelenk == "joint" + (1:6)'), 'Reihenfolge der Gelenke falsch');
assert(max(abs(Z1.p_off - p_soll), [], 'all') < tol, 'Versätze stimmen nicht');
assert(max(abs(Z1.achse - a_soll), [], 'all') < tol, 'Achsen stimmen nicht');
for i = 1:6
    assert(norm(Z1.R_off(:,:,i) - eye(3)) < tol, 'rpy ~= 0 bei Gelenk %d', i);
end
assert(max(abs(rad2deg([Z1.q_min Z1.q_max]) - lim_soll_grad), [], 'all') < 0.5, ...
    'Winkelgrenzen stimmen nicht');

% Übersicht ausgeben
T = table(info.gelenk, Z1.p_off', Z1.achse', ...
    round(rad2deg(Z1.q_min),1), round(rad2deg(Z1.q_max),1), ...
    Z1.dq_max, Z1.tau_max, ...
    'VariableNames', {'Gelenk','Versatz_m','Achse', ...
    'qmin_grad','qmax_grad','dqmax_rads','taumax_Nm'});
disp(T);
disp('test_par_z1: OK');
end