function test_stoerung(SIM)
% TEST_STOERUNG  Prüft das Störmodell für beide Betriebsarten:
%   1. w_B passt exakt zu R_WB, dz zu z (numerische Ableitung)
%   2. R_WB ist eine gültige Drehmatrix
%   3. Kennwerte ausgeben (RMS, Maximum, max. Drehrate)
%   4. Einzelachse 'roll': nur w_x ~= 0
h = 1e-6;

for szen = ["fahren", "laufen"]
    STOER = par_stoerung(szen, SIM);

    %% 1. + 2. Konsistenz an 200 zufälligen Zeitpunkten
    zufall = RandStream('mt19937ar', 'Seed', 7);
    t_test = 0.5*STOER.T_ein + rand(zufall, 200, 1) * (SIM.Tend - STOER.T_ein);
    fehler_w = 0;  fehler_z = 0;
    for k = 1:numel(t_test)
        A = stoerung_a2w(t_test(k) + [-h; 0; h], STOER);
        R = A.R_WB(:,:,2);
        assert(norm(R'*R - eye(3)) < 1e-12 && abs(det(R) - 1) < 1e-12, 'R_WB ungültig');
        W = R' * (A.R_WB(:,:,3) - A.R_WB(:,:,1)) / (2*h);    % [w_B] = R' dR/dt
        fehler_w = max(fehler_w, norm([W(3,2); W(1,3); W(2,1)] - A.w_B(2,:)'));
        fehler_z = max(fehler_z, abs((A.z(3) - A.z(1))/(2*h) - A.dz(2)));
    end
    assert(fehler_w < 1e-6, 'w_B passt nicht zu R_WB (%s)', szen);
    assert(fehler_z < 1e-6, 'dz passt nicht zu z (%s)', szen);

    %% 3. Kennwerte (ohne Einblendphase)
    A   = stoerung_a2w((0:SIM.Ts:SIM.Tend)', STOER);
    i   = A.t >= STOER.T_ein;
    e   = rad2deg(A.eul(i,:));  w = rad2deg(A.w_B(i,:));  z = 100*A.z(i);
    eff = @(x) sqrt(mean(x.^2));                         % RMS
    fprintf('  %-6s Konsistenz w: %.1e rad/s, dz: %.1e m/s\n', szen, fehler_w, fehler_z);
    fprintf('         RMS/max: Roll %.1f/%.1f°  Nick %.1f/%.1f°  Gier %.1f/%.1f°  Heave %.1f/%.1f cm\n', ...
            eff(e(:,1)), max(abs(e(:,1))), eff(e(:,2)), max(abs(e(:,2))), ...
            eff(e(:,3)), max(abs(e(:,3))), eff(z), max(abs(z)));
    fprintf('         max |w|: x %.0f  y %.0f  z %.0f °/s   (Z1-Grenze: 180 °/s je Gelenk)\n', ...
            max(abs(w)));
    fprintf('         max |w_B - Euler-Raten| = %.1f °/s\n', ...
            rad2deg(max(vecnorm(A.w_B(i,:) - A.deul(i,:), 2, 2))));
end

%% 4. Einzelachse: nur Rollen -> nur w_x
A = stoerung_a2w((0:SIM.Ts:SIM.Tend)', par_stoerung('roll', SIM));
assert(all(abs(A.w_B(:,2:3)) < 1e-15, 'all') && all(A.z == 0), ...
       'Szenario roll erzeugt Bewegung in anderen Achsen');
fprintf('  roll   Testsinus: max. %.2f°, max. w_x = %.1f °/s\n', ...
        rad2deg(max(abs(A.eul(:,1)))), rad2deg(max(abs(A.w_B(:,1)))));
disp('test_stoerung: OK');
end