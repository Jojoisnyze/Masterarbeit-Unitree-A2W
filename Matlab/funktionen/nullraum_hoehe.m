function dq = nullraum_hoehe(dq1, v_soll, q, R_WB, Z1, MONT, lambda)
% NULLRAUM_HOEHE  Höhenaufgabe im Nullraum der Orientierungsaufgabe (Task-Priority).
%
% Quelle: Meng et al. 2026, Gl. (11):  N = I - J^+ J
%         Meng et al. 2026, Gl. (17):  Primäraufgabe + N * Nullraum-Anteil
%         Nullraum-Anteil so gewählt, dass die Zweitaufgabe bestmöglich erfüllt
%         wird: Task-Priority nach Siciliano & Slotine (1991)
%         Dämpfung lambda wie Buss (2004)
% Übertragung auf den Z1:
%   Primär (Orientierung):  dq1 = J_w# * w_soll         (Block "Gelenkgeschwindigkeit")
%   Sekundär (Höhe):        j_h = e_z' * R_W0 * J_v      dh/dt durch die Gelenke (1x6)
%   N  = I - pinv(J_w)*J_w   UNGEDÄMPFT -> J_w*N = 0 exakt: Orientierung bleibt unberührt
%   dq = dq1 + N*jN' * (v_soll - j_h*dq1) / (jN*jN' + lambda^2),   jN = j_h*N
J    = z1_jacobian(q, Z1, MONT.T_6C);        % Zeilen [v; w], Frame 0
J_v  = J(1:3, :);
J_w  = J(4:6, :);
R_W0 = R_WB * MONT.R_B0;
N    = eye(6) - pinv(J_w) * J_w;
j_h  = [0 0 1] * R_W0 * J_v;
jN   = j_h * N;
dq   = dq1(:) + N * jN.' * ((v_soll - j_h*dq1(:)) / (jN*jN.' + lambda^2));
end