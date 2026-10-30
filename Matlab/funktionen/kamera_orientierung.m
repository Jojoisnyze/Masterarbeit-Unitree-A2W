function [R_WC, R_W0] = kamera_orientierung(q, R_WB, Z1, MONT)
% KAMERA_ORIENTIERUNG  Orientierung der Kamera im Weltsystem.
%   R_WC = R_WB * R_B0 * R_0C(q)     (Modellgrundlagen, Kette W -> B -> 0 -> C)
% Wird in Simulink von "Kamera_Orientierung" (Regler) und "Gütemaß"
% (Auswertung) aufgerufen, damit beide garantiert dasselbe rechnen.
T_0C = z1_fk(q, Z1, MONT.T_6C);          % ERSTER Ausgang = Pose der Kamera
R_W0 = R_WB * MONT.R_B0;
R_WC = R_W0 * T_0C(1:3, 1:3);
end