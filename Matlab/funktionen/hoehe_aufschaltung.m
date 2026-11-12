function [h, v_rumpf] = hoehe_aufschaltung(q, R_WB, w_B, z_B, dz_B, Z1, MONT)
% HOEHE_AUFSCHALTUNG  Höhe des Endeffektors und die Vertikalgeschwindigkeit,
% die allein die Rumpfbewegung am Endeffektor verursacht (Arm steif gedacht).
%
% Quelle: Meng et al. 2026, Gl. (3):  x_dot = J*q_dot + J_B*V_B
%         Rumpf-Anteil für einen starren Hebel: v = v_B + w x r  (Roncone 2014, Gl. 13a)
% Übertragung auf den Z1 (nur Höhe, Weltsystem W):
%   r_W     = R_WB*(p_B0 + R_B0*p_0C(q))     Hebel Rumpfmitte -> Endeffektor
%   w_W     = R_WB*w_B                        Gyro (Frame B) ins Weltsystem
%   v_rumpf = dz_B + e_z'*(w_W x r_W)
[h, p_WC] = ee_hoehe(q, R_WB, z_B, Z1, MONT);
r_W     = p_WC - [0; 0; z_B];
w_W     = R_WB * w_B(:);
v_hebel = cross(w_W, r_W);
v_rumpf = dz_B + v_hebel(3);
end