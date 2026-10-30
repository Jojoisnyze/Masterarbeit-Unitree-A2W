function [J, T_0C] = z1_jacobian(q, Z1, T_6C)
% Z1_JACOBIAN  Geometrische Jacobi-Matrix der Kamera im Frame 0.
%
%   [v_C; w_C] = J(q) * dq      Zeilen [v; w] (wie Roncone Gl. 12)
%
% Quelle: Roncone et al. (2014) Gl. 12; Modern Robotics Kap. 5.1.1 (Gl. 5.8-5.11)
% Spalte i für ein Drehgelenk (nur Gelenk i dreht mit 1 rad/s):
%   w = a_i                       Drehgeschwindigkeit um die Gelenkachse
%   v = a_i x (p_C - p_i)         Starrkörper: v = w x Hebelarm
%
% Eingang:  q     6x1 Gelenkwinkel [rad]
%           Z1    Struct aus par_z1
%           T_6C  4x4 Kameramontage (aus par_montage)
% Ausgang:  J     6x6 Jacobi-Matrix, Zeilen 1-3: v_C [m/s], 4-6: w_C [rad/s]
%           T_0C  4x4 Kamerapose (fällt ohnehin an, spart zweite FK-Rechnung)

[T_0C, T_0i] = z1_fk(q, Z1, T_6C);
p_C = T_0C(1:3, 4);                     % Position der Kamera im Frame 0

J = zeros(6, Z1.n);
for i = 1:Z1.n
    R_0i = T_0i(1:3, 1:3, i);           % Orientierung von Frame i
    p_i  = T_0i(1:3, 4, i);             % Punkt auf der Gelenkachse i
    a_i  = R_0i * Z1.achse(:, i);       % Gelenkachse i im Frame 0

    J(1:3, i) = cross(a_i, p_C - p_i);  % lineare Geschwindigkeit der Kamera
    J(4:6, i) = a_i;                    % Drehgeschwindigkeit der Kamera
end
end