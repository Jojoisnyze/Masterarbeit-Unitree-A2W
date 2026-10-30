function [T_0C, T_0i] = z1_fk(q, Z1, T_6C)
% Z1_FK  Vorwärtskinematik des Z1: Pose der Kamera im Basisframe 0 (link00).
%
% Quelle: Modern Robotics Kap. 3.3 (homogene Transformationen) und Kap. 4.2 (URDF)
%   T_{i-1,i}(q_i) = [R_off,i p_off,i; 0 1] * [Rot(a_i,q_i) 0; 0 1]
%   T_0C(q)        = T_01(q1) * T_12(q2) * ... * T_56(q6) * T_6C
%
% Eingang:  q     6x1 Gelenkwinkel [rad]
%           Z1    Struct aus par_z1 (Versätze, Achsen)
%           T_6C  4x4 Kameramontage an link06 (aus par_montage)
% Ausgang:  T_0C  4x4 Pose der Kamera im Frame 0
%           T_0i  4x4x6 Pose von Frame i (link0i) im Frame 0. Ursprung p_i und
%                 Achse a_i braucht später die Jacobi-Matrix.

T    = eye(4);
T_0i = zeros(4, 4, Z1.n);

for i = 1:Z1.n
    % fester Teil aus der URDF: Versatz + Drehung (origin xyz/rpy)
    T_fest = [Z1.R_off(:,:,i), Z1.p_off(:,i); 0 0 0 1];
    % variabler Teil: Drehung um die Gelenkachse mit q_i
    T_gel  = [rot_achse(Z1.achse(:,i), q(i)), zeros(3,1); 0 0 0 1];

    T = T * T_fest * T_gel;     % weiter entlang der Kette
    T_0i(:,:,i) = T;
end

T_0C = T * T_6C;                % Kamera fest an link06
end