function [e_0, e_W] = orient_fehler(R_WC, R_WC_des, R_W0)
% ORIENT_FEHLER  Orientierungsfehler der Kamera als Achse*Winkel.
%
% Quelle: Modellgrundlagen, Abschnitt Regelgesetz; Modern Robotics Kap. 3.2.3.3
%   R_err = R_WC_des * R_WC'    Drehung von Ist nach Soll (Weltsystem)
%   e_W   = log(R_err)          Achse * Winkel [rad], Weltsystem
%   e_0   = R_W0' * e_W         derselbe Vektor im Basisframe 0 des Z1
% Für kleine Fehler gilt de/dt = -w_C  ->  mit w_C = Kp*e: de/dt = -Kp*e (PT1)

R_err = R_WC_des * R_WC';
e_W   = rot_log(R_err);
e_0   = R_W0' * e_W;
end