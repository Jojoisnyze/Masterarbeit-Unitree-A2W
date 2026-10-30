function R = rot_achse(a, theta)
% ROT_ACHSE  Drehmatrix für eine Drehung um die Einheitsachse a mit Winkel theta.
% Quelle: Rodrigues-Formel, Modern Robotics Gl. (3.51), Proposition 3.11
%   Rot(a, theta) = I + sin(theta)*[a] + (1 - cos(theta))*[a]^2
a = a / norm(a);          % zur Sicherheit normieren
A = skew(a);
R = eye(3) + sin(theta)*A + (1 - cos(theta))*(A*A);
end