function e = rot_log(R)
% ROT_LOG  Matrix-Logarithmus auf SO(3):  R = Rot(a, theta)  ->  e = a * theta
% Quelle: Modern Robotics Kap. 3.2.3.3, Gl. (3.53), (3.54), Sonderfall theta = pi
%   cos(theta) = (tr R - 1)/2,     a = vee(R - R') / (2 sin(theta))

v = [R(3,2) - R(2,3);  R(1,3) - R(3,1);  R(2,1) - R(1,2)];  % = 2 sin(theta) * a
s = norm(v) / 2;                 % sin(theta)
c = (trace(R) - 1) / 2;          % cos(theta)          (Gl. 3.54)
theta = atan2(s, c);             % robust für alle theta in [0, pi]

if theta < 1e-6
    % sehr kleine Winkel: sin(theta) ~ theta  ->  e ~ v/2
    e = v / 2;
elseif theta < pi - 1e-4
    % Normalfall                                       (Gl. 3.53)
    e = theta / (2*sin(theta)) * v;
else
    % nahe pi: sin(theta) ~ 0, Achse aus dem symmetrischen Anteil
    % (R + R')/2 = cos(theta)*I + (1 - cos(theta)) * a*a'
    B = ((R + R')/2 - c*eye(3)) / (1 - c);      % = a*a'
    [~, k] = max(diag(B));                       % numerisch beste Spalte
    a = B(:,k) / sqrt(B(k,k));
    if dot(a, v) < 0, a = -a; end                % Vorzeichen aus antisymm. Anteil
    e = theta * a;
end
end