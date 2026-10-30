function J_pinv = dls_pinv(J, lambda)
% DLS_PINV  Gedämpfte Pseudoinverse (Damped Least Squares).
%
%   J# = J' * (J*J' + lambda^2 * I)^-1
%
% Quelle: Buss (2004); für lambda = 0 die Pseudoinverse nach Modern Robotics
% Kap. 6.2 (unter allen Lösungen die mit kleinster Gelenkgeschwindigkeit).
% Wirkung je Singulärwert sigma: 1/sigma wird zu sigma/(sigma^2 + lambda^2)
%   -> bei sigma >> lambda fast exakt, bei sigma -> 0 begrenzt statt unendlich.
m = size(J, 1);
J_pinv = J' / (J*J' + lambda^2 * eye(m));    % "/" statt inv(): numerisch besser
end