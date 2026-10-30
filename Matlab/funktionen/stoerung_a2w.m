function A2W = stoerung_a2w(t, STOER)
% STOERUNG_A2W  Rumpfbewegung des A2W: Lage R_WB(t) und dazu exakt passende
% Winkelgeschwindigkeit w_B(t) im Rumpf-Frame (das, was das Gyroskop misst).
%
% Quelle:
%   R_WB = Rot(z,psi)*Rot(y,theta)*Rot(x,phi)     Modern Robotics Anh. B, Gl. B.2/B.8
%   [w_B] = R_WB' * dR_WB/dt                      Modern Robotics Kap. 3.2.2
%   =>  w_B = [ dphi - sin(th)*dpsi
%               cos(phi)*dth + sin(phi)*cos(th)*dpsi
%              -sin(phi)*dth + cos(phi)*cos(th)*dpsi ]
%
% Eingang:  t      Zeitvektor [s] (beliebige Länge N, auch skalar)
%           STOER  Struct aus par_stoerung
% Ausgang:  A2W.t     Nx1
%           A2W.eul   Nx3  [phi theta psi] [rad]
%           A2W.deul  Nx3  Euler-Raten [rad/s]
%           A2W.z     Nx1  Heave [m],  A2W.dz  Nx1 [m/s]
%           A2W.R_WB  3x3xN Lage des Rumpfes im Weltsystem
%           A2W.w_B   Nx3  Winkelgeschwindigkeit im Rumpf-Frame B [rad/s]

t  = t(:);
N  = numel(t);
x  = zeros(N, 4);
dx = zeros(N, 4);
for k = 1:4
    [x(:,k), dx(:,k)] = kanal(t, k, STOER);
end

% weiches Einblenden: x_neu = e*x  ->  dx_neu = de*x + e*dx  (Produktregel)
[e, de] = einblenden(t, STOER.T_ein);
dx = de .* x + e .* dx;
x  = e .* x;

phi = x(:,1);   th = x(:,2);   psi = x(:,3);
dphi = dx(:,1); dth = dx(:,2); dpsi = dx(:,3);

% Winkelgeschwindigkeit im Rumpf-Frame (Herleitung siehe Kopf)
w_B = [ dphi - sin(th).*dpsi, ...
        cos(phi).*dth + sin(phi).*cos(th).*dpsi, ...
       -sin(phi).*dth + cos(phi).*cos(th).*dpsi ];

% Lage des Rumpfes
R_WB = zeros(3, 3, N);
for n = 1:N
    R_WB(:,:,n) = rot_achse([0;0;1], psi(n)) * ...
                  rot_achse([0;1;0], th(n))  * ...
                  rot_achse([1;0;0], phi(n));
end

A2W.t    = t;
A2W.eul  = x(:, 1:3);
A2W.deul = dx(:, 1:3);
A2W.z    = x(:, 4);
A2W.dz   = dx(:, 4);
A2W.R_WB = R_WB;
A2W.w_B  = w_B;
end

% ======================= lokale Hilfsfunktionen ===========================
function [x, dx] = kanal(t, k, S)
% Ein Kanal = Summe von Sinus + Summe von Gaußpulsen, jeweils mit exakter Ableitung
x  = zeros(size(t));
dx = zeros(size(t));
% 1) Sinus-Anteile (Gang, Gelände, Rauschen)
for j = 1:size(S.sin.A, 2)
    w  = 2*pi*S.sin.f(k,j);
    x  = x  + S.sin.A(k,j) * sin(w*t + S.sin.phase(k,j));
    dx = dx + S.sin.A(k,j) * w * cos(w*t + S.sin.phase(k,j));
end
% 2) Gaußpulse (Schlaglöcher, Fußaufsetzer)
%    g = exp(-(t-t0)^2/(2 s^2)),   dg/dt = -(t-t0)/s^2 * g
for m = 1:numel(S.stoss.t0)
    s  = S.stoss.sigma(m);
    g  = exp(-(t - S.stoss.t0(m)).^2 / (2*s^2));
    x  = x  + S.stoss.A(k,m) * g;
    dx = dx - S.stoss.A(k,m) * (t - S.stoss.t0(m)) / s^2 .* g;
end
end


function [e, de] = einblenden(t, T)
% Cosinus-Rampe von 0 auf 1 in der Zeit T (stetig in Wert und Steigung bei t = 0)
e  = ones(size(t));
de = zeros(size(t));
if T > 0
    idx     = t < T;
    e(idx)  = 0.5 * (1 - cos(pi * t(idx) / T));
    de(idx) = 0.5 * pi / T * sin(pi * t(idx) / T);
end
end