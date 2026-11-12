function A2W = stoerung_a2w(t, STOER)
% STOERUNG_A2W  Rumpfbewegung des A2W: Lage R_WB(t) und dazu exakt passende
% Winkelgeschwindigkeit w_B(t) im Rumpf-Frame (das, was das Gyroskop misst),
% Hub z_B mit Ableitungen und die spezifische Kraft f_B (das, was der
% Beschleunigungsmesser misst).
%
% Quelle:
%   R_WB = Rot(z,psi)*Rot(y,theta)*Rot(x,phi)     Modern Robotics Anh. B, Gl. B.2/B.8
%   [w_B] = R_WB' * dR_WB/dt                      Modern Robotics Kap. 3.2.2
%   =>  w_B = [ dphi - sin(th)*dpsi
%               cos(phi)*dth + sin(phi)*cos(th)*dpsi
%              -sin(phi)*dth + cos(phi)*cos(th)*dpsi ]
%   f_B = R_WB' * (a_W + g*e_z),  a_W = [0; 0; ddz]   Meng et al. 2026, Gl. (43)
%   (Längs-/Querbeschleunigung wird nicht modelliert: konstante Fahrt)
%
% Eingang:  t      Zeitvektor [s] (beliebige Länge N, auch skalar)
%           STOER  Struct aus par_stoerung
% Ausgang:  A2W.t     Nx1
%           A2W.eul   Nx3  [phi theta psi] [rad]
%           A2W.deul  Nx3  Euler-Raten [rad/s]
%           A2W.z     Nx1  Hub [m],  A2W.dz  Nx1 [m/s],  A2W.ddz  Nx1 [m/s^2]
%           A2W.R_WB  3x3xN Lage des Rumpfes im Weltsystem
%           A2W.w_B   Nx3  Winkelgeschwindigkeit im Rumpf-Frame B [rad/s]
%           A2W.f_B   Nx3  spezifische Kraft im Rumpf-Frame B [m/s^2]

t   = t(:);
N   = numel(t);
x   = zeros(N, 4);
dx  = zeros(N, 4);
ddx = zeros(N, 4);
for k = 1:4
    [x(:,k), dx(:,k), ddx(:,k)] = kanal(t, k, STOER);
end

% weiches Einblenden: x_neu = e*x  (Produktregel)
%   dx_neu  = de*x + e*dx
%   ddx_neu = dde*x + 2*de*dx + e*ddx
[e, de, dde] = einblenden(t, STOER.T_ein);
ddx = dde .* x + 2 * de .* dx + e .* ddx;      % zuerst, solange x, dx noch ungeblendet
dx  = de .* x + e .* dx;
x   = e .* x;

phi = x(:,1);   th = x(:,2);   psi = x(:,3);
dphi = dx(:,1); dth = dx(:,2); dpsi = dx(:,3);

% Winkelgeschwindigkeit im Rumpf-Frame (Herleitung siehe Kopf)
w_B = [ dphi - sin(th).*dpsi, ...
        cos(phi).*dth + sin(phi).*cos(th).*dpsi, ...
       -sin(phi).*dth + cos(phi).*cos(th).*dpsi ];

% Lage des Rumpfes und spezifische Kraft im Rumpf-Frame
R_WB = zeros(3, 3, N);
f_B  = zeros(N, 3);
for n = 1:N
    R_WB(:,:,n) = rot_achse([0;0;1], psi(n)) * ...
                  rot_achse([0;1;0], th(n))  * ...
                  rot_achse([1;0;0], phi(n));
    f_B(n,:) = (R_WB(:,:,n).' * [0; 0; ddx(n,4) + STOER.g]).';
end

A2W.t    = t;
A2W.eul  = x(:, 1:3);
A2W.deul = dx(:, 1:3);
A2W.z    = x(:, 4);
A2W.dz   = dx(:, 4);
A2W.ddz  = ddx(:, 4);
A2W.R_WB = R_WB;
A2W.w_B  = w_B;
A2W.f_B  = f_B;
end

% ======================= lokale Hilfsfunktionen ===========================
function [x, dx, ddx] = kanal(t, k, S)
% Ein Kanal = Summe von Sinus + Summe von Gaußpulsen, mit exakter 1. und 2. Ableitung
x   = zeros(size(t));
dx  = zeros(size(t));
ddx = zeros(size(t));
% 1) Sinus-Anteile (Gang, Gelände, Rauschen)
for j = 1:size(S.sin.A, 2)
    w   = 2*pi*S.sin.f(k,j);
    arg = w*t + S.sin.phase(k,j);
    x   = x   + S.sin.A(k,j) * sin(arg);
    dx  = dx  + S.sin.A(k,j) * w   * cos(arg);
    ddx = ddx - S.sin.A(k,j) * w^2 * sin(arg);
end
% 2) Gaußpulse (Schlaglöcher, Fußaufsetzer)
%    g = exp(-(t-t0)^2/(2 s^2))
%    dg/dt   = -(t-t0)/s^2 * g
%    d2g/dt2 = ((t-t0)^2/s^4 - 1/s^2) * g
for m = 1:numel(S.stoss.t0)
    s   = S.stoss.sigma(m);
    d   = t - S.stoss.t0(m);
    g   = exp(-d.^2 / (2*s^2));
    x   = x   + S.stoss.A(k,m) * g;
    dx  = dx  - S.stoss.A(k,m) * d / s^2 .* g;
    ddx = ddx + S.stoss.A(k,m) * (d.^2 / s^4 - 1/s^2) .* g;
end
end


function [e, de, dde] = einblenden(t, T)
% Cosinus-Rampe von 0 auf 1 in der Zeit T (stetig in Wert und Steigung bei t = 0)
%   e = 0.5*(1 - cos(pi t/T)),  de = 0.5*(pi/T)*sin(pi t/T),  dde = 0.5*(pi/T)^2*cos(pi t/T)
e   = ones(size(t));
de  = zeros(size(t));
dde = zeros(size(t));
if T > 0
    idx      = t < T;
    e(idx)   = 0.5 * (1 - cos(pi * t(idx) / T));
    de(idx)  = 0.5 * pi / T * sin(pi * t(idx) / T);
    dde(idx) = 0.5 * (pi / T)^2 * cos(pi * t(idx) / T);
end
end