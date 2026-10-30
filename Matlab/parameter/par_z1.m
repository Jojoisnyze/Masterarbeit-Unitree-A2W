function [Z1, info] = par_z1(urdfDatei)
% PAR_Z1  Liest die Kinematik des Unitree Z1 aus der URDF-Datei.
%
% Quelle: ROS-URDF-Spezifikation, <joint>:
%   T_{Vorgänger,i}(q_i) = Tr(xyz_i) * R_rpy,i * Rot(a_i, q_i)
%   R_rpy = Rz(yaw) * Ry(pitch) * Rx(roll)
% Der base_static_joint (world -> link00) wird bewusst nicht übernommen,
% im Modell ersetzt ihn die Kette W -> B -> 0 (Störung + Montage T_B0).
%
% Ausgabe Z1 (nur Zahlen mit fester Größe, später codegen-tauglich):
%   Z1.n        Anzahl Gelenke
%   Z1.p_off    3xn    Versatz von Gelenk i im Frame des Vorgängerlinks [m]
%   Z1.R_off    3x3xn  Drehung aus rpy (beim Z1 überall Einheitsmatrix)
%   Z1.achse    3xn    Drehachse a_i im eigenen Gelenkframe (Einheitsvektor)
%   Z1.q_min    nx1    untere Winkelgrenze [rad]
%   Z1.q_max    nx1    obere Winkelgrenze [rad]
%   Z1.dq_max   nx1    Geschwindigkeitsgrenze [rad/s]
%   Z1.tau_max  nx1    Momentengrenze [Nm]
% Ausgabe info: Gelenk- und Linknamen, nur zur Kontrolle

urdf = readstruct(urdfDatei, 'FileType', 'xml');

% --- 1. Nur die Drehgelenke behalten -----------------------------------
alle    = urdf.joint;
istDreh = arrayfun(@(j) string(j.typeAttribute) == "revolute", alle);
gelenke = alle(istDreh);
n       = numel(gelenke);

% --- 2. Kette sortieren: bei link00 beginnen, immer zum Kind weiter -------
% (macht uns unabhängig von der Reihenfolge in der Datei)
reihenfolge = zeros(1, n);
link = "link00";
for i = 1:n
    k = find(arrayfun(@(j) string(j.parent.linkAttribute) == link, gelenke));
    assert(numel(k) == 1, 'Kette nicht eindeutig bei %s', link);
    reihenfolge(i) = k;
    link = string(gelenke(k).child.linkAttribute);
end
gelenke = gelenke(reihenfolge);

% --- 3. Werte auslesen --------------------------------------------------
Z1.n       = n;
Z1.p_off   = zeros(3, n);
Z1.R_off   = zeros(3, 3, n);
Z1.achse   = zeros(3, n);
Z1.q_min   = zeros(n, 1);
Z1.q_max   = zeros(n, 1);
Z1.dq_max  = zeros(n, 1);
Z1.tau_max = zeros(n, 1);
info.gelenk = strings(n, 1);
info.parent = strings(n, 1);
info.child  = strings(n, 1);

for i = 1:n
    g = gelenke(i);
    info.gelenk(i) = string(g.nameAttribute);
    info.parent(i) = string(g.parent.linkAttribute);
    info.child(i)  = string(g.child.linkAttribute);

    % Versatz und Drehung des Gelenkframes
    Z1.p_off(:, i)    = zahlen(g.origin.xyzAttribute);
    Z1.R_off(:, :, i) = rpy2rotm_urdf(zahlen(g.origin.rpyAttribute));

    % Drehachse (normiert)
    a = zahlen(g.axis.xyzAttribute);
    Z1.achse(:, i) = a / norm(a);

    % Grenzen
    Z1.q_min(i)   = zahlen(g.limit.lowerAttribute);
    Z1.q_max(i)   = zahlen(g.limit.upperAttribute);
    Z1.dq_max(i)  = zahlen(g.limit.velocityAttribute);
    Z1.tau_max(i) = zahlen(g.limit.effortAttribute);
end
end

% ======================= lokale Hilfsfunktionen ===========================
function v = zahlen(x)
% Wandelt einen URDF-Attributwert ("0 0 0.0585" oder 30) in einen Zahlen-
% Spaltenvektor um. readstruct liefert je nach Inhalt string oder double.
v = str2double(split(strtrim(string(x))));
end

function R = rpy2rotm_urdf(rpy)
% URDF-Konvention: feste Achsen, erst Roll um x, dann Pitch um y, dann Yaw um z
% -> R = Rz(yaw) * Ry(pitch) * Rx(roll)
r = rpy(1); p = rpy(2); y = rpy(3);
Rx = [1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
Ry = [cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
Rz = [cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];
R  = Rz * Ry * Rx;
end