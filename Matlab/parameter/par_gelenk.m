function GEL = par_gelenk(Z1)
% PAR_GELENK  Gelenkmodell des Z1 ab Stufe 2 (Variant "Realistisch").
% Geschwindigkeitsbefehl -> Sättigung -> PT1 (interne Motorregelung) -> Integrator mit Anschlägen
% !!! PLATZHALTER: T aus Sprungantwort am echten Z1 bestimmen !!!
GEL.T      = 0.010;          % Zeitkonstante Gelenk-Geschwindigkeitsregelung [s] (PLATZHALTER)
GEL.dq_max = Z1.dq_max(:);   % Geschwindigkeitsgrenzen aus der URDF [rad/s]
GEL.q_min  = Z1.q_min(:);    % Winkelgrenzen aus der URDF [rad]
GEL.q_max  = Z1.q_max(:);
end