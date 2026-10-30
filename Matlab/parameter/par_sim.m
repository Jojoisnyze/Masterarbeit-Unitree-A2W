function SIM = par_sim(pose)
% PAR_SIM  Zentrale Simulationsparameter und Grundpose des Arms.
% Aufruf: SIM = par_sim("vorn")  oder  par_sim("seite")   (Testprotokoll Schritt 5)
arguments
    pose (1,1) string = "vorn"
end
SIM.Ts   = 0.002;   % Abtastzeit des Reglers [s] (500 Hz). Zielhardware noch offen
SIM.Tend = 30;      % Simulationsdauer [s]

% Grundpose q0 [rad]: q2+q3+q4 = 0 -> Kamera waagerecht, q4 in Bereichsmitte,
% alle Gelenke >= 69° vom Anschlag, sigma_min(J_w) = 1
SIM.pose = pose;
switch pose
    case "vorn",  SIM.q0 = [0;    1.2; -1.2; 0; 0; 0];   % Blick nach vorn
    case "seite", SIM.q0 = [pi/2; 1.2; -1.2; 0; 0; 0];   % Blick nach links (Rebzeile)
    otherwise, error('Unbekannte Pose: %s', pose);
end
end