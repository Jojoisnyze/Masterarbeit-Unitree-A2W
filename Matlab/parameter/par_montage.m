function MONT = par_montage()
% PAR_MONTAGE  Feste Montagetransformationen.
% !!! ALLE WERTE SIND PLATZHALTER (Modellgrundlagen, "Offene Parameter") !!!

% --- Kamera an link06: T_6C ---------------------------------------------
% PLATZHALTER: 5 cm in x von link06, Blickrichtung entlang x von link06.
% -> am Aufbau messen
MONT.p_6C = [0.05; 0; 0];        % [m]
MONT.R_6C = eye(3);
MONT.T_6C = [MONT.R_6C, MONT.p_6C; 0 0 0 1];

% --- Z1-Basis (link00) auf A2W-Rumpf: T_B0 -------------------------------
% PLATZHALTER: R_B0 = I (Z1 gerade auf dem Rumpf), Position noch offen.
% -> am Aufbau messen
MONT.p_B0 = [0; 0; 0];           % [m]
MONT.R_B0 = eye(3);
MONT.T_B0 = [MONT.R_B0, MONT.p_B0; 0 0 0 1];
end