function REG = par_regler(SIM, Z1, MONT)
% PAR_REGLER  Parameter des Stabilisierungsreglers (Grundgerüst).

% Rückführverstärkung [1/s]: Fehler klingt mit T = 1/Kp = 0,33 s ab.
% Quelle Startwert: EMMa Abschn. IV-C, F3B-P3 (Kp = 3) beste Variante.
% Phasenreserve bei Totzeit tau: 90° - Kp*tau*180/pi  -> tau = 20 ms: 86,6°
REG.Kp = 3.0;

% Dämpfung der Pseudoinversen (Buss 2004). Fehler <= lambda^2/sigma_min^2
% = 0,25 % bei sigma_min = 0,2 (kleinster Wert innerhalb der Gelenkgrenzen)
REG.lambda = 0.01;
% Schalter Aufschaltung (1 = an, 0 = aus) für das Testprotokoll
REG.k_ff = 1;

% Sollorientierung = Kameraorientierung in der Grundpose bei ruhendem Hund
% (R_WB(0) = I wegen Einblenden der Störung) -> R_WC_des = R_B0 * R_0C(q0)
T_0C = z1_fk(SIM.q0, Z1, MONT.T_6C);
REG.R_WC_des = MONT.R_B0 * T_0C(1:3, 1:3);


% Add-on 2: Sollhöhe des Endeffektors = Höhe in der Grundpose bei ruhendem Hund
% (R_WB = I, z_B = 0). Relativ zum Boden, weil z_B relativ zur Standhöhe ist.
REG.q0    = SIM.q0;
REG.h_des = ee_hoehe(SIM.q0, eye(3), 0, Z1, MONT);

% Add-on 2: Rückführverstärkung der Höhe [1/s], Höhenfehler klingt mit T = 1/K_h ab.
% PLATZHALTER: wie K_p (EMMa, F3B-P3), später eigene Studie
REG.K_h = 3.0;
end