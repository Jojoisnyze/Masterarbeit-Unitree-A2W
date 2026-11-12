function KAL = par_kalman(SIM, SEN, HOE, opt)
% PAR_KALMAN  Kalman-Filter für den Hub (Add-on 2, Variante "Kalman").
% Zustand x = [z_B; dz_B; b_a]: Hub, Hubgeschwindigkeit, Bias des Beschleunigungsmessers
%
% Quelle: ESKF nach Meng et al. 2026, Abschnitt III, auf die Vertikale reduziert:
%   Zustand Gl. (28), Vorhersage Gl. (33)/(36), Korrektur Gl. (39)-(41)
%   Abweichung: Beschleunigung als Eingang (Standard-ESKF, Sola 2017) statt als
%   Messung (Meng Gl. 43). Im linearen 1-D-Fall = gewöhnlicher Kalman-Filter.
% Totzeiten: Die Beinhöhe ist d = N_bein - N_imu Schritte älter als die Beschleunigung.
%   Filter läuft zum Messzeitpunkt der Beinhöhe, Ausgabe wird mit den gespeicherten
%   Beschleunigungen auf "jetzt" vorgerechnet (Neuberechnung, vgl. Larsen et al. 1998).
% Modell = Rumpfkinematik (Punktmasse in z), KEIN Beinmodell des A2W nötig.
% !!! KAL im MATLAB-Function-Block NICHT tunable (d bestimmt die Puffergröße) !!!
% !!! sigma_u, sigma_bw sind EINSTELLWERTE; R aus den Platzhaltern der Beinhöhe !!!
arguments
    SIM struct
    SEN struct
    HOE struct
    opt.verzoegert (1,1) logical = true      % Stufe >= 2: Totzeiten bekannt und berücksichtigt
end

Ts     = SIM.Ts;
KAL.Ts = Ts;
KAL.g  = SIM.g;

% Totzeiten in Abtastschritten (dem Entwickler bekannt, am A2W zu messen)
if opt.verzoegert
    KAL.N_imu  = SEN.N;
    KAL.N_bein = HOE.N_bein;
else
    KAL.N_imu  = 0;
    KAL.N_bein = 0;
end
KAL.d = max(0, KAL.N_bein - KAL.N_imu);    % so viel älter ist die Beinhöhe

% Modell: konstante Beschleunigung je Abtastschritt, Bias wird abgezogen
KAL.F = [1 Ts -Ts^2/2;  0 1 -Ts;  0 0 1];
KAL.B = [Ts^2/2; Ts; 0];
KAL.H = [1 0 0;  0 1 0];                    % Beinhöhe misst z und dz

% Rauschen
HOE3     = par_hoehe(SIM, rauschen = true);  % Messrauschen der Beinhöhe (Platzhalter Stufe 3)
KAL.R    = diag(HOE3.sigma.^2);
sigma_u  = 0.1;      % Unsicherheit Beschleunigung je Schritt [m/s^2]  (EINSTELLWERT)
sigma_bw = 1e-3;     % Drift des Bias [m/s^2/sqrt(s)]                    (EINSTELLWERT)
KAL.Q    = KAL.B*KAL.B.' * sigma_u^2 + diag([0, 0, sigma_bw^2 * Ts]);
KAL.P0   = diag([1e-6, 1e-4, 0.05^2]);       % Anfangsunsicherheit: Hund steht, Bias unbekannt
end