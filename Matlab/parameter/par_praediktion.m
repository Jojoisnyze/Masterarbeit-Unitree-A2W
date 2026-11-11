function PRAED = par_praediktion(SEN, GEL)
% PAR_PRAEDIKTION  Parameter von Add-on 1: prädiktive Störgrößenaufschaltung.
% Quelle: Woolfrey et al. 2021, Abschnitt III, Gl. (7): AR(p)-Modell
%   y(t) = sum_{i=1..p} alpha_i * y(t-i),
%   Koeffizienten per linearer Kleinster-Quadrate-Schätzung über ein gleitendes
%   Fenster, in jeder Regelperiode neu.
% Übertragung: vorhergesagt wird die gemessene Drehrate w_B je Achse, und zwar
%   um die effektive Verzögerung tau_eff = tau + Ts/2 + T_Gelenk (Totzeitstudie).
PRAED.p  = 20;      % Modellordnung (Woolfrey: AR(20), Tab. I)              (Startwert)
PRAED.M  = 250;     % Fensterlänge [Abtastwerte], 250 * 2 ms = 0,5 s         (Startwert)
PRAED.mu = 1e-6;    % Regularisierung der Kleinste-Quadrate-Lösung (Ridge)   (Startwert)

% Horizont: so weit muss vorhergesagt werden, um die Verzögerung auszugleichen
PRAED.tau_eff = SEN.tau + SEN.Ts/2 + GEL.T;      % [s]
PRAED.h       = round(PRAED.tau_eff / SEN.Ts);   % [Abtastschritte]
end