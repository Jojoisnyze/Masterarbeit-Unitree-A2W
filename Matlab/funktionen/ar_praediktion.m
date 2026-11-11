function [x_pred, alpha] = ar_praediktion(fenster, p, h, mu)
% AR_PRAEDIKTION  Vorhersage eines Signals um h Abtastschritte mit einem AR(p)-Modell.
% Quelle: Woolfrey et al. 2021, Gl. (7):  y(t) = sum_{i=1..p} alpha_i * y(t-i)
%   Koeffizienten alpha per Kleinster-Quadrate-Schätzung über das Fenster,
%   dann Gl. (7) h-mal hintereinander anwenden.
% Eingang:  fenster  M x 1  letzte M Messwerte, ältester zuerst, neuester zuletzt
%           p        Modellordnung (Anzahl Vorgänger)
%           h        Vorhersage-Horizont [Abtastschritte]
%           mu       Regularisierung (Ridge), hält das Gleichungssystem lösbar
% Ausgang:  x_pred   Vorhersage h Schritte nach dem neuesten Wert
%           alpha    p x 1 Gewichte, alpha(1) gehört zum jüngsten Vorgänger
y = fenster(:);
M = numel(y);

% 1) Regressionsmatrix: Zeile = die p Vorgänger eines Werts, Ziel = der Wert selbst
Phi = zeros(M-p, p);
for i = 1:p
    Phi(:, i) = y(p+1-i : M-i);          % Spalte i = Vorgänger im Abstand i
end
ziel  = y(p+1 : M);
alpha = (Phi.'*Phi + mu*eye(p)) \ (Phi.'*ziel);

% 2) h Schritte vorhersagen: jede Vorhersage wird wieder als Vorgänger verwendet
verlauf = y(M-p+1 : M);                  % die letzten p Werte, ältester zuerst
for j = 1:h
    neu     = alpha.' * flipud(verlauf); % alpha(1)*jüngster + alpha(2)*zweitjüngster ...
    verlauf = [verlauf(2:end); neu];
end
x_pred = verlauf(end);
end