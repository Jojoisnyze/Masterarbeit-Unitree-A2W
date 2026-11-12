function [x, P, ubuf, hub_m] = kalman_hoehe(x, P, ubuf, a_z, y, KAL)
% KALMAN_HOEHE  Ein Abtastschritt des Kalman-Filters für den Hub (Add-on 2).
% Eingang:  x, P   Zustand [z; dz; b_a] und Kovarianz zum Messzeitpunkt der Beinhöhe
%           ubuf   die letzten d+2 Vertikalbeschleunigungen [m/s^2] (neueste rechts)
%           a_z    neue Vertikalbeschleunigung aus der IMU [m/s^2], Schwerkraft schon abgezogen
%           y      Beinhöhe von Unitree [z; dz], d Schritte älter als a_z
% Ausgang:  hub_m  Schätzung [z; dz] für JETZT (an den Regler)
%
% Quelle: Meng et al. 2026, Gl. (33) Vorhersage, (36) Kovarianz, (39)-(41) Korrektur,
%         auf die Vertikale reduziert, Beschleunigung als Eingang (Sola 2017).
%         Verzögerte Messung: Filter zum Messzeitpunkt, danach Neuberechnung mit den
%         gespeicherten Beschleunigungen (vgl. Larsen et al. 1998).
% Integration mit der Trapezregel: Mittelwert der Beschleunigung am Anfang und Ende des Schritts.

ubuf = [ubuf(2:end), a_z];

% 1) Vorhersage bis zum Messzeitpunkt der Beinhöhe (Meng Gl. 33, 36)
x = KAL.F*x + KAL.B*(ubuf(1) + ubuf(2))/2;
P = KAL.F*P*KAL.F.' + KAL.Q;

% 2) Korrektur mit der Beinhöhe (Meng Gl. 39-41)
S = KAL.H*P*KAL.H.' + KAL.R;
K = P*KAL.H.' / S;
x = x + K*(y(:) - KAL.H*x);
P = (eye(3) - K*KAL.H)*P;

% 3) Neuberechnung bis zur neuesten Beschleunigung (d Schritte)
xa = x;
for j = 2:numel(ubuf)-1
    xa = KAL.F*xa + KAL.B*(ubuf(j) + ubuf(j+1))/2;
end

% 4) Vorhalt über die IMU-Totzeit: neueste Beschleunigung konstant halten
for j = 1:KAL.N_imu
    xa = KAL.F*xa + KAL.B*ubuf(end);
end
hub_m = xa(1:2);
end