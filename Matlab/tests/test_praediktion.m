function test_praediktion(A2W, SIM, PRAED)
% TEST_PRAEDIKTION  Prüft den AR-Prädiktor (Add-on 1) offline, ohne Simulink.
% 1) Reiner Sinus: muss exakt vorhergesagt werden (ein AR-Modell kann einen Sinus exakt darstellen).
% 2) Gelände-Drehraten mit Gyro-Rauschen (Stufe 3): Vorhersagefehler gegen den Fehler
%    ohne Vorhersage (alter Messwert). Das zeigt vorab, was das Add-on bringen kann.
fprintf('test_praediktion: AR(%d), Fenster %d Werte (%.2f s), Horizont %d Schritte (%.0f ms)\n', ...
    PRAED.p, PRAED.M, PRAED.M*SIM.Ts, PRAED.h, 1000*PRAED.h*SIM.Ts);

%% 1. Reiner Sinus 1 Hz
t      = (0:PRAED.M-1)' * SIM.Ts;
x      = sin(2*pi*t);
x_pred = ar_praediktion(x, PRAED.p, PRAED.h, PRAED.mu);
x_wahr = sin(2*pi*(t(end) + PRAED.h*SIM.Ts));
abw    = abs(x_pred - x_wahr);
fprintf('  Sinus 1 Hz: Fehler mit Vorhersage %.1e, ohne Vorhersage %.3f\n', abw, abs(x(end) - x_wahr));
assert(abw < 1e-5, 'test_praediktion: Sinus wird nicht exakt vorhergesagt.');

%% 2. Gelände-Drehraten aus start_model (fahren, 2 m/s) mit Gyro-Rauschen wie in Stufe 3
SEN3   = par_sensor(SIM, rauschen = true);
zufall = RandStream('mt19937ar', 'Seed', 7);
w_mess = A2W.w_B + SEN3.sigma_g * randn(zufall, size(A2W.w_B));   % N x 3 [rad/s]
N      = size(w_mess, 1);
k_alle = (round(1/SIM.Ts) + PRAED.M) : 5 : (N - PRAED.h);         % nach dem Einblenden, jeder 5. Schritt
e_ohne = zeros(numel(k_alle), 3);
e_mit  = e_ohne;
for n = 1:numel(k_alle)
    k = k_alle(n);
    for a = 1:3
        wahr        = A2W.w_B(k + PRAED.h, a);                % wahre Drehrate h Schritte später
        e_ohne(n,a) = wahr - w_mess(k, a);                    % ohne Vorhersage: alter Messwert
        e_mit(n,a)  = wahr - ar_praediktion(w_mess(k-PRAED.M+1:k, a), PRAED.p, PRAED.h, PRAED.mu);
    end
end
rms_ohne = rad2deg(sqrt(mean(e_ohne.^2)));
rms_mit  = rad2deg(sqrt(mean(e_mit.^2)));
achse    = ["Rollen" "Nicken" "Gieren"];
for a = 1:3
    fprintf('  %-6s: Drehratenfehler ohne Vorhersage %6.2f °/s, mit %6.2f °/s  (Faktor %.2f)\n', ...
        achse(a), rms_ohne(a), rms_mit(a), rms_mit(a)/rms_ohne(a));
end
if any(rms_mit >= rms_ohne)
    warning('test_praediktion: Vorhersage ist in mindestens einer Achse nicht besser als der alte Messwert.');
end
fprintf('test_praediktion: OK\n');
end