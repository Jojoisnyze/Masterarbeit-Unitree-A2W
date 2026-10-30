function pruefe_regression(ERG, SEN, GEL)
% PRUEFE_REGRESSION  Automatische Prüfung nach jedem Testlauf.
% Stufe 1 (ideal): Die Mathematik muss exakt stimmen, sonst Abbruch.
%   - jeder Basis-Lauf mit Aufschaltung: Reduktion >= 99,9 %  (gemessen 99,98 %)
%   - Rückführungstest: Fehler bei t = T gleich e0*exp(-1)    (gemessen 0,0002°)
% Stufe 2 (Totzeit-Test): Sinus 1 Hz, nur Aufschaltung. Der Arm dreht um
%   tau_eff verspätet und über ein PT1 gegen -> Restfehler = |1 - G(jw)| * Störung,
%   G(jw) = exp(-jw*tau_eff) / (1 + jw*T),  tau_eff = N*Ts + Ts/2 (Halteglied).
GRENZE_RED = 99.9;    % [%]
GRENZE_PT1 = 0.01;    % [°]
GRENZE_S2  = 0.10;    % zulässige relative Abweichung von der Theorie

n = 0;
for k = 1:numel(ERG)
    E = ERG(k);
    if E.tf.gruppe ~= "basis", continue; end

    if E.tf.stufe == 1 && E.tf.k_ff == 1 && E.tf.szenario ~= "aus"
        if E.reduktion < GRENZE_RED
            error(['REGRESSION Stufe 1: "%s" erreicht nur %.3f %% (Soll >= %.1f %%).\n' ...
                'Beim letzten Umbau ist ein Fehler hineingekommen.'], ...
                E.tf.name, E.reduktion, GRENZE_RED);
        end
        n = n + 1;

    elseif E.tf.stufe == 1 && E.tf.startfehler_grad ~= 0 && E.tf.Kp > 0
        T   = 1 / E.tf.Kp;
        abw = abs(interp1(E.t, E.fehler, T) - E.tf.startfehler_grad * exp(-1));
        if abw > GRENZE_PT1
            error('REGRESSION Stufe 1: Rückführung weicht bei t = T um %.4f° vom PT1 ab.', abw);
        end
        n = n + 1;

    elseif E.tf.stufe == 2 && E.tf.szenario == "roll"
        S       = par_stoerung("roll", par_sim("vorn"));
        A       = rad2deg(S.sin.A(1,1));             % Amplitude Rollen [°]
        w       = 2*pi * S.sin.f(1,1);               % Kreisfrequenz [rad/s]
        tau_eff = SEN.N * SEN.Ts + SEN.Ts/2;         % Totzeit + halbe Abtastung
        G       = exp(-1j*w*tau_eff) / (1 + 1j*w*GEL.T);
        rms_th  = A * abs(1 - G) / sqrt(2);
        rel     = abs(E.rms_fehler - rms_th) / rms_th;
        fprintf('Totzeit-Test Stufe 2: RMS %.3f° (Theorie %.3f°, Abweichung %.1f %%)\n', ...
            E.rms_fehler, rms_th, 100*rel);
        if rel > GRENZE_S2
            error('Totzeit-Test Stufe 2 weicht um %.0f %% von der Theorie ab.', 100*rel);
        end
        n = n + 1;
    end
end
fprintf('Regressionstest bestanden: %d Läufe geprüft.\n', n);
end