function pruefe_regression(ERG, GEL)
% PRUEFE_REGRESSION  Automatische Prüfung nach jedem Testlauf.
% Geprüft wird nur, was gerechnet wurde (AUSWAHL in start_model.m).
% Stufe 1 (ideal): Die Mathematik muss exakt stimmen, sonst Abbruch.
%   - jeder Basis-Lauf mit Aufschaltung: Reduktion >= 99,9 %  (gemessen 99,98 %)
%   - TF4 Rückführung: Fehler bei t = T gleich e0*exp(-1)     (gemessen 0,0002°)
% Stufe 2: TF6 Totzeit: Restfehler = |1 - G(jw)| * Störung,
%          G(jw) = exp(-jw*tau_eff) / (1 + jw*T),  tau_eff = N*Ts + Ts/2
% Stufe 3: TF7 Gyro-Bias mit Rückführung: Endwert e = |b| / Kp
%          TF8 Gyro-Bias ohne Rückführung: Drift  e(t) = |b| * (t - tau)
GRENZE_RED  = 99.9;   % [%]
GRENZE_PT1  = 0.01;   % [°]
GRENZE_S2   = 0.10;   % zulässige relative Abweichung Totzeit-Test
GRENZE_BIAS = 0.15;   % zulässige relative Abweichung Bias-Tests (Rauschen streut)

SEN2 = par_sensor(par_sim("vorn"));                  % Standard-Totzeit, ohne Rauschen
SEN3 = par_sensor(par_sim("vorn"), rauschen = true); % mit Bias
b    = rad2deg(norm(SEN3.bias_g));                   % Betrag Gyro-Bias [°/s]

if ~any(arrayfun(@(E) E.tf.stufe == 1 && E.tf.gruppe == "basis", ERG))
    fprintf('Hinweis: Stufe 1 nicht gerechnet – Regressionstest der Mathematik entfällt.\n');
end

n = 0;
for k = 1:numel(ERG)
    E = ERG(k);
    if E.tf.gruppe ~= "basis", continue; end
        if E.tf.addon == "praediktion" && E.tf.stufe == 2 && E.tf.nr == 6
        % Add-on 1: Sinus wird exakt vorhergesagt -> nur noch PT1 und Rundung von h bleiben
        S     = par_stoerung("roll", par_sim("vorn"));
        A     = rad2deg(S.sin.A(1,1));
        w     = 2*pi * S.sin.f(1,1);
        PR    = par_praediktion(SEN2, GEL);
        rest  = SEN2.N*SEN2.Ts + SEN2.Ts/2 - PR.h*SEN2.Ts;    % Restverzögerung (negativ = Vorhalt)
        G     = exp(-1j*w*rest) / (1 + 1j*w*GEL.T);
        soll  = A * abs(1 - G) / sqrt(2);
        ist   = std(E.e_W(E.t >= 2, 1));   % Schwingungsanteil um x (Rollen) ab 2 s; der konstante
                                           % Versatz vom Umschalten bei t = 0,5 s (K_p = 0) zählt nicht
        pruefe(ist, soll, 0.25, "Stufe 2, TF6 mit Prädiktion (RMS)");
        n = n + 1;
        continue
    end
    if E.tf.addon ~= "keins", continue; end      % übrige Theorie-Tests gelten für das Grundgerüst

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

    elseif E.tf.stufe == 2 && E.tf.nr == 6
        S       = par_stoerung("roll", par_sim("vorn"));
        A       = rad2deg(S.sin.A(1,1));             % Amplitude Rollen [°]
        w       = 2*pi * S.sin.f(1,1);               % Kreisfrequenz [rad/s]
        tau_eff = SEN2.N * SEN2.Ts + SEN2.Ts/2;      % Totzeit + halbe Abtastung
        G       = exp(-1j*w*tau_eff) / (1 + 1j*w*GEL.T);
        soll    = A * abs(1 - G) / sqrt(2);
        pruefe(E.rms_fehler, soll, GRENZE_S2, "Stufe 2, TF6 Totzeit-Test (RMS)");
        n = n + 1;

    elseif E.tf.stufe == 3 && E.tf.nr == 7
        soll = b / E.tf.Kp;                          % Endwert |b|/Kp [°]
        ist  = mean(E.fehler(E.t >= 10));            % nach dem Einschwingen (>> 1/Kp)
        pruefe(ist, soll, GRENZE_BIAS, "Stufe 3, TF7 Gyro-Bias mit Rückführung (Endwert)");
        n = n + 1;

    elseif E.tf.stufe == 3 && E.tf.nr == 8
        soll = b * (E.t(end) - SEN3.tau);            % lineare Drift [°]
        ist  = E.fehler(end);
        pruefe(ist, soll, GRENZE_BIAS, "Stufe 3, TF8 Gyro-Bias ohne Rückführung (Drift)");
        n = n + 1;
    end
end
fprintf('Regressionstest bestanden: %d Läufe geprüft.\n', n);
end

% ---------------------------------------------------------------------------
function pruefe(ist, soll, grenze, text)
% Vergleicht Simulation mit Theorie, gibt beides aus, bricht bei zu großer Abweichung ab
rel = abs(ist - soll) / soll;
fprintf('%s: Simulation %.4f°, Theorie %.4f°, Abweichung %.1f %%\n', text, ist, soll, 100*rel);
if rel > grenze
    error('%s weicht um %.0f %% von der Theorie ab.', text, 100*rel);
end
end