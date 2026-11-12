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
% Alle Stufen: TF9 ohne Add-on 2: Höhenfehler = Hub, RMS = A/sqrt(2)
% Add-on 1: Stufe 2, TF6: Restfehler nur noch aus PT1 und Rundung des Horizonts
% Add-on 2: Stufe 1: Höhe >= 98 % weniger, Orientierung unverändert (Vorrang)
GRENZE_RED   = 99.9;   % [%]
GRENZE_RED_H = 98;     % [%] Add-on 2 in Stufe 1: Höhe muss fast vollständig verschwinden
GRENZE_PT1   = 0.01;   % [°]
GRENZE_S2    = 0.10;   % zulässige relative Abweichung Totzeit-Test
GRENZE_BIAS  = 0.15;   % zulässige relative Abweichung Bias-Tests (Rauschen streut)

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

    % ---------------- Add-on 1: Prädiktion ----------------
    if E.tf.addon == "praediktion" && E.tf.stufe == 2 && E.tf.nr == 6
        % Sinus wird exakt vorhergesagt -> nur noch PT1 und Rundung von h bleiben
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

    % ---------------- Add-on 2: Höhe im Nullraum ----------------
    if E.tf.addon == "heave" && E.tf.stufe == 1
        % ideale Höhe, ideale Gelenke: Höhe fast vollständig weg,
        % Orientierung unverändert (Vorrang, J_w*N = 0)
        if E.rms_hund == 0
            ok_or = E.rms_fehler < 1e-3;                  % TF9: keine Drehung -> Fehler ~ 0
        else
            ok_or = E.reduktion >= GRENZE_RED;
        end
        if ~ok_or
            error('REGRESSION Add-on 2: Orientierung in "%s" verschlechtert.', E.tf.name);
        end
        if E.reduktion_hoehe < GRENZE_RED_H
            error('REGRESSION Add-on 2: "%s" Höhe nur %.2f %% reduziert (Soll >= %d %%).', ...
                  E.tf.name, E.reduktion_hoehe, GRENZE_RED_H);
        end
        fprintf('%s: Höhe %.3f mm (%.2f %% weniger), Orientierung ok\n', ...
                E.tf.name, E.rms_hoehe, E.reduktion_hoehe);
        n = n + 1;
        continue
    end

    if E.tf.addon ~= "keins", continue; end      % übrige Theorie-Tests gelten für das Grundgerüst

    % ---------------- Grundgerüst ----------------
    if E.tf.nr == 9
        % Ohne Höhenausgleich bleibt der Hub vollständig stehen.
        % Keine Drehung -> Arm bewegt sich nicht -> Höhenfehler = z_B, RMS = A/sqrt(2)
        S    = par_stoerung("heave", par_sim("vorn"));
        soll = 1000 * S.sin.A(4,1) / sqrt(2);                 % [mm]
        pruefe(E.rms_hoehe, soll, 0.02, "S" + E.tf.stufe + " TF9 Höhe ohne Ausgleich (RMS)", "mm");
        n = n + 1;
        continue
    end

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
function pruefe(ist, soll, grenze, text, einheit)
% Vergleicht Simulation mit Theorie, gibt beides aus, bricht bei zu großer Abweichung ab
if nargin < 5, einheit = "°"; end
rel = abs(ist - soll) / soll;
fprintf('%s: Simulation %.4f %s, Theorie %.4f %s, Abweichung %.1f %%\n', ...
        text, ist, einheit, soll, einheit, 100*rel);
if rel > grenze
    error('%s weicht um %.0f %% von der Theorie ab.', text, 100*rel);
end
end