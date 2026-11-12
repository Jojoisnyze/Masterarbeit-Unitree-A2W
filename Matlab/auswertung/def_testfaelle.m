function TF = def_testfaelle(AUSWAHL)
% DEF_TESTFAELLE  Liste aller Simulationsläufe, gesteuert über AUSWAHL (start_model.m).
%   AUSWAHL.stufen        z. B. [1 2 3]: jede Stufe rechnet alle (ausgewählten) Basis-Testfälle
%   AUSWAHL.testfaelle    [] = alle Basis-Testfälle, sonst Nummern, z. B. [3 6]
%   AUSWAHL.v_studie      true: TF3 bei v = 1...5 m/s, Posen vorn und seite, je Stufe
%   AUSWAHL.tau_studie    true: TF3 mit tau = 2...40 ms bei 2 und 4 m/s, je Stufe >= 2
%   AUSWAHL.addons        z. B. ["keins" "praediktion" "heave" "praediktion+heave"]
%   AUSWAHL.addon_stufen  in welchen Stufen die Add-ons laufen, z. B. [2 3]
% Neuer Basis-Testfall: eine weitere Zeile B(end+1) = tf(...) anhängen.
arguments
    AUSWAHL struct
end
stufen = AUSWAHL.stufen(:)';
addons = string(AUSWAHL.addons(:))';

%% Basis-Testfälle (Testprotokoll)
%            Nr Titel                          Szenario  Pose     k_ff Kp Start[°] v[m/s]
B    = tf(1, "ohne Stabilisierung",           "fahren", "vorn",  0,   0,  0,       2.0);
B(2) = tf(2, "nur IMU (Roncone)",             "fahren", "vorn",  1,   0,  0,       2.0);
B(3) = tf(3, "mit Rückführung",               "fahren", "vorn",  1,   3,  0,       2.0);
B(4) = tf(4, "Rückführung: Startfehler",      "aus",    "vorn",  1,   3,  5,       2.0);
B(5) = tf(5, "Blick zur Seite",               "fahren", "seite", 1,   3,  0,       2.0);
B(6) = tf(6, "Totzeit-Test: Sinus 1 Hz",      "roll",   "vorn",  1,   0,  0,       2.0);
B(7) = tf(7, "Gyro-Bias mit Rückführung",     "aus",    "vorn",  1,   3,  0,       2.0);
B(8) = tf(8, "Gyro-Bias ohne Rückführung",    "aus",    "vorn",  1,   0,  0,       2.0);
B(9) = tf(9, "Federn-Test: Hub 2 cm, 2 Hz",   "heave",  "vorn",  1,   3,  0,       2.0);
B(7).ab_stufe = 3;      % Bias gibt es erst ab Stufe 3
B(8).ab_stufe = 3;

% Nur ausgewählte Basis-Testfälle
if ~isempty(AUSWAHL.testfaelle)
    B = B(ismember([B.nr], AUSWAHL.testfaelle));
end

%% Jede Stufe rechnet alle (ausgewählten) Basis-Testfälle
TF = B([]);                                     % leere Liste mit denselben Feldern
for s = stufen
    for k = 1:numel(B)
        t = B(k);
        if s < t.ab_stufe, continue; end         % Testfall in dieser Stufe nicht sinnvoll
        t.stufe  = s;
        t.gruppe = "basis";
        t.name   = "S" + s + " TF" + t.nr + " " + t.titel;
        TF(end+1) = t;                           %#ok<AGROW>
    end
end

%% Geschwindigkeitsstudie (je Stufe)
if AUSWAHL.v_studie
    for s = stufen
        for pose = ["vorn" "seite"]
            for v = 1:5
                t        = tf(0, "v-Studie mit Rückführung", "fahren", pose, 1, 3, 0, v);
                t.stufe  = s;
                t.gruppe = "v_studie";
                t.name   = "S" + s + " v = " + v + " m/s, " + pose;
                TF(end+1) = t;                   %#ok<AGROW>
            end
        end
    end
end

%% Totzeitstudie (je Stufe >= 2, nur dort gibt es eine Totzeit)
if AUSWAHL.tau_studie
    for s = stufen(stufen >= 2)
        for v = [2 4]
            for tau = [0.002 0.004 0.010 0.020 0.040]
                t        = tf(0, "tau-Studie mit Rückführung", "fahren", "vorn", 1, 3, 0, v);
                t.stufe  = s;
                t.gruppe = "tau_studie";
                t.tau    = tau;
                t.name   = "S" + s + " tau = " + 1000*tau + " ms, v = " + v + " m/s";
                TF(end+1) = t;                   %#ok<AGROW>
            end
        end
    end
end

%% Jeden Lauf je Add-on wiederholen
% "keins" läuft immer überall. Add-ons nur in AUSWAHL.addon_stufen und nur,
% wenn überhaupt stabilisiert wird (k_ff = 1).
% basisname = Name ohne Add-on (für den Referenzvergleich), name = mit Zusatz.
TF0 = TF;
TF  = TF0([]);
for a = addons
    for k = 1:numel(TF0)
        t = TF0(k);
        if a ~= "keins"
            if ~ismember(t.stufe, AUSWAHL.addon_stufen), continue; end
            if t.k_ff == 0, continue; end
        end
        t.addon     = a;
        t.basisname = t.name;
        if a ~= "keins"
            t.name = t.name + " + " + a;
        end
        TF(end+1) = t;                           %#ok<AGROW>
    end
end
end

% ======================= lokale Hilfsfunktion =============================
function s = tf(nr, titel, szenario, pose, k_ff, Kp, startfehler_grad, v)
% Ein Testfall mit allen Feldern (Standardwerte für tau, ab_stufe, stufe, gruppe, addon)
s.nr               = nr;
s.titel            = titel;
s.name             = titel;           % wird oben um Stufe, Nummer und Add-on ergänzt
s.basisname        = titel;           % Name ohne Add-on (Referenzvergleich)
s.szenario         = szenario;
s.pose             = pose;
s.k_ff             = k_ff;
s.Kp               = Kp;
s.startfehler_grad = startfehler_grad;   % Kamera startet um x verdreht
s.v                = v;
s.tau              = NaN;                % NaN = Standard-Totzeit aus par_sensor
s.ab_stufe         = 1;                  % kleinste Stufe, in der der Testfall läuft
s.addon            = "keins";            % Add-on(s) im Regler (addon_einstellen.m)
s.stufe            = 1;
s.gruppe           = "basis";
end