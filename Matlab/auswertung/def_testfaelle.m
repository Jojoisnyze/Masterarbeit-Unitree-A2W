function TF = def_testfaelle(stufen, v_studie, tau_studie)
% DEF_TESTFAELLE  Liste aller Simulationsläufe.
%   TF = def_testfaelle([1 2], true, true)
%   stufen      jede Stufe in der Liste rechnet alle Basis-Testfälle
%   v_studie    true: zusätzlich "mit Rückführung" bei v = 1...5 m/s, Posen vorn und seite
%   tau_studie  true: zusätzlich Stufe 2 mit Totzeit 2...40 ms bei v = 2 und 4 m/s
% Neuer Basis-Testfall: eine weitere Zeile B(end+1) = tf(...) anhängen.
arguments
    stufen     (1,:) double  = 1
    v_studie   (1,1) logical = false
    tau_studie (1,1) logical = false
end

%% Basis-Testfälle (Testprotokoll)
%            Nr Titel                          Szenario  Pose     k_ff Kp Start[°] v[m/s]
B    = tf(1, "ohne Stabilisierung",           "fahren", "vorn",  0,   0,  0,       2.0);
B(2) = tf(2, "nur IMU (Roncone)",             "fahren", "vorn",  1,   0,  0,       2.0);
B(3) = tf(3, "mit Rückführung",               "fahren", "vorn",  1,   3,  0,       2.0);
B(4) = tf(4, "Rückführung: Startfehler",      "aus",    "vorn",  1,   3,  5,       2.0);
B(5) = tf(5, "Blick zur Seite",               "fahren", "seite", 1,   3,  0,       2.0);
B(6) = tf(6, "Totzeit-Test: Sinus 1 Hz",      "roll",   "vorn",  1,   0,  0,       2.0);

%% Jede Stufe rechnet alle Basis-Testfälle
TF = B([]);                                     % leere Liste mit denselben Feldern
for s = stufen
    for k = 1:numel(B)
        t        = B(k);
        t.stufe  = s;
        t.gruppe = "basis";
        t.name   = "S" + s + " TF" + t.nr + " " + t.titel;
        TF(end+1) = t;                           %#ok<AGROW>
    end
end

%% Geschwindigkeitsstudie
if v_studie
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

%% Totzeitstudie (nur Stufe 2, nur dort gibt es eine Totzeit)
if tau_studie
    for v = [2 4]
        for tau = [0.002 0.004 0.010 0.020 0.040]
            t        = tf(0, "tau-Studie mit Rückführung", "fahren", "vorn", 1, 3, 0, v);
            t.stufe  = 2;
            t.gruppe = "tau_studie";
            t.tau    = tau;
            t.name   = "S2 tau = " + 1000*tau + " ms, v = " + v + " m/s";
            TF(end+1) = t;                       %#ok<AGROW>
        end
    end
end
end

% ======================= lokale Hilfsfunktion =============================
function s = tf(nr, titel, szenario, pose, k_ff, Kp, startfehler_grad, v)
% Ein Testfall mit allen Feldern (Standardwerte für stufe, gruppe, tau)
s.nr               = nr;
s.titel            = titel;
s.name             = titel;           % wird oben um Stufe und Nummer ergänzt
s.szenario         = szenario;
s.pose             = pose;
s.k_ff             = k_ff;
s.Kp               = Kp;
s.startfehler_grad = startfehler_grad;   % Kamera startet um x verdreht
s.v                = v;
s.tau              = NaN;                % NaN = Standard-Totzeit aus par_sensor
s.stufe            = 1;
s.gruppe           = "basis";
end