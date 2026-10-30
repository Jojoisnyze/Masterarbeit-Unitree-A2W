function STOER = par_stoerung(szenario, SIM, opt)
% PAR_STOERUNG  Parameter der Rumpfbewegung des A2W (Störung).
%
% Aufruf:  STOER = par_stoerung("fahren", SIM)                  -> Standardwerte
%          STOER = par_stoerung("fahren", SIM, v=3, klasse="D", steigung_grad=15)
%
% Kanäle k: 1 = Rollen phi [rad], 2 = Nicken theta [rad],
%           3 = Gieren psi [rad], 4 = Heave z [m]
% Jeder Kanal = Summe von Sinus (STOER.sin) + Gaußpulse (STOER.stoss).
% Ein konstanter Anteil (Hang) ist ein Sinus mit f = 0 und Phase pi/2.
%
% Szenarien (Testprotokoll, Modellgrundlagen):
%   'aus'                 keine Störung
%   'roll','nick','gier'  Einzelachse, Testsinus 3° bei 1 Hz
%   'kombi'               alle drei Testsinus gleichzeitig
%   'fahren'              Radmodus: Gelände ISO 8608 + Schlaglöcher + Hang
%   'laufen'              Laufmodus (Trab): Gang + Rauschen + Fußaufsetzer
%
% Vorzeichen: x vorne, y links, z oben. phi > 0: linke Seite hoch,
%             theta > 0: Nase runter, psi > 0: nach links drehen.
%
% !!! ALLE ZAHLEN SIND BEGRÜNDETE SCHÄTZUNGEN (PLATZHALTER) bis A2W-Daten vorliegen !!!
arguments
    szenario (1,1) string
    SIM      struct
    opt.v             (1,1) double = 2.0   % Fahrgeschwindigkeit [m/s]
    opt.klasse        (1,1) string = "E"   % ISO-8608-Klasse "A" ... "H"
    opt.steigung_grad (1,1) double = 0     % Hang in Fahrtrichtung, + = bergauf [°]
    opt.quer_grad     (1,1) double = 0     % Hang quer, + = linke Seite höher [°]
end

STOER.szenario = szenario;
STOER.opt      = opt;                  % Einstellungen merken (für Plots/Auswertung)
STOER.T_ein    = 1.0;                  % weiches Einblenden [s]
zufall = RandStream('mt19937ar', 'Seed', 42);   % reproduzierbar

% leere Bausteine, werden je Szenario gefüllt
STOER.sin.A       = zeros(4, 0);  STOER.sin.f = zeros(4, 0);  STOER.sin.phase = zeros(4, 0);
STOER.stoss.t0    = zeros(1, 0);  STOER.stoss.A = zeros(4, 0);
STOER.stoss.sigma = zeros(1, 0);

switch STOER.szenario
    case "aus"
        % keine Anteile

    case {"roll", "nick", "gier", "kombi"}
        % ---- Testsinus (Testprotokoll Schritt 1 und 2) ----
        A_test = deg2rad(3);   f_test = 1.0;          % -> max. 18,8 °/s
        m = double(STOER.szenario == ["roll"; "nick"; "gier"]);
        if STOER.szenario == "kombi", m = [1; 1; 1]; end
        STOER = sinus_hinzu(STOER, A_test*[m; 0], f_test*ones(4,1), [0; pi/2; pi/4; 0]);

    case "fahren"
        v = opt.v;
        % ---- PLATZHALTER Fahrzeug -------------------------------------------
        L     = 0.60;       % Radstand vorne-hinten [m]   (geschätzt, messen!)
        B     = 0.38;       % Spurweite links-rechts [m]  (geschätzt, messen!)
        d_rad = 0.20;       % Raddurchmesser [m]          (geschätzt, messen!)
        fc    = 0.5;        % bis hierher gleichen die Beine Neigungen aus [Hz]
        % ---- Gelände nach ISO 8608 ------------------------------------------
        % G_d(n0) je Klasse: A = 16e-6, jede weitere Klasse x4 (D = 1024e-6, E = 4096e-6)
        klassen = ["A" "B" "C" "D" "E" "F" "G" "H"];
        G0    = 16e-6 * 4^(find(klassen == opt.klasse) - 1);   % [m^3]
        n0    = 0.1;        % Bezugs-Wegfrequenz [1/m]
        n_ber = [0.1 5];    % Wegfrequenzen [1/m] -> Wellenlängen 10 m ... 0,2 m
        N     = 60;         % Anzahl Sinus je Spur
        % ---- PLATZHALTER Schlaglöcher, Wurzeln --------------------------------
        s_loch = 8.0;% mittlerer Abstand [s]
        T_loch = s_loch / v;
        h_loch = [0.03 0.08];  % Tiefe [m]
        l_loch = 0.20;         % Länge [m]
        % ---- PLATZHALTER Gieren (Lenkkorrekturen) ------------------------------
        rms_gier = deg2rad(0.3);  band_gier = [0.1 2];

        % 1) Bodenprofil je Spur:  h(x) = sum A_i sin(2 pi n_i x + phi_i)
        %    ISO 8608: G_d(n) = G0 (n/n0)^-2,   A_i = sqrt(2 G_d(n_i) dn_i)
        nk = logspace(log10(n_ber(1)), log10(n_ber(2)), N+1);  % Bandgrenzen
        n  = sqrt(nk(1:end-1) .* nk(2:end));                   % Bandmitten
        dn = diff(nk);
        A  = sqrt(2 * G0 * (n/n0).^(-2) .* dn);                % Amplituden [m]
        w  = 2*pi*v*n;                                         % f = v*n -> Kreisfrequenz

        % 2) Komplexe Zeiger c = A e^{j phi}, linke und rechte Spur unabhängig
        cL = A .* exp(1j*2*pi*rand(zufall, 1, N));
        cR = A .* exp(1j*2*pi*rand(zufall, 1, N));
        d  = exp(-1j*w*L/v);                    % Hinterrad: um L/v verzögert
        wc = 2*pi*fc;
        H  = (1j*w/wc) ./ (1 + 1j*w/wc);        % Hochpass: Ausgleich durch die Beine

        % 3) Rumpflage aus den Radhöhen (vorne + hinten gemittelt)
        Z_roll  =  H .* (cL - cR) .* (1 + d)/2 / B;     % phi   = (h_L - h_R)/B
        Z_nick  = -H .* (cL + cR)/2 .* (1 - d) / L;     % theta = -(h_v - h_h)/L
        Z_heave =  H .* (cL + cR)/2 .* (1 + d)/2;       % z     = mittlere Radhöhe
        Z = [Z_roll; Z_nick; zeros(1, N); Z_heave];
        STOER = sinus_hinzu(STOER, abs(Z), repmat(v*n, 4, 1), angle(Z));

        % 4) Gieren: langsame Lenkkorrekturen
        STOER = rauschen_hinzu(STOER, [0; 0; rms_gier; 0], band_gier, 20, zufall);

        % 5) Schlaglöcher: erst Vorderrad, nach L/v das Hinterrad
        t_loch = STOER.T_ein + cumsum(T_loch * (0.5 + rand(zufall, 1, 100)));
        t_loch = t_loch(t_loch < SIM.Tend - 0.5);
        sigma  = (l_loch + d_rad) / (4*v);       % Pulsbreite aus Überfahrzeit
        for t0 = t_loch
            h = h_loch(1) + rand(zufall) * diff(h_loch);
            s = sign(rand(zufall) - 0.5);        % +1: linkes Rad, -1: rechtes Rad
            A_v = [-s*h/(2*B);  h/(2*L); 0; -h/4];   % Vorderrad: Nase runter
            A_h = [-s*h/(2*B); -h/(2*L); 0; -h/4];   % Hinterrad: Heck runter
            STOER = stoss_hinzu(STOER, [t0, t0 + L/v], [A_v, A_h], [sigma, sigma]);
        end

        % 6) Hang: konstante Rumpfneigung (Annahme: A2W nivelliert NICHT -> worst case)
        %    bergauf = Nase hoch = theta < 0;  linke Seite höher = phi > 0
        %    Konstante = Sinus mit f = 0 und Phase pi/2  (sin(pi/2) = 1, Ableitung 0)
        hang = [deg2rad(opt.quer_grad); -deg2rad(opt.steigung_grad); 0; 0];
        STOER = sinus_hinzu(STOER, hang, zeros(4,1), pi/2*ones(4,1));

    case "laufen"
        % ---- PLATZHALTER Trab -------------------------------------------------
        % Orientierung: HyQ ±4,8° Nicken / ±2,4° Rollen (eben), Bazeille: < 10 Hz
        f_s  = 2.0;                                            % Schrittfrequenz [Hz]
        A_g  = [deg2rad(2); deg2rad(3); deg2rad(1.5); 0.015];  % Gangschwingung
        f_g  = [f_s/2; f_s; f_s/2; f_s];
        STOER = sinus_hinzu(STOER, A_g, f_g, [0; pi/2; pi/4; 0]);

        rms_r = [deg2rad(0.5); deg2rad(0.5); deg2rad(0.3); 0.003];
        STOER = rauschen_hinzu(STOER, rms_r, [3 10], 20, zufall);

        % Fußaufsetzer: ein Puls pro Schritt, +-20 ms Jitter
        t0  = STOER.T_ein : 1/f_s : SIM.Tend - 0.1;
        t0  = t0 + 0.02 * (2*rand(zufall, size(t0)) - 1);
        A_f = [deg2rad(1); deg2rad(1.5); deg2rad(0.5); 0.005];
        M   = numel(t0);
        A   = A_f .* (0.5 + 0.5*rand(zufall, 4, M)) .* sign(rand(zufall, 4, M) - 0.5);
        STOER = stoss_hinzu(STOER, t0, A, 0.015 * ones(1, M));

        % Hang auch beim Laufen
        hang = [deg2rad(opt.quer_grad); -deg2rad(opt.steigung_grad); 0; 0];
        STOER = sinus_hinzu(STOER, hang, zeros(4,1), pi/2*ones(4,1));

    otherwise
        error('Unbekanntes Szenario: %s', STOER.szenario);
end
end

% ======================= lokale Hilfsfunktionen ===========================
function S = sinus_hinzu(S, A, f, ph)
% hängt Sinus-Anteile an (A, f, ph jeweils 4xK)
S.sin.A     = [S.sin.A, A];
S.sin.f     = [S.sin.f, f];
S.sin.phase = [S.sin.phase, ph];
end

function S = rauschen_hinzu(S, rms_k, band, N, zufall)
% Rauschen als Summe von N Sinus mit zufälliger Frequenz im Band und Phase.
% N Sinus mit Amplitude a haben RMS = a*sqrt(N/2)  ->  a = RMS*sqrt(2/N)
f  = band(1) + rand(zufall, 4, N) * diff(band);
ph = 2*pi * rand(zufall, 4, N);
A  = rms_k(:) * sqrt(2/N) .* ones(4, N);
S  = sinus_hinzu(S, A, f, ph);
end

function S = stoss_hinzu(S, t0, A, sigma)
% hängt Gaußpulse an (t0 1xM, A 4xM, sigma 1xM)
S.stoss.t0    = [S.stoss.t0, t0];
S.stoss.A     = [S.stoss.A, A];
S.stoss.sigma = [S.stoss.sigma, sigma];
end