function plot_testfaelle(ERG, Z1, ORDNER, details_anzeigen)
% PLOT_TESTFAELLE  Grafiken zu allen Läufen.
% Grundsatz: ein Signal pro Kachel, eigene y-Skala, Titel = Frage + Kennzahl.
%   S<s>_TF<k>_...  Details je Basis-Testfall und Stufe -> nur gespeichert
%   90_S<s>_...     Kamerafehler aller Basis-Testfälle einer Stufe
%   91_...          Kennwerte, Stufen nebeneinander
%   92_S<s>_...     Rückführungstest (PT1) je Stufe
%   94_...          Geschwindigkeitsstudie (falls gerechnet)
if nargin < 4, details_anzeigen = false; end

ist_basis = arrayfun(@(E) E.tf.gruppe == "basis", ERG);
basis     = ERG(ist_basis);
studie    = ERG(~ist_basis);
stufen    = unique(arrayfun(@(E) E.tf.stufe, basis));
stufen    = stufen(:)';                          % Zeilenvektor für die for-Schleife

%% 1. Details je Basis-Testfall
for k = 1:numel(basis)
    E  = basis(k);
    p  = "S" + E.tf.stufe + "_TF" + E.tf.nr;
    f1 = fig_fehler(E, details_anzeigen);
    f2 = fig_gelenke(E, Z1, details_anzeigen);
    f3 = fig_geschw(E, Z1, details_anzeigen);
    speichere_figur(f1, ORDNER, p + "_1_Kamerafehler");
    speichere_figur(f2, ORDNER, p + "_2_Gelenkwinkel");
    speichere_figur(f3, ORDNER, p + "_3_Gelenkgeschw");
    if ~details_anzeigen, close([f1 f2 f3]); end
end

%% 2. Vergleiche je Stufe und über die Stufen
for s = stufen
    B = basis(arrayfun(@(E) E.tf.stufe == s, basis));
    speichere_figur(fig_vergleich(B, s), ORDNER, "90_S" + s + "_Vergleich_Kamerafehler");
    k = find(arrayfun(@(E) E.tf.startfehler_grad ~= 0 && E.tf.Kp > 0, B), 1);
    if ~isempty(k)
        speichere_figur(fig_rueckfuehrung(B(k)), ORDNER, "92_S" + s + "_Rueckfuehrung_PT1");
    end
end
speichere_figur(fig_kennwerte(basis, Z1), ORDNER, "91_Vergleich_Kennwerte");

%% 3. Studien
ist_v   = arrayfun(@(E) E.tf.gruppe == "v_studie",   ERG);
ist_tau = arrayfun(@(E) E.tf.gruppe == "tau_studie", ERG);
if any(ist_v)
    speichere_figur(fig_geschwindigkeit(ERG(ist_v), Z1), ORDNER, "94_Geschwindigkeitsstudie");
end
if any(ist_tau)
    speichere_figur(fig_totzeit(ERG(ist_tau)), ORDNER, "95_Totzeitstudie");
end

%% 4. Tabelle und Daten
T = kennwert_tabelle(ERG);
disp(T);
writetable(T, fullfile(ORDNER, "93_Kennwerte.csv"), 'Delimiter', ';');
save(fullfile(ORDNER, "ERG.mat"), 'ERG');
fprintf('Grafiken, Tabelle und Daten gespeichert in:\n  %s\n', ORDNER);
end

% =============================== Figures ==================================
function f = fig_fehler(E, vis)
c  = farben();
f  = neue_figur(E.tf.name + " – Kamerafehler", vis, 5);
tl = tiledlayout(f, 5, 1, 'TileSpacing', 'compact');
title(tl, E.tf.name, 'FontWeight', 'bold');
subtitle(tl, beschreibung(E));
kachel(nexttile(tl), E.t, E.hund, '[°]', ...
    sprintf('Wie stark ist der HUND gegenüber dem Start verdreht? (= Kamerafehler ohne Stabilisierung)   max %.2f°', ...
    E.max_hund), c.grau);
kachel(nexttile(tl), E.t, E.fehler, '[°]', ...
    sprintf('Wie stark ist die KAMERA verdreht?   RMS %.3g°, max %.3g°%s', ...
    E.rms_fehler, E.max_fehler, red_text(E)), c.blau);
achse = ["x_W (vorne, Rollen)", "y_W (links, Nicken)", "z_W (oben, Gieren)"];
for i = 1:3
    ax = kachel(nexttile(tl), E.t, E.e_W(:,i), '[°]', ...
        sprintf('Anteil des Kamerafehlers um %s   max %.3g°', achse(i), max(abs(E.e_W(:,i)))), c.blau);
end
xlabel(ax, 't [s]');
end

function f = fig_gelenke(E, Z1, vis)
c     = farben();
namen = gelenk_namen();
f  = neue_figur(E.tf.name + " – Gelenkwinkel", vis, 6);
tl = tiledlayout(f, 6, 1, 'TileSpacing', 'compact');
title(tl, E.tf.name + ": Wie weit bewegt sich jedes Gelenk aus der Grundpose?", 'FontWeight', 'bold');
subtitle(tl, beschreibung(E));
q0 = E.q(1, :)';
for i = 1:6
    dq_grad = rad2deg(E.q(:,i) - q0(i));
    rest    = rad2deg(min(q0(i) - Z1.q_min(i), Z1.q_max(i) - q0(i)));
    ax = kachel(nexttile(tl), E.t, dq_grad, '\Delta q [°]', ...
        sprintf('%s:  max. Auslenkung %.2f°   (Abstand Grundpose bis Anschlag: %.0f°)', ...
        namen(i), max(abs(dq_grad)), rest), c.gruen);
end
xlabel(ax, 't [s]');
end

function f = fig_geschw(E, Z1, vis)
c     = farben();
namen = gelenk_namen();
f  = neue_figur(E.tf.name + " – Gelenkgeschwindigkeiten", vis, 6);
tl = tiledlayout(f, 6, 1, 'TileSpacing', 'compact');
title(tl, E.tf.name + ": Wie schnell soll sich jedes Gelenk drehen? (Befehl des Reglers)", 'FontWeight', 'bold');
subtitle(tl, beschreibung(E));
for i = 1:6
    w      = rad2deg(E.dq(:,i));
    grenze = rad2deg(Z1.dq_max(i));
    ax = kachel(nexttile(tl), E.t, w, '[°/s]', ...
        sprintf('%s:  max. %.0f °/s  = %.0f %% der Z1-Grenze (%.0f °/s)', ...
        namen(i), max(abs(w)), 100*max(abs(w))/grenze, grenze), c.orange);
    if max(abs(w)) > 0.5*grenze                     % Grenze nur zeigen, wenn relevant
        yline(ax, [-grenze grenze], 'r--');
    end
end
xlabel(ax, 't [s]');
end

function f = fig_vergleich(ERG, s)
c  = farben();
n  = numel(ERG);
f  = neue_figur("Stufe " + s + " – Vergleich Kamerafehler", true, n);
tl = tiledlayout(f, n, 1, 'TileSpacing', 'compact');
title(tl, "Stufe " + s + ": Kamerafehler je Testfall  –  ACHTUNG: jede Kachel hat ihre eigene y-Skala", ...
      'FontWeight', 'bold');
for k = 1:n
    E  = ERG(k);
    ax = kachel(nexttile(tl), E.t, E.fehler, '[°]', ...
        sprintf('%s:   RMS %.3g°, max %.3g°%s', E.tf.name, E.rms_fehler, E.max_fehler, red_text(E)), c.blau);
end
xlabel(ax, 't [s]');
end

function f = fig_kennwerte(ERG, Z1)
c      = farben();
nrs    = unique(arrayfun(@(E) E.tf.nr, ERG));     nrs    = nrs(:);
stufen = unique(arrayfun(@(E) E.tf.stufe, ERG));  stufen = stufen(:)';
RED = nan(numel(nrs), numel(stufen));  RMS = RED;  DQ = RED;
namen = strings(numel(nrs), 1);
for k = 1:numel(ERG)
    E = ERG(k);
    i = find(nrs == E.tf.nr);
    j = find(stufen == E.tf.stufe);
    RED(i,j) = E.reduktion;
    RMS(i,j) = E.rms_fehler;
    DQ(i,j)  = E.max_dq;
    namen(i) = "TF" + E.tf.nr + " " + E.tf.titel;
end
x   = reordercats(categorical(namen), namen);
leg = "Stufe " + stufen;

f  = neue_figur("Vergleich Kennwerte", true, 3);
tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact');
title(tl, 'Kennwerte aller Testfälle: Stufen nebeneinander', 'FontWeight', 'bold');

ax = nexttile(tl);  balken(ax, x, RED, c);
ylabel(ax, '[%]');  legend(ax, leg, 'Location', 'southeast');
title(ax, 'Um wie viel Prozent wird der Kamerafehler verringert? (RMS, Maß wie bei Roncone)', 'FontWeight', 'normal');

ax = nexttile(tl);  balken(ax, x, RMS, c);
set(ax, 'YScale', 'log');  ylabel(ax, '[°] (log)');
title(ax, 'Wie groß bleibt der Kamerafehler? (RMS, logarithmische Achse)', 'FontWeight', 'normal');

ax = nexttile(tl);  balken(ax, x, DQ, c);  hold(ax, 'on');
yline(ax, rad2deg(max(Z1.dq_max)), 'r--', 'Z1-Grenze');
ylabel(ax, '[°/s]');
title(ax, 'Wie schnell soll sich das schnellste Gelenk drehen? (Befehl)', 'FontWeight', 'normal');
end

function f = fig_rueckfuehrung(E)
c    = farben();
T    = 1 / E.tf.Kp;
theo = E.tf.startfehler_grad * exp(-E.t / T);
f  = neue_figur("Stufe " + E.tf.stufe + " – Rückführung PT1", true, 3);
tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact');
title(tl, sprintf('Stufe %d – Test der Rückführung: Kamera startet %g° verdreht, Hund steht still', ...
      E.tf.stufe, E.tf.startfehler_grad), 'FontWeight', 'bold');
subtitle(tl, sprintf('Erwartung: e(t) = e_0 \\cdot exp(-t/T),  T = 1/K_p = %.2f s', T));
ax1 = kachel(nexttile(tl), E.t, E.fehler, '[°]', ...
      sprintf('Simulation: Kamerafehler   (bei t = T: %.3f°)', interp1(E.t, E.fehler, T)), c.blau);
ax2 = kachel(nexttile(tl), E.t, theo, '[°]', ...
      sprintf('Theorie PT1   (bei t = T: %.3f° = 37 %% des Startwerts)', E.tf.startfehler_grad*exp(-1)), c.rot);
ax3 = kachel(nexttile(tl), E.t, E.fehler - theo, '[°]', ...
      'Abweichung Simulation minus Theorie (Stufe 1: sehr klein)', c.grau);
set([ax1 ax2 ax3], 'XLim', [0 min(6*T, E.t(end))]);
xlabel(ax3, 't [s]');
end

function f = fig_geschwindigkeit(V, Z1)
c  = farben();
f  = neue_figur("Geschwindigkeitsstudie", true, 3);
tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact');
title(tl, 'Wie schnell darf der A2W fahren?  (mit Rückführung, Gelände Klasse E)', 'FontWeight', 'bold');
subtitle(tl, 'Farbe = Stufe (blau ideal, orange realistisch),  Linie = Pose (durchgezogen vorn, gestrichelt seite)');
ax = gobjects(3, 1);
for i = 1:3, ax(i) = nexttile(tl); hold(ax(i), 'on'); grid(ax(i), 'on'); end

legende = strings(0);
stufen  = unique(arrayfun(@(E) E.tf.stufe, V));  stufen = stufen(:)';
for s = stufen
    for pose = ["vorn" "seite"]
        idx = arrayfun(@(E) E.tf.stufe == s && E.tf.pose == pose, V);
        if ~any(idx), continue; end
        W      = V(idx);
        [v, o] = sort(arrayfun(@(E) E.tf.v, W));
        W      = W(o);
        farbe = c.blau;  if s == 2, farbe = c.orange; end
        stil  = '-';     if pose == "seite", stil = '--'; end
        args  = {'Color', farbe, 'LineStyle', stil, 'Marker', 'o', 'LineWidth', 1.3};
        plot(ax(1), v, [W.reduktion],  args{:});
        plot(ax(2), v, [W.rms_fehler], args{:});
        plot(ax(3), v, [W.max_dq],     args{:});
        legende(end+1) = "Stufe " + s + ", Blick " + pose;   %#ok<AGROW>
    end
end
yline(ax(3), rad2deg(max(Z1.dq_max)), 'r--', 'Z1-Grenze');
legend(ax(1), legende, 'Location', 'southwest');
set(ax(2), 'YScale', 'log');
set(ax, 'XLim', [0.5 5.5], 'XTick', 1:5);
ylabel(ax(1), '[%]');         title(ax(1), 'Um wie viel Prozent wird der Kamerafehler verringert?', 'FontWeight', 'normal');
ylabel(ax(2), '[°] (log)');   title(ax(2), 'Wie groß bleibt der Kamerafehler? (RMS)', 'FontWeight', 'normal');
ylabel(ax(3), '[°/s]');       title(ax(3), 'Wie schnell soll sich das schnellste Gelenk drehen? (Befehl vor der Sättigung)', 'FontWeight', 'normal');
xlabel(ax(3), 'Fahrgeschwindigkeit v [m/s]');
end
function f = fig_totzeit(V)
c  = farben();
f  = neue_figur("Totzeitstudie", true, 2);
tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact');
title(tl, 'Wie stark schadet die Totzeit?  (Stufe 2, mit Rückführung, Blick vorn)', 'FontWeight', 'bold');
subtitle(tl, 'Totzeit = Verzögerung IMU -> Regler; dazu kommen immer 1 ms Abtastung und 10 ms Gelenk-PT1');
ax1 = nexttile(tl);  hold(ax1, 'on');  grid(ax1, 'on');
ax2 = nexttile(tl);  hold(ax2, 'on');  grid(ax2, 'on');
farb    = {c.blau, c.orange, c.gruen, c.rot};
vs      = unique(arrayfun(@(E) E.tf.v, V));  vs = vs(:)';
legende = strings(0);
for i = 1:numel(vs)
    W        = V(arrayfun(@(E) E.tf.v == vs(i), V));
    [tau, o] = sort(arrayfun(@(E) E.tf.tau, W));
    W        = W(o);
    args     = {'Color', farb{i}, 'Marker', 'o', 'LineWidth', 1.3};
    plot(ax1, 1000*tau, [W.reduktion],  args{:});
    plot(ax2, 1000*tau, [W.rms_fehler], args{:});
    legende(end+1) = "v = " + vs(i) + " m/s";                %#ok<AGROW>
end
legend(ax1, legende, 'Location', 'northeast');
ylabel(ax1, '[%]');  title(ax1, 'Um wie viel Prozent wird der Kamerafehler verringert?', 'FontWeight', 'normal');
ylabel(ax2, '[°]');  title(ax2, 'Wie groß bleibt der Kamerafehler? (RMS)', 'FontWeight', 'normal');
xlabel(ax2, 'Totzeit \tau [ms]');
end

% =============================== Tabelle ==================================
function T = kennwert_tabelle(ERG)
n = numel(ERG);
Lauf = strings(n,1);  Stufe = zeros(n,1);  Pose = strings(n,1);  v_m_s = zeros(n,1);
RMS_Hund_grad = zeros(n,1);  RMS_Kamera_grad = zeros(n,1);  Max_Kamera_grad = zeros(n,1);
Reduktion_Proz = zeros(n,1);  Max_dq_grad_s = zeros(n,1);
for k = 1:n
    E = ERG(k);
    Lauf(k)            = E.tf.name;
    Stufe(k)           = E.tf.stufe;
    Pose(k)            = E.tf.pose;
    v_m_s(k)           = E.tf.v;
    RMS_Hund_grad(k)   = E.rms_hund;
    RMS_Kamera_grad(k) = E.rms_fehler;
    Max_Kamera_grad(k) = E.max_fehler;
    Reduktion_Proz(k)  = E.reduktion;
    Max_dq_grad_s(k)   = E.max_dq;
end
T = table(Lauf, Stufe, Pose, v_m_s, RMS_Hund_grad, RMS_Kamera_grad, Max_Kamera_grad, ...
          Reduktion_Proz, Max_dq_grad_s);
end

% ============================ Hilfsfunktionen =============================
function ax = kachel(ax, t, y, ylab, titel, farbe)
% Eine Kachel = ein Signal, eigene y-Skala, Titel als Frage
plot(ax, t, y, 'Color', farbe, 'LineWidth', 1.1);  grid(ax, 'on');
ylabel(ax, ylab);
title(ax, titel, 'FontWeight', 'normal');
xlim(ax, [t(1) t(end)]);
end

function balken(ax, x, Y, c)
% Gruppierte Balken: eine Farbe pro Stufe
b    = bar(ax, x, Y);
farb = {c.blau, c.orange, c.gruen};
for j = 1:numel(b), b(j).FaceColor = farb{j}; end
grid(ax, 'on');
end

function f = neue_figur(name, vis, n_kacheln)
f = figure('Name', name, 'Color', 'w', 'Visible', matlab.lang.OnOffSwitchState(vis), ...
           'Position', [60 40 1000 min(140 + 150*n_kacheln, 1000)]);
hell(f);                                   % helles Theme, gut lesbare Schrift
end

function s = beschreibung(E)
s = sprintf('Stufe %d, Szenario "%s", v = %.1f m/s, Pose "%s", k_{ff} = %g, K_p = %g', ...
            E.tf.stufe, E.tf.szenario, E.tf.v, E.tf.pose, E.tf.k_ff, E.tf.Kp);
end

function s = red_text(E)
if isnan(E.reduktion), s = ''; else, s = sprintf(',  Reduktion %.1f %%', E.reduktion); end
end

function n = gelenk_namen()
n = ["Gelenk 1 (Basis gieren, z)", "Gelenk 2 (Schulter, y)", "Gelenk 3 (Ellbogen, y)", ...
     "Gelenk 4 (Handgelenk nicken, y)", "Gelenk 5 (Handgelenk gieren, z)", ...
     "Gelenk 6 (Handgelenk rollen, x)"];
end

function c = farben()
c.blau   = [0 0.45 0.74];
c.rot    = [0.85 0.33 0.10];
c.grau   = [0.45 0.45 0.45];
c.gruen  = [0.47 0.67 0.19];
c.orange = [0.93 0.55 0.13];
end