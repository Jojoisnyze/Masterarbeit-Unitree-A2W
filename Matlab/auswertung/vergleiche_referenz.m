function V = vergleiche_referenz(ERG, refdatei, ORDNER)
% VERGLEICHE_REFERENZ  Vergleicht jeden Lauf mit dem gleichnamigen Lauf einer Referenz.
% Referenz = 93_Kennwerte.csv eines früheren Stands (z. B. referenz/v0.3/).
% Zuordnung über den Basisnamen (Laufname ohne Add-on-Zusatz).
% Ausgabe: Tabelle 96_Vergleich_Referenz.csv + Grafik (nur Basis-Testfälle).
%   Aenderung_Proz < 0  -> Kamerafehler kleiner als in der Referenz (besser)
%   Aenderung_Proz > 0  -> größer (schlechter)
V = table();
if isempty(refdatei) || ~isfile(refdatei)
    fprintf('Kein Referenzvergleich (Datei nicht gefunden: %s)\n', refdatei);
    return
end
R = readtable(refdatei, 'Delimiter', ';', 'TextType', 'string', 'Encoding', 'UTF-8');

%% 1. Tabelle: jeder Lauf gegen seinen Referenzlauf
n = numel(ERG);
Lauf = strings(n,1);  Addon = strings(n,1);
RMS_Ref = nan(n,1);   RMS_Neu = nan(n,1);
Red_Ref = nan(n,1);   Red_Neu = nan(n,1);
for k = 1:n
    E = ERG(k);
    Lauf(k)    = E.tf.name;
    Addon(k)   = E.tf.addon;
    RMS_Neu(k) = E.rms_fehler;
    Red_Neu(k) = E.reduktion;
    i = find(R.Lauf == E.tf.basisname, 1);
    if ~isempty(i)
        RMS_Ref(k) = R.RMS_Kamera_grad(i);
        Red_Ref(k) = R.Reduktion_Proz(i);
    end
end
Aenderung_Proz = 100 * (RMS_Neu - RMS_Ref) ./ RMS_Ref;
V = table(Lauf, Addon, RMS_Ref, RMS_Neu, Aenderung_Proz, Red_Ref, Red_Neu);
disp(V);
writetable(V, fullfile(ORDNER, "96_Vergleich_Referenz.csv"), 'Delimiter', ';');

%% 2. Grafik: Änderung je Basis-Testfall
ist_basis = arrayfun(@(E) E.tf.gruppe == "basis", ERG);
namen = Lauf(ist_basis);
d     = Aenderung_Proz(ist_basis);
if ~isempty(d)
    f  = figure('Name', 'Vergleich mit Referenz', 'Color', 'w', ...
        'Position', [60 40 1000 min(200 + 24*numel(d), 1000)]);
    hell(f);
    ax = axes(f);
    x  = reordercats(categorical(namen), namen);
    b  = barh(ax, x, d, 'FaceColor', 'flat');
    farbe           = repmat([0 0.45 0.74], numel(d), 1);          % blau = besser oder gleich
    farbe(d > 0, :) = repmat([0.85 0.33 0.10], nnz(d > 0), 1);     % rot  = schlechter
    b.CData = farbe;
    set(ax, 'YDir', 'reverse');  grid(ax, 'on');  xline(ax, 0, 'k');
    xlabel(ax, 'Änderung des RMS-Kamerafehlers gegenüber der Referenz [%]   (negativ = besser)');
    [~, refname, rest] = fileparts(fileparts(refdatei));
    refname = refname + rest;                         % "v0" + ".3"
    title(ax, "Was hat sich gegenüber der Referenz " + refname + " verändert?", 'FontWeight', 'bold');
    speichere_figur(f, ORDNER, "96_Vergleich_Referenz");
end

%% 3. Warnung, wenn das Grundgerüst (ohne Add-on) schlechter geworden ist
schlechter = Addon == "keins" & Aenderung_Proz > 1;
if any(schlechter)
    warning('Ohne Add-on schlechter als die Referenz (> 1 %%):\n  %s', strjoin(Lauf(schlechter), '\n  '));
else
    fprintf('Referenzvergleich: Grundgerüst unverändert oder besser.\n');
end
end