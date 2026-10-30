function figs = plot_stoerung(A2W, STOER, sichtbar)
% PLOT_STOERUNG  Rumpfbewegung des A2W, ein Signal pro Kachel.
%   figs(1): Lage (Rollen, Nicken, Gieren, Heave)
%   figs(2): Drehraten im Rumpf-Frame (= Gyroskop, Eingang der Aufschaltung)
if nargin < 3, sichtbar = true; end
vis  = matlab.lang.OnOffSwitchState(sichtbar);
t    = A2W.t;
blau = [0 0.45 0.74];  orange = [0.85 0.33 0.10];

info = sprintf('Szenario "%s"', STOER.szenario);
if any(STOER.szenario == ["fahren", "laufen"])
    info = sprintf('%s, v = %.1f m/s, Bodenklasse %s, Hang %g° / quer %g°', info, ...
        STOER.opt.v, STOER.opt.klasse, STOER.opt.steigung_grad, STOER.opt.quer_grad);
end

%% Figure 1: Lage des Rumpfes
figs(1) = figure('Name', 'Störung A2W – Lage', 'Color', 'w', 'Visible', vis, ...
                 'Position', [60 60 1000 760]);
hell(figs(1));                             % helles Theme, gut lesbare Schrift
tl = tiledlayout(figs(1), 4, 1, 'TileSpacing', 'compact');
title(tl, 'Wie bewegt sich der Rumpf des A2W? (wahre Bewegung = Störung)', 'FontWeight', 'bold');
subtitle(tl, info);
y    = {rad2deg(A2W.eul(:,1)), rad2deg(A2W.eul(:,2)), rad2deg(A2W.eul(:,3)), 100*A2W.z};
lab  = {'\phi [°]', '\theta [°]', '\psi [°]', 'z [cm]'};
einh = {'°', '°', '°', ' cm'};
tit  = {'Rollen \phi  (> 0: linke Seite hoch)', ...
        'Nicken \theta  (> 0: Nase runter)', ...
        'Gieren \psi  (> 0: nach links drehen)', ...
        'Heave z  (Auf und Ab – wird im Grundgerüst NICHT ausgeglichen)'};
for i = 1:4
    ax = nexttile(tl);
    plot(ax, t, y{i}, 'Color', blau, 'LineWidth', 1);  grid(ax, 'on');
    ylabel(ax, lab{i});
    title(ax, sprintf('%s   |   max %.2f%s', tit{i}, max(abs(y{i})), einh{i}), 'FontWeight', 'normal');
end
xlabel(ax, 't [s]');

%% Figure 2: Drehraten = Gyroskop
figs(2) = figure('Name', 'Störung A2W – Drehraten', 'Color', 'w', 'Visible', vis, ...
                 'Position', [80 80 1000 600]);
hell(figs(2));
tl = tiledlayout(figs(2), 3, 1, 'TileSpacing', 'compact');
title(tl, 'Was misst das Gyroskop?  ^B\omega_B = Eingang der Aufschaltung', 'FontWeight', 'bold');
subtitle(tl, info);
tit = {'Drehrate um x_B (Rollrate)', 'Drehrate um y_B (Nickrate)', 'Drehrate um z_B (Gierrate)'};
for i = 1:3
    w  = rad2deg(A2W.w_B(:,i));
    ax = nexttile(tl);
    plot(ax, t, w, 'Color', orange, 'LineWidth', 1);  grid(ax, 'on');
    ylabel(ax, '[°/s]');
    title(ax, sprintf('%s   |   max %.0f °/s', tit{i}, max(abs(w))), 'FontWeight', 'normal');
end
xlabel(ax, 't [s]');
end