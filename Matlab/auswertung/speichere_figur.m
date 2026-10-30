function speichere_figur(fig, ordner, name)
% SPEICHERE_FIGUR  Speichert eine Figure als PNG (Bericht) und FIG (zum Zoomen).
% Gleichnamige Dateien werden überschrieben.
% Ist die Figure schon geschlossen (z. B. weggeklickt), wird nur gewarnt,
% damit der Testlauf nicht abbricht.
name = string(name);
if isempty(fig) || ~isgraphics(fig) || ~isvalid(fig)
    warning('speichere_figur: Figure "%s" ist geschlossen – nicht gespeichert.', name);
    return
end
drawnow;                                           % Figure vollständig aufbauen lassen
exportgraphics(fig, fullfile(ordner, name + ".png"), 'Resolution', 150);
% Damit die .fig später sichtbar öffnet, auch wenn sie unsichtbar erzeugt wurde:
fig.CreateFcn = 'set(gcbo, ''Visible'', ''on'')';
savefig(fig, fullfile(ordner, name + ".fig"));
end