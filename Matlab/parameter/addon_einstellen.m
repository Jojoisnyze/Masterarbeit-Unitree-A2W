function ADD = addon_einstellen(addon)
% ADDON_EINSTELLEN  Übersetzt den Add-on-Namen eines Laufs in die Schalter für Simulink.
% Jedes Add-on ist im Regler ein Variant Subsystem ("aus"/"an"), gesteuert über
% eine einfache Variable (0/1), genau wie STUFE:
%   PRAEDIKTION  Add-on 1: prädiktive Störgrößenaufschaltung (Woolfrey 2021)
%   HEAVE        Add-on 2: Ausgleich des Hochfederns (Roncone Gl. 13a, Meng 2026)
% Kombinationen mit "+":  "praediktion+heave"
% Neues Add-on: hier einen Schalter und einen case ergänzen.
arguments
    addon (1,1) string = "keins"
end

ADD.praediktion = 0;
ADD.heave       = 0;
if addon == "keins", return; end

teile = strtrim(split(addon, "+"));
for i = 1:numel(teile)
    switch teile(i)
        case "praediktion"
            ADD.praediktion = 1;
        case "heave"
            ADD.heave = 1;
        otherwise
            error('Unbekanntes Add-on: "%s". Erlaubt: keins, praediktion, heave und Kombinationen mit "+".', teile(i));
    end
end
end