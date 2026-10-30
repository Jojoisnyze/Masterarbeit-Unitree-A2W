function ordner_leeren(ordner)
% ORDNER_LEEREN  Legt den Ergebnisordner an bzw. löscht alte Ergebnisse darin,
% damit jeder Lauf von start_model einen sauberen, aktuellen Stand erzeugt.
if ~isfolder(ordner)
    mkdir(ordner);
    return;
end
for muster = ["*.png", "*.fig", "*.csv", "*.mat"]
    if ~isempty(dir(fullfile(ordner, muster)))
        delete(fullfile(ordner, muster));
    end
end
end