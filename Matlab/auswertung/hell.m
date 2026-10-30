function hell(f)
% HELL  Helles Erscheinungsbild für eine Figure, unabhängig vom MATLAB-Theme
% (im dunklen Theme wäre die Schrift auf weißem Hintergrund kaum lesbar).
try
    theme(f, "light");          % ab R2025a: Figure-Theme festlegen
catch
end
f.Color = 'w';
set(f, 'DefaultAxesFontSize', 11, 'DefaultTextFontSize', 11);
end