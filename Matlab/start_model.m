%% start_model.m – Einstieg: Kamerastabilisierung Unitree Z1 auf A2W
% Nur diese Datei ausführen (F5 bzw. "Run"). Sie lädt alle Parameter in den
% Workspace, führt die Tests aus und öffnet (später) das Simulink-Modell.
clear; clc; close all;

%% 0. Pfade – relativ zu dieser Datei
root = fileparts(mfilename('fullpath'));
if isempty(root), root = pwd; end          % falls abschnittsweise ausgeführt
addpath(fullfile(root,'parameter'), ...
        fullfile(root,'funktionen'), ...
        fullfile(root,'tests'), ...
        fullfile(root,'auswertung/'));
urdfDatei = fullfile(root,'z1_description','xacro','z1.urdf');

%% 1. Simulationsparameter und Grundpose ("vorn" oder "seite")
SIM = par_sim("vorn");

%% 2. Kinematik des Z1 aus der URDF
[Z1, Z1_info] = par_z1(urdfDatei);

%% 3. Montage (Platzhalter!)
MONT = par_montage();

%% 4. Störung des A2W
% Szenario: 'aus','roll','nick','gier','kombi','fahren','laufen'
% Optionen (nur fahren/laufen): v [m/s], klasse "A".."H", steigung_grad, quer_grad
STOER = par_stoerung("fahren", SIM, v=2.0, klasse="E", steigung_grad=0, quer_grad=0);
A2W   = stoerung_a2w((0:SIM.Ts:SIM.Tend)', STOER);

%% 5. Regler
REG = par_regler(SIM, Z1, MONT);

%% 5b. Stufe 2: Sensor- und Gelenkmodell (Platzhalter!)
SEN = par_sensor(SIM);
GEL = par_gelenk(Z1);

%% 6. Tests
test_par_z1(Z1, Z1_info);
test_z1_fk(Z1, MONT, urdfDatei);
test_z1_jacobian(Z1, MONT, urdfDatei);
test_stoerung(SIM);
test_regler(Z1, MONT, SIM, REG);

%% 7. Ergebnisordner vorbereiten, Störung anzeigen und speichern
ORDNER = fullfile(root, 'ergebnisse');
ordner_leeren(ORDNER);                         % alte Grafiken löschen
figs = plot_stoerung(A2W, STOER);
speichere_figur(figs(1), ORDNER, "00_Stoerung_Lage");
speichere_figur(figs(2), ORDNER, "00_Stoerung_Drehraten");

%% 8. Testfall (Testprotokoll) – Standard aus par_regler: k_ff = 1, Kp = 3
% ohne Stabilisierung:  REG.k_ff = 0;  REG.Kp = 0;
% nur IMU (Roncone):    REG.k_ff = 1;  REG.Kp = 0;
% mit Rückführung:      REG.k_ff = 1;  REG.Kp = 3;
% Test Rückführung:     Szenario "aus" wählen und Startfehler 5° um x:
%   REG.R_WC_des = rot_achse([1;0;0], deg2rad(5)) * REG.R_WC_des;

%% 9. Signale für Simulink (From Workspace)
ts_R_WB = timeseries(A2W.R_WB, A2W.t);                    % 3x3xN, Zeit in letzter Dim.
ts_w_B  = timeseries(permute(A2W.w_B, [2 3 1]), A2W.t);    % 3x1xN (Spaltenvektor)

%% 10. Simulink-Modell öffnen und Testfälle durchlaufen
% STUFE wählt die aktive Variante (Sensor IMU, Gelenkmodell):
%   1 = ideal (Mathematik-Prüfung)
%   2 = Abtastung, Totzeit, Gelenk-PT1, Sättigung, Winkelgrenzen
STUFE  = 1;                      % gilt für das manuelle Arbeiten im Modell
modell = 'Kamerastabilisierung';
open_system(modell);

TESTLAUF         = true;     % true: Testfälle simulieren
STUFEN           = [1 2];    % jede dieser Stufen rechnet alle Basis-Testfälle
V_STUDIE         = true;     % zusätzlich v = 1...5 m/s, beide Posen (dauert länger)
TAU_STUDIE       = true;     % zusätzlich Stufe 2 mit tau = 2...40 ms (10 Läufe)
DETAILS_ANZEIGEN = false;    % true: auch die Detail-Figures je Testfall öffnen
if TESTLAUF
    TF  = def_testfaelle(STUFEN, V_STUDIE, TAU_STUDIE);
    fprintf('Starte %d Simulationsläufe ...\n', numel(TF));
    ERG = simuliere_testfaelle(TF, Z1, MONT, modell);
    plot_testfaelle(ERG, Z1, ORDNER, DETAILS_ANZEIGEN);
    pruefe_regression(ERG, SEN, GEL);   % nach den Plots, damit du bei Abbruch nachsehen kannst
end