%% start_model.m – Einstieg: Kamerastabilisierung Unitree Z1 auf A2W
% Nur diese Datei ausführen (F5 bzw. "Run"). Sie lädt alle Parameter in den
% Workspace, führt die Tests aus, öffnet das Simulink-Modell und simuliert
% alle in AUSWAHL (Abschnitt 11) gewählten Läufe.
clear; clc; close all;

%% 0. Pfade – relativ zu dieser Datei
root = fileparts(mfilename('fullpath'));
if isempty(root), root = pwd; end          % falls abschnittsweise ausgeführt
addpath(fullfile(root,'parameter'), ...
        fullfile(root,'funktionen'), ...
        fullfile(root,'tests'), ...
        fullfile(root,'auswertung'));
urdfDatei = fullfile(root,'z1_description','xacro','z1.urdf');

%% 1. Simulationsparameter und Grundpose ("vorn" oder "seite")
SIM = par_sim("vorn");

%% 2. Kinematik des Z1 aus der URDF
[Z1, Z1_info] = par_z1(urdfDatei);

%% 3. Montage (Platzhalter!)
MONT = par_montage();

%% 4. Störung des A2W (für das manuelle Arbeiten im Modell)
% Szenario: 'aus','roll','nick','gier','kombi','fahren','laufen'
% Optionen (nur fahren/laufen): v [m/s], klasse "A".."H", steigung_grad, quer_grad
STOER = par_stoerung("fahren", SIM, v=2.0, klasse="E", steigung_grad=0, quer_grad=0);
A2W   = stoerung_a2w((0:SIM.Ts:SIM.Tend)', STOER);

%% 5. Regler (Grundgerüst)
REG = par_regler(SIM, Z1, MONT);

%% 5b. Gelenkmodell ab Stufe 2 (Platzhalter!)
% Das Sensormodell SEN hängt von der Stufe ab und wird in Abschnitt 10 gesetzt.
GEL = par_gelenk(Z1);
PRAED = par_praediktion(par_sensor(SIM), GEL);    % Add-on 1: AR-Prädiktor (Woolfrey 2021)

%% 6. Tests
test_par_z1(Z1, Z1_info);
test_z1_fk(Z1, MONT, urdfDatei);
test_z1_jacobian(Z1, MONT, urdfDatei);
test_stoerung(SIM);
test_regler(Z1, MONT, SIM, REG);
test_praediktion(A2W, SIM, PRAED);

%% 7. Ergebnisordner vorbereiten, Störung anzeigen und speichern
ORDNER = fullfile(root, 'ergebnisse');
ordner_leeren(ORDNER);                         % alte Grafiken löschen
figs = plot_stoerung(A2W, STOER);
speichere_figur(figs(1), ORDNER, "00_Stoerung_Lage");
speichere_figur(figs(2), ORDNER, "00_Stoerung_Drehraten");

%% 8. Testfälle von Hand (nur zur Erinnerung, die Testläufe setzen das selbst)
% ohne Stabilisierung:  REG.k_ff = 0;  REG.Kp = 0;
% nur IMU (Roncone):    REG.k_ff = 1;  REG.Kp = 0;
% mit Rückführung:      REG.k_ff = 1;  REG.Kp = 3;
% Test Rückführung:     Szenario "aus" wählen und Startfehler 5° um x:
%   REG.R_WC_des = rot_achse([1;0;0], deg2rad(5)) * REG.R_WC_des;

%% 9. Signale für Simulink (From Workspace)
ts_R_WB = timeseries(A2W.R_WB, A2W.t);                    % 3x3xN, Zeit in letzter Dim.
ts_w_B  = timeseries(permute(A2W.w_B, [2 3 1]), A2W.t);    % 3x1xN (Spaltenvektor)

%% 10. Simulink-Modell öffnen
% Schalter für das manuelle Arbeiten im Modell (die Testläufe setzen sie selbst):
%   STUFE        1 = ideal, 2 = Totzeit + Gelenk, 3 = wie 2 + Gyro-Rauschen/-Bias
%   PRAEDIKTION  Add-on 1 (0 = aus, 1 = an)
%   HEAVE        Add-on 2 (0 = aus, 1 = an)
STUFE       = 1;
PRAEDIKTION = 0;
HEAVE       = 0;
SEN    = par_sensor(SIM, rauschen = STUFE >= 3);    % Sensorparameter passend zu STUFE
modell = 'Kamerastabilisierung';
open_system(modell);

%% 11. Auswahl: was soll simuliert werden?
TESTLAUF             = true;      % false: nur Modell öffnen, nichts simulieren
AUSWAHL.stufen       = [ 2 3];   % welche Stufen rechnen
AUSWAHL.testfaelle   = [3 6];        % [] = alle Basis-Testfälle, z. B. [3 6] = nur TF3 und TF6
AUSWAHL.v_studie     = false;      % TF3 bei v = 1...5 m/s, Posen vorn/seite, je Stufe
AUSWAHL.tau_studie   = false;      % TF3 mit tau = 2...40 ms, je Stufe >= 2
AUSWAHL.addons       = ["keins" "praediktion"];   % später: ["keins" "praediktion" "heave" "praediktion+heave"]
AUSWAHL.addon_stufen = [2 3];     % Add-ons nur in diesen Stufen
AUSWAHL.referenz     = fullfile(root, "referenz", "v0.3", "93_Kennwerte.csv");  % "" = kein Vergleich
DETAILS_ANZEIGEN     = false;     % true: auch die Detail-Figures je Testfall öffnen
% Beispiel nur ein einziger Lauf:
%   AUSWAHL.stufen = 3;  AUSWAHL.testfaelle = 3;
%   AUSWAHL.v_studie = false;  AUSWAHL.tau_studie = false;

if TESTLAUF
    TF  = def_testfaelle(AUSWAHL);
    fprintf('Starte %d Simulationsläufe ...\n', numel(TF));
    ERG = simuliere_testfaelle(TF, Z1, MONT, modell);
    plot_testfaelle(ERG, Z1, ORDNER, DETAILS_ANZEIGEN);
    vergleiche_referenz(ERG, AUSWAHL.referenz, ORDNER);
    pruefe_regression(ERG, GEL);   % nach den Plots, damit du bei Abbruch nachsehen kannst
end