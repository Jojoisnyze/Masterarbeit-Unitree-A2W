clc;
clear all;
close all;


%JacobiMatrix
Z1_Tree = importrobot('/Users/johannes/Library/Mobile Documents/com~apple~CloudDocs/Elektrotechnik /Masterarbeit/Matlab/Version_1/Matlab/z1_description/xacro/z1.urdf');

% 1. Hunde-Trajektorien berechnen und gezielt als Struct "A2W" ablegen
disp('Generiere Trajektorien für den A2W...');
A2W = Simulation_A2W(); 

% 2. Simulink-Modell öffnen
disp('Öffne Simulink-Modell...');
open('High_Lvl_Regelung_Winkel.slx');