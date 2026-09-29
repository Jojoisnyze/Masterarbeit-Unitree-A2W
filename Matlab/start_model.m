function start_model()

clc;
clear all;
close all;

S = load('para_mdl.mat');

p = S.para_mdl;

para_mdl=p;

p=para_mdl;
x0=[0; 0; 0; 0; 0; 0; 0; 0];
x0_omega_r=p.omega_r_R;

%Arbeitspunkte:
beta_e = [ ...
    0.0000;
    0.0306;
    0.0560;
    0.0772;
    0.0953;
    0.1113;
    0.1254;
    0.1382;
    0.1499;
    0.1607;
    0.1783;
    0.2134;
    0.2421;
    0.2669;
    0.2890;
    0.3091;
    0.3279;
    0.3456;
    0.3625;
    0.3789];

v_e = [ ...
    11.2600;
    11.5378;
    11.8156;
    12.0933;
    12.3711;
    12.6489;
    12.9267;
    13.2044;
    13.4822;
    13.7600;
    14.2600;
    15.4533;
    16.6467;
    17.8400;
    19.0333;
    20.2267;
    21.4200;
    22.6133;
    23.8067;
    25.0000];

ue = [beta_e, v_e];

p.v_e=v_e;
p.beta_e=beta_e;

[Ki,Kp]=CntrlParaR3(p,0.5);
p.Ki=Ki;
p.Kp=Kp;


p.x0=x0; %Startwert Zustand
p.x0_region3 = x0_omega_r;
p.ue=ue;




%Übergabe an den Workspace
assignin('base','p',p);
%assignin('base','para_mdl',para_mdl);

%Simulink-Modell Version 2024b Aufruf:
%open('Modell_Windturbine.slx');


%Simulink-Modell Version 2024a Aufruf:
open('Roboter_Arm.slx');

end