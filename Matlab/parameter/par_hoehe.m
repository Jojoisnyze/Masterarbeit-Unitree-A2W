function HOE = par_hoehe(SIM, opt)
% PAR_HOEHE  Höhenmessung für Add-on 2: Sensor "Beinhöhe (Unitree)".
% Der A2W schätzt Höhe und Hubgeschwindigkeit selbst (Beinkinematik + IMU,
% Unitree SDK2: SportModeState.position/body_height, velocity).
% Modell nach derselben Stufenleiter wie die IMU (par_sensor):
%   Stufe 1: ideal
%   Stufe 2: Abtastung + Totzeit tau_bein (enthält Filterverzug des Unitree-Schätzers)
%   Stufe 3: zusätzlich Rauschen auf Höhe und Geschwindigkeit
% Signal hub = [z_B; dz_B] relativ zur Standhöhe (= relativ zum Boden).
% !!! PLATZHALTER: Rate, Totzeit, Rauschen am A2W messen (SportModeState gegen Motion Capture) !!!
arguments
    SIM struct
    opt.rauschen (1,1) logical = false       % Rauschen (Stufe 3)
end

HOE.Ts       = SIM.Ts;                         % Abtastzeit [s]                  (PLATZHALTER)
HOE.tau_bein = 0.030;                          % Totzeit inkl. Filterverzug [s]  (PLATZHALTER)
HOE.N_bein   = max(1, round(HOE.tau_bein / HOE.Ts));   % Delay in Abtastschritten

HOE.sigma = [0.002; 0.02];                     % [m; m/s] je Abtastwert          (PLATZHALTER)
HOE.seed  = [31; 32];                          % reproduzierbares Rauschen
if ~opt.rauschen
    HOE.sigma = [0; 0];
end
HOE.var = HOE.sigma.^2;                        % Varianz für den Random-Number-Block
end