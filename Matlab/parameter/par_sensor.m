function SEN = par_sensor(SIM, tau, opt)
% PAR_SENSOR  Sensormodell der IMU (Variant "Abgetastet", aktiv ab Stufe 2).
% Stufe 1: ideal durchverbunden.
% Stufe 2: Abtastung mit SIM.Ts und Totzeit (rauschen = false).
% Stufe 3: wie Stufe 2, zusätzlich Gyro-Rauschen und -Bias (rauschen = true).
% Aufruf: SEN = par_sensor(SIM)                         -> 20 ms, ohne Rauschen
%         SEN = par_sensor(SIM, 0.010)                  -> 10 ms, ohne Rauschen
%         SEN = par_sensor(SIM, 0.020, rauschen=true)   -> 20 ms, mit Rauschen und Bias
% !!! PLATZHALTER: Totzeit messen; Rauschdichte und Bias aus Datenblatt/Messung der A2W-IMU !!!
arguments
    SIM struct
    tau (1,1) double = 0.020                     % Totzeit IMU-Pfad [s] (PLATZHALTER)
    opt.rauschen (1,1) logical = false           % Gyro-Rauschen und -Bias (Stufe 3)
end

%% Abtastung und Totzeit
SEN.Ts  = SIM.Ts;                                % Abtastzeit des Reglers [s]
SEN.tau = tau;
SEN.N   = max(1, round(SEN.tau / SEN.Ts));       % Totzeit in Abtastschritten (Delay-Block)
SEN.R0  = repmat(eye(3), 1, 1, SEN.N);           % Anfangswert Lage im Delay: 3x3xN

%% Gyroskop: Rauschen und Bias
% Rauschdichte typischer MEMS-IMUs ca. 0,014 °/s/sqrt(Hz).
% Pro Abtastwert: sigma = Rauschdichte * sqrt(Abtastrate)
SEN.rauschen = opt.rauschen;
SEN.N_g    = deg2rad(0.014);                     % Rauschdichte [rad/s/sqrt(Hz)]      (PLATZHALTER)
SEN.bias_g = deg2rad([0.1; -0.1; 0.05]);         % Restbias nach Kalibrierung [rad/s]  (PLATZHALTER)
SEN.seed_g = [11; 12; 13];                       % reproduzierbares Rauschen je Achse
if ~SEN.rauschen
    SEN.N_g    = 0;
    SEN.bias_g = zeros(3,1);
end
SEN.sigma_g = SEN.N_g * sqrt(1/SEN.Ts);          % Standardabweichung je Abtastwert [rad/s]
SEN.var_g   = SEN.sigma_g^2 * ones(3,1);         % Varianz für den Random-Number-Block

%% Beschleunigungsmesser (Add-on 2): Rauschen, Bias, Anfangswert im Delay
% Misst die spezifische Kraft im Rumpf-Frame: f_B = R_WB'*(a_W + g*e_z) + b_a + n_a
% (Meng et al. 2026, Gl. 43). Rauschdichte typischer MEMS-IMUs ca. 175 µg/sqrt(Hz).
SEN.N_a    = 175e-6 * SIM.g;                     % Rauschdichte [m/s^2/sqrt(Hz)]    (PLATZHALTER)
SEN.bias_a = [0.02; -0.02; 0.03];                % Restbias nach Kalibrierung [m/s^2] (PLATZHALTER)
SEN.seed_a = [21; 22; 23];                       % reproduzierbares Rauschen je Achse
if ~SEN.rauschen
    SEN.N_a    = 0;
    SEN.bias_a = zeros(3,1);
end
SEN.sigma_a = SEN.N_a * sqrt(1/SEN.Ts);          % Standardabweichung je Abtastwert [m/s^2]
SEN.var_a   = SEN.sigma_a^2 * ones(3,1);         % Varianz für den Random-Number-Block
% Anfangswert im Delay: Hund steht -> f_B = [0; 0; g]. Ohne das würde der Sensor
% in den ersten N Schritten 0 melden, also scheinbar freien Fall (-9,81 m/s^2).
SEN.f0      = repmat([0; 0; SIM.g], 1, 1, SEN.N); % 3x1xN wie SEN.R0
end