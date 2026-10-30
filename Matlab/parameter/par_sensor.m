function SEN = par_sensor(SIM, tau)
% PAR_SENSOR  Sensormodell der IMU ab Stufe 2 (Variant "Abgetastet").
% Stufe 1: ideal durchverbunden. Stufe 2: Abtastung mit SIM.Ts + Totzeit.
% Aufruf: SEN = par_sensor(SIM)          -> Standard-Totzeit (Platzhalter)
%         SEN = par_sensor(SIM, 0.010)   -> Totzeit 10 ms (Totzeitstudie)
% !!! PLATZHALTER: Totzeit messen (IMU -> Netzwerk -> Regler -> Z1-Befehl) !!!
arguments
    SIM struct
    tau (1,1) double = 0.020                     % Totzeit IMU-Pfad [s] (PLATZHALTER)
end
SEN.Ts  = SIM.Ts;                                % Abtastzeit des Reglers [s]
SEN.tau = tau;
SEN.N   = max(1, round(SEN.tau / SEN.Ts));       % Totzeit in Abtastschritten (Delay-Block)
SEN.R0  = repmat(eye(3), 1, 1, SEN.N);           % Anfangswert Lage im Delay: 3x3xN
end