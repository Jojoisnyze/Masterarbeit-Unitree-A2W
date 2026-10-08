%% Trajektoriengenerierung für Unitree A2W (Hund-Simulation)
% Dieses Skript wird im start_model aufgerufen und lädt die Daten in den Workspace

function A2W = Simulation_A2W()

% 1. Zeiteinstellungen
dt = 0.01;                  % Abtastzeit (100 Hz)
t = (0:dt:15)';             % Simulation für 15 Sekunden
N = length(t);

% 2. Lauf-Phasen (Geschwindigkeiten in m/s) definieren
v_x = zeros(N, 1);
v_y = zeros(N, 1);

for i = 1:N
    if t(i) <= 5.0
        % Phase 1: Geradeaus (0 - 5s)
        v_x(i) = 0.4;       
        v_y(i) = 0.0;
    elseif t(i) <= 10.0
        % Phase 2: Schräg laufen (5 - 10s)
        v_x(i) = 0.4;
        v_y(i) = 0.2;
    else
        % Phase 3: Rückwärts laufen (10 - 15s)
        v_x(i) = -0.3;
        v_y(i) = 0.0;
    end
end

% 3. Geschwindigkeiten aufintegrieren, um die Basis-Position (X, Y) zu erhalten
X_base = zeros(N, 1);
Y_base = zeros(N, 1);
for i = 2:N
    X_base(i) = X_base(i-1) + v_x(i) * dt;
    Y_base(i) = Y_base(i-1) + v_y(i) * dt;
end

% 4. Dynamisches Wackeln generieren (Hundegang / Trot)
f = 2.0;                    % Schrittfrequenz: 2 Hz (typisch für schnelles Gehen)
omega = 2 * pi * f;

% Amplituden des Wackelns (in Metern bzw. Radiant)
A_z = 0.015;                % 1.5 cm Auf- und Ab-Hüpfen
A_roll = 0.035;             % Seitliches Wanken (~2 Grad)
A_pitch = 0.05;             % Vor-Zurück Nicken (~3 Grad)
A_yaw = 0.015;              % Leichtes Gieren um die Hochachse

% Das Wackeln findet nur statt, wenn der Hund auch wirklich läuft
is_moving = abs(v_x) > 0 | abs(v_y) > 0;

Z_wobble = A_z * sin(omega * t) .* is_moving;
Roll_wobble = A_roll * sin(omega/2 * t) .* is_moving; % Rollen hat oft die halbe Frequenz
Pitch_wobble = A_pitch * cos(omega * t) .* is_moving; 
Yaw_wobble = A_yaw * sin(omega/2 * t) .* is_moving;

% 5. Finale Signale zusammenbauen
Z_base = 0.4;               % Der Hund ist ca. 40 cm hoch

X_final = X_base; 
Y_final = Y_base;
Z_final = Z_base + Z_wobble;
Roll_final = Roll_wobble;
Pitch_final = Pitch_wobble;
Yaw_final = Yaw_wobble;

% 6. Als Timeseries-Objekte in einem Struct für Simulink verpacken
A2W.Hund_X = timeseries(X_final, t);
A2W.Hund_Y = timeseries(Y_final, t);
A2W.Hund_Z = timeseries(Z_final, t);
A2W.Hund_Roll = timeseries(Roll_final, t);
A2W.Hund_Pitch = timeseries(Pitch_final, t);
A2W.Hund_Yaw = timeseries(Yaw_final, t);
A2W.dt = dt;

disp('A2W-Trajektorien generiert.');
end