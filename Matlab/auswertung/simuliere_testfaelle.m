function ERG = simuliere_testfaelle(TF, Z1, MONT, modell)
% SIMULIERE_TESTFAELLE  Startet das Simulink-Modell einmal pro Testfall und
% sammelt die geloggten Signale in ERG.
% Pro Testfall werden SIM, REG und die Störung (ts_R_WB, ts_w_B) nur für
% diesen einen Lauf überschrieben (setVariable). Der Basis-Workspace bleibt.

load_system(modell);

%% 1. Eingaben pro Testfall vorbereiten
for k = 1:numel(TF)
    SIM_k = par_sim(TF(k).pose);
    STOER = par_stoerung(TF(k).szenario, SIM_k, v = TF(k).v);
    A2W   = stoerung_a2w((0:SIM_k.Ts:SIM_k.Tend)', STOER);

    REG_k      = par_regler(SIM_k, Z1, MONT);
    REG_k.k_ff = TF(k).k_ff;
    REG_k.Kp   = TF(k).Kp;
    if TF(k).startfehler_grad ~= 0
        % Soll um x gedreht -> Kamera startet mit diesem Fehler
        REG_k.R_WC_des = rot_achse([1;0;0], deg2rad(TF(k).startfehler_grad)) * REG_k.R_WC_des;
    end
    if isnan(TF(k).tau)
        SEN_k = par_sensor(SIM_k);                 % Standard-Totzeit
    else
        SEN_k = par_sensor(SIM_k, TF(k).tau);      % Totzeitstudie
    end

    in(k) = Simulink.SimulationInput(modell); %#ok<AGROW>
    in(k) = in(k).setVariable('SEN', SEN_k);
    in(k) = in(k).setVariable('STUFE', TF(k).stufe);
    in(k) = in(k).setVariable('SIM', SIM_k);
    in(k) = in(k).setVariable('REG', REG_k);
    in(k) = in(k).setVariable('ts_R_WB', timeseries(A2W.R_WB, A2W.t));
    in(k) = in(k).setVariable('ts_w_B',  timeseries(permute(A2W.w_B, [2 3 1]), A2W.t));
end

%% 2. Alle Testfälle simulieren
out = sim(in, 'ShowProgress', 'on');

%% 3. Ergebnisse abholen und Kennwerte berechnen
for k = 1:numel(TF)
    if ~isempty(out(k).ErrorMessage)
        error('Testfall "%s" fehlgeschlagen:\n%s', TF(k).name, out(k).ErrorMessage);
    end
    E.tf = TF(k);
    [E.t, E.fehler] = signal(out(k), 'fehler_grad');   % [°]
    [~,   E.hund]   = signal(out(k), 'hund_grad');     % [°]
    [~,   E.e_W]    = signal(out(k), 'e_W_grad');      % Nx3 [°]
    [~,   E.q]      = signal(out(k), 'q');             % Nx6 [rad]
    [~,   E.dq]     = signal(out(k), 'dq_cmd');        % Nx6 [rad/s]

    i = E.t >= 1;                                      % ohne Einblendphase
    E.rms_fehler = sqrt(mean(E.fehler(i).^2));
    E.max_fehler = max(E.fehler(i));
    E.rms_hund   = sqrt(mean(E.hund(i).^2));
    E.max_hund   = max(E.hund(i));
    E.reduktion  = 100 * (1 - E.rms_fehler / E.rms_hund);   % wie Roncone: % weniger
    if E.rms_hund == 0, E.reduktion = NaN; end
    E.max_dq     = rad2deg(max(abs(E.dq), [], 'all'));

    ERG(k) = E;                                        %#ok<AGROW>
end
end

% ---------------------------------------------------------------------------
function [t, d] = signal(o, name)
% Holt ein geloggtes Signal und bringt es auf die Form N x Breite
ts = o.logsout.get(name).Values;
t  = ts.Time;
d  = squeeze(ts.Data);
if size(d, 1) ~= numel(t), d = d.'; end
end