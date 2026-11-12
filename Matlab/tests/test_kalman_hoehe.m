function test_kalman_hoehe(SIM)
% TEST_KALMAN_HOEHE  Prüft den Kalman-Filter (Add-on 2) offline, ohne Simulink.
% Daten aus dem Störmodell ("heave" und "laufen"), Totzeiten und Rauschen wie im Modell:
%   ideal:    ohne Totzeit, ohne Rauschen -> Schätzung = Wahrheit (< 0,1 mm)
%   Stufe 2:  Totzeiten IMU und Beinhöhe -> Kalman schätzt z mind. 3x besser als "direkt"
%   Stufe 3:  zusätzlich Rauschen und Bias -> wie Stufe 2, und der Bias wird gelernt
t = (0:SIM.Ts:10)';   n = numel(t);   i = t >= 2;
for szen = ["heave" "laufen"]
    A    = stoerung_a2w(t, par_stoerung(szen, SIM));
    wahr = [A.z, A.dz];
    a_z  = zeros(n, 1);
    for k = 1:n
        a_z(k) = [0 0 1] * A.R_WB(:,:,k) * A.f_B(k,:).' - SIM.g;   % wie im Simulink-Block
    end
    for fall = ["ideal" "Stufe 2" "Stufe 3"]
        verz   = fall ~= "ideal";
        rausch = fall == "Stufe 3";
        SEN = par_sensor(SIM, rauschen = rausch);
        HOE = par_hoehe(SIM, rauschen = rausch);
        KAL = par_kalman(SIM, SEN, HOE, verzoegert = verz);

        zuf = RandStream('mt19937ar', 'Seed', 5);
        a_m = verschiebe(a_z + SEN.bias_a(3) + SEN.sigma_a*randn(zuf, n, 1), verz*SEN.N);
        y_m = verschiebe(wahr + HOE.sigma.' .* randn(zuf, n, 2),             verz*HOE.N_bein);

        x = zeros(3,1);  P = KAL.P0;  ub = zeros(1, KAL.d + 2);  sch = zeros(n, 2);
        for k = 1:n
            [x, P, ub, h] = kalman_hoehe(x, P, ub, a_m(k), y_m(k,:), KAL);
            sch(k,:) = h.';
        end
        e_k = 1000 * sqrt(mean((sch(i,:) - wahr(i,:)).^2));   % [mm, mm/s]
        e_d = 1000 * sqrt(mean((y_m(i,:) - wahr(i,:)).^2));
        fprintf(['  %-6s %-7s Kalman: z %5.2f mm, dz %6.1f mm/s | direkt: z %5.2f mm, ' ...
            'dz %6.1f mm/s | Bias %+.3f (wahr %+.3f) m/s^2\n'], ...
            szen, fall, e_k, e_d, x(3), SEN.bias_a(3));

        if fall == "ideal"
            assert(e_k(1) < 0.1, 'test_kalman_hoehe: ohne Störungen weicht z um %.2f mm ab', e_k(1));
        else
            assert(e_k(1) < 0.3*e_d(1), 'test_kalman_hoehe: Kalman nicht besser als direkt (%s, %s)', szen, fall);
        end
        if rausch
            assert(abs(x(3) - SEN.bias_a(3)) < 0.01, 'test_kalman_hoehe: Bias nicht gelernt (%.3f)', x(3));
        end
    end
end
disp('test_kalman_hoehe: OK');
end

function v = verschiebe(v, N)
% Totzeit von N Schritten: vorne mit Nullen auffüllen (Hund steht anfangs)
v = [zeros(N, size(v,2)); v(1:end-N, :)];
end