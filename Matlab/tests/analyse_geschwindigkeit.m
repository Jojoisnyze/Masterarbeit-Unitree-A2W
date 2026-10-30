function analyse_geschwindigkeit(SIM, klasse)
% ANALYSE_GESCHWINDIGKEIT  Wie hängt die Störung von der Fahrgeschwindigkeit ab?
% Aufruf: analyse_geschwindigkeit(SIM)  oder  analyse_geschwindigkeit(SIM, "D")
if nargin < 2, klasse = "E"; end
t = (0:SIM.Ts:SIM.Tend)';
fprintf('Klasse %s:   v [m/s] | max Roll [°] | max Nick [°] | max w_x [°/s] | max w_y [°/s]\n', klasse);
for v = 1:5
    STOER = par_stoerung("fahren", SIM, v=v, klasse=klasse);
    A = stoerung_a2w(t, STOER);
    i = t >= STOER.T_ein;
    e = rad2deg(abs(A.eul(i,:)));  w = rad2deg(abs(A.w_B(i,:)));
    fprintf('            %4.1f   |    %5.1f     |    %5.1f     |     %5.0f     |     %5.0f\n', ...
        v, max(e(:,1)), max(e(:,2)), max(w(:,1)), max(w(:,2)));
end
fprintf('Z1-Gelenkgrenze: 180 °/s\n');
end