function [h, p_WC] = ee_hoehe(q, R_WB, z_B, Z1, MONT)
% EE_HOEHE  Höhe des Endeffektors (Frame C) im Weltsystem.
%
% Quelle: Modern Robotics Kap. 3.3 (Verkettung homogener Transformationen)
%   p_WC = p_WB + R_WB * (p_B0 + R_B0 * p_0C(q)),   p_WB = [0; 0; z_B]
%   h    = z-Komponente von p_WC
% z_B ist der Hub des Rumpfes gegenüber seiner Standhöhe (Störung, Kanal 4),
% also relativ zum Boden. x und y des Rumpfes ändern die Höhe nicht.
% Eine Rumpfdrehung verschiebt den Endeffektor über den Hebelarm (Roncone Gl. 13a).
%
% Wird von "Gütemaß" (Auswertung) und später vom Regler (Add-on 2) aufgerufen,
% damit beide garantiert dasselbe rechnen.
T_0C = z1_fk(q, Z1, MONT.T_6C);
p_WC = [0; 0; z_B] + R_WB * (MONT.p_B0 + MONT.R_B0 * T_0C(1:3, 4));
h    = p_WC(3);
end