%% Function to calculate control parameter for (Volllastzone) Region 3 of 1DOF WEA Model
%
% Input: v_inf (columnvector); beta (columnvector) ; tau (const.)
%
% Output:
% Ki - integrator parameter (vector);
% Kp - proportional parameter (vector)
%
function [Ki, Kp] = CntrlParaR3(p,  Referenzzeitkonstante)


% Convert from name to greek sign and calculate Schnelllaufzahl
lambda = (p.R * p.omega_r_R)./p.v_e;
beta = p.beta_e;
tau = Referenzzeitkonstante;

% Coefficients from Curvefit of CQ values from FAST-Model Simulation
c1  = 0.005;
c2  = 1.53;
c3  = 0.5;
c4  = 0.18;
c5  = 121;
c6  = 27.9;
c7  = 198;
c8  = 2.36;
c9  = 5.74;
c10 = 11.35;
c11 = 16.1;
c12 = 201;

%% lamdbda_i for analytical Cq Map (lambda > 0)
lambda_i = (1./(lambda+0.08.*beta)) - (0.035./(c11+c12.*beta.^3));

%% Derivatives for derivatives with respect to pitch of function 2 & 3
dlambda_i_la = -1./(lambda+0.08.*beta).^2;
dlambda_i_b = -(0.08./(lambda + 0.08 .* beta).^2) + ((3 * 0.035 * c12 .* beta.^2)./(c11 + c12 .* beta.^3).^2);

%% Auxiliary functions for chain rule
f1 = c4./lambda;
f2 = c5.*lambda_i-c6.*beta-(c7.*beta.^c8)-c9;
f3 = exp(-c10.*lambda_i);

%% Derivative of aux. functions with respect to lamdba (Schnelllaufzahl)
df1_la = -c4./(lambda.^2);
df2_la = c5.*dlambda_i_la;
df3_la = -c10.*exp(-c10.*lambda_i).*(dlambda_i_la);

%% Derivative of aux. functions with respect to beta (Pitchangle)
df1_b = 0;
df2_b = (c5.*dlambda_i_b) - c6 -(c7*c8.*beta.^(c8-1));
df3_b = -c10.*exp(-c10.*lambda_i).*dlambda_i_b;

%% Results
Cq = c1.*(1+c2.*(beta+c3).^(-1/2))+f1.*f2.*f3;   % Momentenbeiwert
dCq_lambda = (df1_la.*f2.*f3) + (f1.*df2_la.*f3) + (f1.*f2.*df3_la);     % Derivative with respect to Schnelllaufzahl
dCq_beta = (df1_b.*f2.*f3)+ (f1.*df2_b.*f3) + (f1.*f2.*df3_b) + (0.5*c1*c2).*((beta+c3).^(-1/2));

%Cq = c1.*(1+c2.*(beta+c3).^(-1/2))+f1.*f2.*f3;   % Momentenbeiwert
%dCq_lambda = (df1_la.*f2.*f3) + (f1.*df2_la.*f3) + (f1.*f2.*df3_la);     % Derivative with respect to Schnelllaufzahl
%dCq_beta = (df1_b.*f2.*f3)+ (f1.*df2_b.*f3) + (f1.*f2.*df3_b) + (0.5*c1*c2).*((beta+c3).^(-1/2));  % Derivative with respect to pitchangle

%% Calculation of constant coefficients from linear Systems
komega = (pi .* p.R^4 .* dCq_lambda * p.rho .* p.v_e)./2;
kbeta = pi .* p.R^3 .* dCq_beta * p.rho .* p.v_e.^2.*0.5;

%% Control Parameters with reference time constant tau
J = p.Jr + p.ngear^2 * p.Jg;
a = -komega./J;
b =  kbeta./J;
Kp = 1./(b.*tau);
Ki = a./(b.*tau);

end