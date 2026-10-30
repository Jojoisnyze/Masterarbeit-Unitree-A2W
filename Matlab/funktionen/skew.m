function S = skew(a)
% SKEW  Schiefsymmetrische Matrix [a], sodass skew(a)*x = cross(a,x).
% Quelle: Modern Robotics (Lynch & Park), Gl. (3.30)
S = [  0    -a(3)   a(2)
    a(3)    0    -a(1)
    -a(2)   a(1)    0  ];
end