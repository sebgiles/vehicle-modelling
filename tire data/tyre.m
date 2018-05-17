
load('B1464run45.mat')
N = N/60*2*pi;
R0 = RE/100;
RL = RL/100;
V = V/3.6;
SA = SA/180*pi;
RE = R0./(1+R0.*SL);

Vx = V .* cos(pi/180*SA);
% slip based on R

myS = N./(Vx./R0) - 1;

% (slip based on RL) == SR
mySR = N./(Vx./RL) - 1;

S = ( (RE./RL) .* (1+SR) ) - 1;
%S = (N.*RE./Vx)-1;
