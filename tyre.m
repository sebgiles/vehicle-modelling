
load('tire data/B1175run36.mat')
N = N/60*2*pi;
RE = RE/100;
RL = RL/100;
V = V/3.6;
SA = SA/180*pi;
Vx = V .* cos(SA);
IA = IA/180*pi;
R0 = Vx.*(SL+1)./N;
dN = [0; diff(N)];
dSA = [0; diff(SA)];
tyreID = 'Hoosier FSAE 20.5x7.0-13 43129 @ 12 psi, 7 inch rim';

[F_x,  F_y,  F_x0,  F_y0]  = PAC96(SR, SA, IA, -FZ, tyreID);
[F_xL, F_yL, F_x0L, F_y0L] = PAC96(SL, SA, IA, -FZ, tyreID);

F_y  = -F_y;
F_yL = -F_yL;
F_y0 = -F_y0;
F_y0L= -F_y0L;

plot(FY)
hold on
plot(F_y)
%plot(F_y0)
plot(F_yL)
%plot(F_y0L)
legend('FY','F_y','F_yL');
%legend('FY','F_y','F_yL','F_y0L');
