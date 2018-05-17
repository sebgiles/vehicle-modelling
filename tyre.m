
load('tire data/B1175run37.mat')
N = N/60*2*pi;
RE = RE/100;
RL = RL/100;
V = V/3.6;
SA = SA/180*pi;
Vx = V .* cos(pi/180*SA);
IA = IA/180*pi;

[F_x, F_y, F_x0, F_y0] = PAC96(SR, SA, IA, -FZ,0);
[F_xL, F_yL, F_x0L, F_y0L] = PAC96(SL, SA, IA, -FZ,0);
