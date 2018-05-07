%{
% wheel vertical forces for friction calculation
Z_fr = - k_f * d_fr;
Z_fl = - k_f * d_fl;
Z_rr = - k_r * d_rr;
Z_rl = - k_r * d_rl;

% dynamic friction coefficient
syms mu

% planar forces at contact points wrt inertial frame
f_fr = -mu*Z_fr*sign(Dw_lfr)*[sin(y);-cos(y);0];
f_fl = -mu*Z_fl*sign(Dw_lfl)*[sin(y);-cos(y);0];
f_rr = -mu*Z_rr*sign(Dw_lrr)*[sin(y);-cos(y);0];
f_rl = -mu*Z_rl*sign(Dw_lrl)*[sin(y);-cos(y);0];

%}