% dynamic friction coefficient
syms mu

% wheel rotation equivalent speed (input to dynamic system)
v_wfr = 10;
v_wfl = 10;
v_wrr = 10;
v_wrl = 10;

% wheel vertical forces for friction calculation
Z_fr = - k_f * d_fr;
Z_fl = - k_f * d_fl;
Z_rr = - k_r * d_rr;
Z_rl = - k_r * d_rl;

% wheel contact point velocities wrt inertial frame
DW_fr = diff(W_fr, t);
DW_fl = diff(W_fl, t);
DW_rr = diff(W_rr, t);
DW_rl = diff(W_rl, t);

% wheel contact point velocities speed wrt undercarriage frame
v_ufr = Rz\DW_fr;
v_ufl = Rz\DW_fl;
v_urr = Rz\DW_rr;
v_url = Rz\DW_rl;

% wheel contact point lateral speeds
v_lfr = getel(v_ufr,2,t);
v_lfl = getel(v_ufl,2,t);
v_lrr = getel(v_urr,2,t);
v_lrl = getel(v_url,2,t);

% slip angle
%a = 

%s_lfr = 

mu_fr = 1;
mu_fl = 1; 
mu_rr = 1;
mu_rl = 1;

% planar forces at contact points wrt undercarriage frame
f_ufr = [0; - mu_fr*Z_fr;0];
f_ufl = [0; - mu_fl*Z_fl;0];
f_urr = [0; - mu_rr*Z_rr;0];
f_url = [0; - mu_rl*Z_rl;0];

% planar forces at contact points wrt undercarriage frame
f_ufr = [0; 800; 0];
f_ufl = [0; 800; 0];
f_urr = [0; 800; 0];
f_url = [0; 800; 0];