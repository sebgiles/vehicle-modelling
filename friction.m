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

% ground velocities wrt undercarriage frame
v_ufr = Rz\DW_fr;
v_ufl = Rz\DW_fl;
v_urr = Rz\DW_rr;
v_url = Rz\DW_rl;

% wheel contact point lateral speeds
v_uyfr = getel(v_ufr,2,t);
v_uyfl = getel(v_ufl,2,t);
v_uyrr = getel(v_urr,2,t);
v_uyrl = getel(v_url,2,t);
% wheel contact point longitudinal speeds
v_uxfr = getel(v_ufr,1,t);
v_uxfl = getel(v_ufl,1,t);
v_uxrr = getel(v_urr,1,t);
v_uxrl = getel(v_url,1,t);

% slip angle (angle between wheel velocities and undercarriage x axis)
a_fr = atan(v_uyfr/v_uxfr);
a_fr = atan(v_uyfl/v_uxfl);
a_fr = atan(v_uyrr/v_uxrr);
a_fr = atan(v_uyrl/v_uxrl);

%s_lfr = 

mu_Sfr = 0;
mu_Sfl = 0;
mu_Srr = 0;
mu_Srl = 0;


mu_Lfr = mu_Rfr*s_Lfr



% planar forces at contact points wrt undercarriage frame
f_ufr = [mu_Lfr*Z_fr; mu_Sfr*Z_fr;0];
f_ufl = [mu_Lfl*Z_fl; mu_Sfl*Z_fl;0];
f_urr = [mu_Lrr*Z_rr; mu_Srr*Z_rr;0];
f_url = [mu_Lrl*Z_rl; mu_Srl*Z_rl;0];