% dynamic friction coefficient
syms mu

% wheel rotation equivalent speed input
v_wfr = 10;
v_wfl = 10;
v_wrr = 10;
v_wrl = 10;

% steer angle (no ackermann geometry for now)
a_s = 0;
% passive rotation matrix from undercarriage frame to front wheel frame
R_steer = [ cos(a_s) -sin(a_s)  0
            sin(a_s)  cos(a_s)  0
            0         0         1 ];

% wheel vertical forces for friction calculation
Z_fr = - k_f * d_fr + b_f * diff(d_fr,t);
Z_fl = - k_f * d_fl + b_f * diff(d_fl,t);
Z_rr = - k_r * d_rr + b_r * diff(d_rr,t);
Z_rl = - k_r * d_rl + b_r * diff(d_rl,t);

% wheel contact point velocities wrt inertial frame
DW_fr = diff(W_fr, t);
DW_fl = diff(W_fl, t);
DW_rr = diff(W_rr, t);
DW_rl = diff(W_rl, t);

% ground velocities wrt corresponding wheel frames
v_ufr = R_steer\Rz\DW_fr;
v_ufl = R_steer\Rz\DW_fl;
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
a_fl = atan(v_uyfl/v_uxfl);
a_rr = atan(v_uyrr/v_uxrr);
a_rl = atan(v_uyrl/v_uxrl);

%s_lfr = 

mu_Sfr = 0;
mu_Sfl = 0;
mu_Srr = 0;
mu_Srl = 0;

mu_Lfr = 0; %mu_Rfr*s_Lfr
mu_Lfl = 0;
mu_Lrr = 0;
mu_Lrl = 0;




% planar forces at contact points wrt undercarriage frame
f_ufr = R_steer*[mu_Lfr*Z_fr; mu_Sfr*Z_fr;0];
f_ufl = R_steer*[mu_Lfl*Z_fl; mu_Sfl*Z_fl;0];
f_urr =         [mu_Lrr*Z_rr; mu_Srr*Z_rr;0];
f_url =         [mu_Lrl*Z_rl; mu_Srl*Z_rl;0];