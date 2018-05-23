% wheel angular velocities
N_fr = 38.5;
N_fl = 38.5;
N_rr = 38.5;
N_rl = 38.5;

% steer angle (no ackermann geometry for now)
a_s = 0;
% passive rotation matrix from undercarriage frame to front wheel frame
R_steer = [ cos(a_s) -sin(a_s)  0
            sin(a_s)  cos(a_s)  0
            0         0         1 ];

% wheel vertical forces for friction calculation
FZ_fr = - k_f * d_fr + b_f * diff(d_fr,t);
FZ_fl = - k_f * d_fl + b_f * diff(d_fl,t);
FZ_rr = - k_r * d_rr + b_r * diff(d_rr,t);
FZ_rl = - k_r * d_rl + b_r * diff(d_rl,t);

% wheel contact point velocities wrt inertial frame
Dw_fr = diff(w_fr, t);
Dw_fl = diff(w_fl, t);
Dw_rr = diff(w_rr, t);
Dw_rl = diff(w_rl, t);

% ground velocities wrt corresponding wheel frames
v_ufr = R_steer\Rz\Dw_fr;
v_ufl = R_steer\Rz\Dw_fl;
v_urr = Rz\Dw_rr;
v_url = Rz\Dw_rl;

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

RE_fr = v_uxfr/N_fr;
RE_fl = v_uxfl/N_fl;
RE_rr = v_uxrr/N_rr;
RE_rl = v_uxrl/N_rl;

R0_fr = 0.26;
R0_fl = 0.26;
R0_rr = 0.26;
R0_rl = 0.26;

SL_fr = R0_fr/RE_fr-1;
SL_fl = R0_fl/RE_fl-1;
SL_rr = R0_rr/RE_rr-1;
SL_rl = R0_rl/RE_rl-1;

% slip angle (angle between wheel velocities and undercarriage x axis)
SA_fr = atan(-v_uyfr/v_uxfr);
SA_fl = atan(-v_uyfl/v_uxfl);
SA_rr = atan(-v_uyrr/v_uxrr);
SA_rl = atan(-v_uyrl/v_uxrl);

tyreID = 'Hoosier FSAE 20.5x7.0-13 43129 @ 12 psi, 7 inch rim';
[FX_fr, FY_fr, ~, ~] = PAC96(SL_fr, SA_fr, 0, FZ_fr,tyreID);
[FX_fl, FY_fl, ~, ~] = PAC96(SL_fl, SA_fl, 0, FZ_fl,tyreID);
[FX_rr, FY_rr, ~, ~] = PAC96(SL_rr, SA_rr, 0, FZ_rr,tyreID);
[FX_rl, FY_rl, ~, ~] = PAC96(SL_rl, SA_rl, 0, FZ_rl,tyreID);

% planar forces at contact points wrt undercarriage frame
%f_ufr = R_steer*[FX_fr; FY_fr; 0];
%f_ufl = R_steer*[FX_fl; FY_fl; 0];
%f_urr =         [FX_rr; FY_rr; 0];
%f_url =         [FX_rl; FY_rl; 0];
f_ufr = R_steer*[FX_fr; 0; 0];
f_ufl = R_steer*[FX_fl; 0; 0];
f_urr =         [FX_rr; 0; 0];
f_url =         [FX_rl; 0; 0];
