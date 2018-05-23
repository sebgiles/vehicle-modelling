tic
% wheel angular velocities
N_fr = 38.5;
N_fl = 38.5;
N_rr = 38.5;
N_rl = 38.5;

% steer angle (no ackermann geometry for now)
a_s = 0;

% passive rotation matrix from undercarriage frame to front wheel frames
R_steer = [ cos(a_s) -sin(a_s)  0
            sin(a_s)  cos(a_s)  0
            0         0         1 ];
toc
% wheel vertical forces for friction calculation
FZ_fr = - k_f * d_fr - b_f * Dd_fr;
FZ_fl = - k_f * d_fl - b_f * Dd_fl;
FZ_rr = - k_r * d_rr - b_r * Dd_rr;
FZ_rl = - k_r * d_rl - b_r * Dd_rl;

% wheel contact point velocities wrt inertial frame
Dw_fr = subs(diff(w_fr, t), diff(q(t),t), Dq(t));
Dw_fl = subs(diff(w_fl, t), diff(q(t),t), Dq(t));
Dw_rr = subs(diff(w_rr, t), diff(q(t),t), Dq(t));
Dw_rl = subs(diff(w_rl, t), diff(q(t),t), Dq(t));

toc
% ground velocities wrt corresponding wheel frames
v_ufr = R_steer\Rz\Dw_fr;
v_ufl = R_steer\Rz\Dw_fl;
v_urr = Rz\Dw_rr;
v_url = Rz\Dw_rl;

% wheel contact point lateral speeds
v_uyfr = [0 1 0]*v_ufr;
v_uyfl = [0 1 0]*v_ufl;
v_uyrr = [0 1 0]*v_urr;
v_uyrl = [0 1 0]*v_url;

% wheel contact point longitudinal speeds
v_uxfr = [1 0 0]*v_ufr;
v_uxfl = [1 0 0]*v_ufl;
v_uxrr = [1 0 0]*v_urr;
v_uxrl = [1 0 0]*v_url;

% effective radius - defined as the radius of the wheel that would not slip for
% given omega (N) and forward velocity (v_ux)
RE_fr = v_uxfr/N_fr;
RE_fl = v_uxfl/N_fl;
RE_rr = v_uxrr/N_rr;
RE_rl = v_uxrl/N_rl;

% free rolling radius - defined as the radius of the wheel that is not exerting
% planar forces for given load and omega
R0_fr = 0.26;
R0_fl = 0.26;
R0_rr = 0.26;
R0_rl = 0.26;

% slip ratio - defined as ratio between contact point velocity and wheel
% velocity
SL_fr = R0_fr/RE_fr-1;
SL_fl = R0_fl/RE_fl-1;
SL_rr = R0_rr/RE_rr-1;
SL_rl = R0_rl/RE_rl-1;

% slip angle (angle between wheel velocities and wheel x axis)
SA_fr = atan(-v_uyfr/v_uxfr);
SA_fl = atan(-v_uyfl/v_uxfl);
SA_rr = atan(-v_uyrr/v_uxrr);
SA_rl = atan(-v_uyrl/v_uxrl);
toc
tyreID = 'Hoosier FSAE 20.5x7.0-13 43129 @ 12 psi, 7 inch rim';
[FX_fr, FY_fr, ~, ~] = PAC96(SL_fr, SA_fr, 0, FZ_fr, tyreID);
[FX_fl, FY_fl, ~, ~] = PAC96(SL_fl, SA_fl, 0, FZ_fl, tyreID);
[FX_rr, FY_rr, ~, ~] = PAC96(SL_rr, SA_rr, 0, FZ_rr, tyreID);
[FX_rl, FY_rl, ~, ~] = PAC96(SL_rl, SA_rl, 0, FZ_rl, tyreID);
toc
% planar forces at contact points wrt undercarriage
f_ufr = R_steer*[FX_fr; FY_fr; 0];
f_ufl = R_steer*[FX_fl; FY_fl; 0];
f_urr =         [FX_rr; FY_rr; 0];
f_url =         [FX_rl; FY_rl; 0];
%f_ufr = R_steer*[FX_fr; 0; 0];
%f_ufl = R_steer*[FX_fl; 0; 0];
%f_urr =         [FX_rr; 0; 0];
%f_url =         [FX_rl; 0; 0];
toc
