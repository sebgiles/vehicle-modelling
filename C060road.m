% vertical projection to road matrix 
P = [1 0 0; 
     0 1 0; 
     0 0 0];
 
% wheel contact points are obtained by projecting suspension attachment
% points on the road surface
w_fr = P*p_fr;
w_fl = P*p_fl;
w_rr = P*p_rr;
w_rl = P*p_rl;

% wheel contact point velocities wrt inertial frame
Dw_fr = subs(diff(w_fr, t), diff(q(t),t), Dq(t));
Dw_fl = subs(diff(w_fl, t), diff(q(t),t), Dq(t));
Dw_rr = subs(diff(w_rr, t), diff(q(t),t), Dq(t));
Dw_rl = subs(diff(w_rl, t), diff(q(t),t), Dq(t));

% contact point velocities wrt corresponding wheel frames
v_ufr = R_steer\(Rz\Dw_fr);
v_ufl = R_steer\(Rz\Dw_fl);
v_urr = Rz\Dw_rr;
v_url = Rz\Dw_rl;

% wheel loads for friction calculations (obtained as suspension forces)
FZ_fr = - k_f * d_fr - b_f * Dd_fr;
FZ_fl = - k_f * d_fl - b_f * Dd_fl;
FZ_rr = - k_r * d_rr - b_r * Dd_rr;
FZ_rl = - k_r * d_rl - b_r * Dd_rl;

% planar forces at contact points wrt inertial frame
f_fr = Rz*R_steer*[FX_fr; FY_fr; 0];
f_fl = Rz*R_steer*[FX_fl; FY_fl; 0];
f_rr = Rz*        [FX_rr; FY_rr; 0];
f_rl = Rz*        [FX_rl; FY_rl; 0];

