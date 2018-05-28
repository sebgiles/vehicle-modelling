% spring length at no load
h_f = Z_f - h_CG ;
h_r = Z_r - h_CG ;

% Suspension attachment point coordinates wrt body frame
P_fr = [ l_f;  T_f/2; Z_f];
P_fl = [ l_f; -T_f/2; Z_f];
P_rr = [-l_r;  T_r/2; Z_r];
P_rl = [-l_r; -T_r/2; Z_r];

% Suspension attachment point coordinates wrt inertial frame
p_fr = p_CG + R*P_fr ;
p_fl = p_CG + R*P_fl ;
p_rl = p_CG + R*P_rl ;
p_rr = p_CG + R*P_rr ;

% suspension travel
d_fr = -[0 0 1]*p_fr + h_f;
d_fl = -[0 0 1]*p_fl + h_f;
d_rr = -[0 0 1]*p_rr + h_r;
d_rl = -[0 0 1]*p_rl + h_r;

% suspension velocities
Dd_fr = subs(diff(d_fr,t), diff(q(t)), Dq(t));
Dd_fl = subs(diff(d_fl,t), diff(q(t)), Dq(t));
Dd_rr = subs(diff(d_rr,t), diff(q(t)), Dq(t));
Dd_rl = subs(diff(d_rl,t), diff(q(t)), Dq(t));
