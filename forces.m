% generalized external forces
Q = sym('Q',[6,1]);

% ----- compute wheel contact points wrt inertial frame ----------
w_fr = [ getel(p_fr, 1:2, t); 0];
w_fl = [ getel(p_fl, 1:2, t); 0];
w_rr = [ getel(p_rr, 1:2, t); 0];
w_rl = [ getel(p_rl, 1:2, t); 0];

% ----- compute wheel contact points wrt body frame --------------
W_fr = R\w_fr - p_CG;
W_fl = R\w_fl - p_CG;
W_rr = R\w_rr - p_CG;
W_rl = R\w_rl - p_CG;

% wheel vertical forces for friction calculation
Z_fr = - k_f * d_fr;
Z_fl = - k_f * d_fl;
Z_rr = - k_r * d_rr;
Z_rl = - k_r * d_rl;

% dynamic friction coefficient
syms mu

% planar forces at contact points wrt undercarriage frame
f_ufr = [0;0;0];
f_ufl = [0;0;0];
f_urr = [800;0;0];
f_url = [800;0;0];

% planar forces at contact points wrt inertial frame
f_fr = Rz*f_ufr;
f_fl = Rz*f_ufl;
f_rr = Rz*f_urr;
f_rl = Rz*f_url;

% planar forces at contact points wrt body frame
F_fr = R\f_fr;
F_fl = R\f_fl;
F_rr = R\f_rr;
F_rl = R\f_rl;

% generalized external forces
for i = 1:n_DOF
    Q(i) =        F_fr.' * fdiff(W_fr, getel(q,i,t),t);
    Q(i) = Q(i) + F_fl.' * fdiff(W_fl, getel(q,i,t),t);
    Q(i) = Q(i) + F_rr.' * fdiff(W_rr, getel(q,i,t),t);
    Q(i) = Q(i) + F_rl.' * fdiff(W_rl, getel(q,i,t),t);
end