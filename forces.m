friction
% generalized external forces
Q = sym('Q',[6,1]);

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
    Q(i) =        f_fr.' * fdiff(w_fr, getel(q,i,t),t);
    Q(i) = Q(i) + f_fl.' * fdiff(w_fl, getel(q,i,t),t);
    Q(i) = Q(i) + f_rr.' * fdiff(w_rr, getel(q,i,t),t);
    Q(i) = Q(i) + f_rl.' * fdiff(w_rl, getel(q,i,t),t);
end

% Lagrange Equations: dt(ddq_k(L)) - dq_k(L) + ddq_k(D) = Q_k
E = L_int == Q;

% make substitutions to transform into first order equations
[V,S] = odeToVectorField(E);