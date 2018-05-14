friction

% planar forces at contact points wrt inertial frame
f_fr = Rz*f_ufr;
f_fl = Rz*f_ufl;
f_rr = Rz*f_urr;
f_rl = Rz*f_url;

% all external applied forces
f = [f_fr f_fl f_rr f_rl];
% respective application points
p = [w_fr w_fl w_rr w_rl];

Q = genforces(f,p,q,t);