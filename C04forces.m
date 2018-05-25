
syms FX_fr FY_fr FX_fl FY_fl FX_rr FY_rr FX_rl FY_rl

% planar forces at contact points wrt undercarriage
f_ufr = R_steer*[FX_fr; FY_fr; 0];
f_ufl = R_steer*[FX_fl; FY_fl; 0];
f_urr =         [FX_rr; FY_rr; 0];
f_url =         [FX_rl; FY_rl; 0];

% planar forces at contact points wrt inertial frame
f_fr = Rz*f_ufr;
f_fl = Rz*f_ufl;
f_rr = Rz*f_urr;
f_rl = Rz*f_url;

% all external applied forces
f = [f_fr f_fl f_rr f_rl];
% respective application points
p = [w_fr w_fl w_rr w_rl];

Q = C04_1genforces(f,p,q,t);