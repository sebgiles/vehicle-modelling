syms FX_fr FY_fr FX_fl FY_fl FX_rr FY_rr FX_rl FY_rl steer
F = [FX_fr; FY_fr; FX_fl; FY_fl; FX_rr; FY_rr; FX_rl; FY_rl];

if linear
    R_steer = [     1     0  steer
                    0     1      0
                steer     0      1];
else
    R_steer = [ cos(steer) -sin(steer)  0
                sin(steer)  cos(steer)  0
                         0           0  1 ];
end

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

Q = C041genforces(f(t),p(t),q(t));
