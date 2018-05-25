% suspension position at no load
h_f = Z_f - h_CG ;
h_r = Z_r - h_CG ;

% Suspension attachment point coordinates wrt body frame
P_fr = [ l_f  T_f/2 Z_f].';
P_fl = [ l_f -T_f/2 Z_f].';
P_rr = [-l_r  T_r/2 Z_r].';
P_rl = [-l_r -T_r/2 Z_r].';

% steer angle (no ackermann geometry for now)
a_s = 0; % TODO: steering

% passive rotation matrix from undercarriage frame to front wheel frames
if linear
    R_steer = [  1  0 a_s
                 0  1   0
               a_s  0   1];
else
    R_steer = [ cos(a_s) -sin(a_s)  0
                sin(a_s)  cos(a_s)  0
                       0         0  1 ];
end

% time
syms t

% -------- LAGRANGIAN COORDINATES ------------------------------
% body frame orientation angles (ZYX Euler - YPR - Tait Bryan)
syms y(t)   p(t)   r(t)
syms Dy(t)  Dp(t)  Dr(t)
syms DDy  DDp  DDr
% CG position wrt inertial frame
syms x_CG(t)  y_CG(t)  z_CG(t)
syms Dx_CG(t) Dy_CG(t) Dz_CG(t)
syms DDx_CG DDy_CG DDz_CG

q  = [ y;  p;  r;  x_CG;  y_CG;  z_CG];
diffq = diff(q(t));
Dq = [Dy; Dp; Dr; Dx_CG; Dy_CG; Dz_CG];
DDq = [DDy; DDp; DDr; DDx_CG; DDy_CG; DDz_CG];

p_CG = [x_CG; y_CG; z_CG];

% velocity of CG wrt inertial frame
v_CG = [Dx_CG; Dy_CG; Dz_CG];
% --------------------------------------------------------------

% successive rotation matrices from Inertial frame to body frame
Rz = [ cos(y)  -sin(y)        0
       sin(y)   cos(y)        0
            0        0        1 ];
if linear
  Ry = [ 1 0 p
         0 1 0
         p 0 1 ];

  Rx = [ 1 0 0
         0 1 r
         0 r 1 ];

else

  Ry = [ cos(p)       0  sin(p)
              0       1       0
        -sin(p)       0  cos(p) ];

  Rx = [      1       0      0
              0  cos(r) -sin(r)
              0  sin(r)  cos(r) ];

end

% this matrix carries all information about vehicle attitude,
% it can be used to rotate vectors in body to inertial frame
R = Rz*Ry*Rx ;

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

% wheel contact points wrt inertial frame
w_fr = [1 0 0; 0 1 0; 0 0 0]*p_fr;
w_fl = [1 0 0; 0 1 0; 0 0 0]*p_fl;
w_rr = [1 0 0; 0 1 0; 0 0 0]*p_rr;
w_rl = [1 0 0; 0 1 0; 0 0 0]*p_rl;

% Rotational velocity vector wrt inertial frame
if linear
  E = [ 0  -sin(y)  p*cos(y)
        0   cos(y)  p*sin(y)
        1        0        -p ];
else
  E = [ 0 -sin(y)  cos(p)*cos(y)
        0  cos(y)  cos(p)*sin(y)
        1       0        -sin(p) ];
end

w = E * [Dy; Dp; Dr];
% Rotational velocity vector wrt body frame
W = R\w;
