linearized = false;

%% time
syms t

%% LAGRANGIAN COORDINATES 
% body orientation angles & derivatives (ZYX Euler - YPR - Tait Bryan) 
syms y(t)   p(t)   r(t)
syms Dy(t)  Dp(t)  Dr(t)
syms DDy  DDp  DDr

% CG position wrt inertial frame & derivatives
syms x_CG(t)  y_CG(t)  z_CG(t)
syms Dx_CG(t) Dy_CG(t) Dz_CG(t)
syms DDx_CG DDy_CG DDz_CG

% various handy vector combinations of the above 
q  = [ y;  p;  r;  x_CG;  y_CG;  z_CG];
Dq = [Dy; Dp; Dr; Dx_CG; Dy_CG; Dz_CG];
DDq = [DDy; DDp; DDr; DDx_CG; DDy_CG; DDz_CG];

% position of CG wrt inertial frame
p_CG = [x_CG; y_CG; z_CG];

% velocity of CG wrt inertial frame
v_CG = [Dx_CG; Dy_CG; Dz_CG];

%% Successive ROTATION MATRICES from Inertial frame to body frame
% yaw rotation matrix
Rz = [ cos(y)  -sin(y)        0
       sin(y)   cos(y)        0
            0        0        1 ];
if linearized
  % linearized roll rotation matrix
  Ry = [ 1 0 p
         0 1 0
         p 0 1 ];
  % linearized roll rotation matrix
  Rx = [ 1 0 0
         0 1 r
         0 r 1 ];
else
  % pitch rotation matrix
  Ry = [ cos(p)       0  sin(p)
              0       1       0
        -sin(p)       0  cos(p) ];
  % roll rotation matrix
  Rx = [      1       0      0
              0  cos(r) -sin(r)
              0  sin(r)  cos(r) ];
end

%% Full rotation matrix from Inertial frame to body frame
% this matrix carries all information about vehicle attitude,
% it can be used to rotate vectors in body to inertial frame
R = Rz*Ry*Rx ;

%% Rotation Matrix from undercarriage frame to wheel frame
if linearized
    R_steer = [     1 -steer      0
                steer      1      0
                    0      0      1];
else
    R_steer = [ cos(steer) -sin(steer)  0
                sin(steer)  cos(steer)  0
                         0           0  1 ];
end