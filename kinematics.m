%-------- KINEMATICS PARAMETERS --------------------------------
% track widths
syms T_f T_r    

% axle positions wrt body frame 
% "0=CG, (x,y,z) -> (forward,right,down)"
syms l_f l_r                       
syms Z_f Z_r    

% CG no load height (negative)
syms h_CG       

% suspension position at no load
h_f = Z_f - h_CG ;
h_r = Z_r - h_CG ;

% Suspension attachment point coordinates wrt body frame
P_fr = [ l_f  T_f/2 Z_f].';
P_fl = [ l_f -T_f/2 Z_f].';
P_rr = [-l_r  T_r/2 Z_r].';
P_rl = [-l_r -T_r/2 Z_r].';
%---------------------------------------------------------------

% time
syms t

% -------- LAGRANGIAN COORDINATES ------------------------------
% body frame orientation angles (ZYX Euler - YPR - Tait Bryan)
syms y(t) p(t) r(t)   
% CG position wrt inertial frame
syms x_CG(t) y_CG(t) z_CG(t)   
  

q    = [   y;    p;    r; x_CG; y_CG; z_CG];
p_CG = [x_CG; y_CG; z_CG];

% --------------------------------------------------------------

% successive rotation matrices from Inertial frame to body frame
Rz = [ cos(y)  -sin(y)        0
       sin(y)   cos(y)        0
            0        0        1 ];
     
Ry = [ cos(p)        0   sin(p)
            0        1        0
      -sin(p)        0   cos(p) ];
    
Rx = [      1        0       0
            0   cos(r)  -sin(r)
            0   sin(r)   cos(r) ];

% this matrix carries all information about vehicle attitude, 
% it can be used to rotate vectors in body to inertial frame
R = Rz*Ry*Rx ; 

% Suspension attachment point coordinates wrt inertial frame
p_fr = p_CG + R*P_fr ;
p_fl = p_CG + R*P_fl ;
p_rl = p_CG + R*P_rl ;
p_rr = p_CG + R*P_rr ;

% suspension travel 
d_fr = -[0 0 1]*p_fr+h_f;
d_fl = -[0 0 1]*p_fl+h_f;
d_rr = -[0 0 1]*p_rr+h_r;
d_rl = -[0 0 1]*p_rl+h_r;

% wheel contact points wrt inertial frame 
w_fr = p_fr.*[1;1;0];
w_fl = p_fl.*[1;1;0];
w_rr = p_rr.*[1;1;0];
w_rl = p_rl.*[1;1;0];

% velocity of CG wrt inertial frame
v_CG = diff(p_CG,t);


% Rotational velocity vector wrt inertial frame
E = [ 0  -sin(y)  cos(p)*cos(y)
      0   cos(y)  cos(p)*sin(y)
      1        0        -sin(p) ];
w = E * diff([y;p;r],t);

% Rotational velocity vector wrt body frame
W = R\w;
