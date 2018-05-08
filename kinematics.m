%-------- KINEMATICS PARAMETERS -------------------------------------------
% track widths
syms T_f T_r    

% axle positions wrt body frame "0=CG, (x,y,z) -> (forward,right,down)"
syms l_f l_r                       
syms Z_f Z_r    

% CG no load height (negative)
syms h_CG       

% suspension position at no load (angles are all assumed to be 0)
h_f = Z_f - h_CG ;
h_r = Z_r - h_CG ;

% Suspension attachment point coordinates wrt body frame
P_fr = [ l_f  T_f/2 Z_f].';
P_fl = [ l_f -T_f/2 Z_f].';
P_rr = [-l_r  T_r/2 Z_r].';
P_rl = [-l_r -T_r/2 Z_r].';

% time variable for differentiation
syms t

% -------- LAGRANGIAN COORDINATES -----------------------------------------
% CG position wrt inertial frame
syms x_CG(t) y_CG(t) z_CG(t)     
p_CG = [x_CG y_CG z_CG].';
% body frame orientation angles (ZYX Euler - YPR - Tait Bryan)
syms y(t) p(t) r(t)     

q = [y p r x_CG y_CG z_CG].';
n_DOF = 6;

% -------------------------------------------------------------------------

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
% it can be used to rotate reference frame from body to inertial
R = Rz*Ry*Rx ; 
    
% Suspension attachment point coordinates wrt inertial (rotate and translate)
p_fr = p_CG + R*P_fr ;
p_fl = p_CG + R*P_fl ;
p_rl = p_CG + R*P_rl ;
p_rr = p_CG + R*P_rr ;

p_fr = p_fr(t);
p_fl = p_fl(t);
p_rr = p_rr(t);
p_rl = p_rl(t);

% suspension travel 
d_fr = -p_fr(3)+h_f;
d_fl = -p_fl(3)+h_f;
d_rr = -p_rr(3)+h_r;
d_rl = -p_rl(3)+h_r;

p_fr(t) = p_fr;
p_fl(t) = p_fl;
p_rr(t) = p_rr;
p_rl(t) = p_rl;
d_fr(t) = d_fr;
d_fl(t) = d_fl;
d_rr(t) = d_rr;
d_rl(t) = d_rl;

%  wheel contact points wrt inertial frame 
w_fr = [ getel(p_fr, 1:2, t); 0];
w_fl = [ getel(p_fl, 1:2, t); 0];
w_rr = [ getel(p_rr, 1:2, t); 0];
w_rl = [ getel(p_rl, 1:2, t); 0];

%  wheel contact points wrt body frame 
W_fr = R\w_fr - p_CG;
W_fl = R\w_fl - p_CG;
W_rr = R\w_rr - p_CG;
W_rl = R\w_rl - p_CG;