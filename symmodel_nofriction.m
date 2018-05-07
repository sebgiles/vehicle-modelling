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

% time derivatives on lagrange coordinates
Dy    =diff(y);
Dp    =diff(p);
Dr    =diff(r);
Dx_CG =diff(x_CG);
Dy_CG =diff(y_CG);
Dz_CG =diff(z_CG);


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
    
% Suspension attachment point coordinates wrt road (rotate and translate)
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

% ----- DYNAMIC PARAMETERS ------------------------------------------------
% gravitational accelearation
syms g
% vehicle mass
syms m  
% vehicle inertia tensor wrt body frame
I = sym('I', [3 3]);

% y axis is principle rotation axis (assuming left/right simmetry)
I(1,2)=0;
I(3,2)=0;
% and because the inertia tensor must always be simmetric
I(2,1)=0;
I(2,3)=0;

% spring coefficients 
syms k_f k_r

% damper coefficients
syms b_f b_r

%--------------------------------------------------------------------------
% calculation of translational kinetic energy
v_CG = diff(p_CG,t);
T_trans = 1/2*m*(v_CG.')*v_CG;

% calculation of angular velocity vector
% formula for the first step found on "robot dynamics" lecture notes (ETHZ)
% angular velocity of body frame wrt inertial
w_skew = diff(R,t)*(R.');
w_skew = w_skew(t);
w = [w_skew(3,2); w_skew(1,3); w_skew(2,1)];
% angular velocity of body frame wrt body frame
W = R\w;

% Alternative less general method from same source:
% E = [ 0  -sin(y)  cos(p)*cos(y)
%       0   cos(y)  cos(p)*sin(y)
%       1        0    -sin(pitch) ];
% w = E * diff([y,p,r],t);
% W = R\w;

% rotational kinetic energy
T_rot = W.'*I*W ;

% total kinetic energy
T = T_rot + T_trans;

% energy stored in suspension springs
V_spring = 1/2 * k_f * (d_fr^2 + d_fl^2) + 1/2 * k_r * (d_rl^2 + d_rr^2);

% gravitational potential energy
V_g = -m*g*z_CG;
V = V_spring + V_g;

% damper-dissipated energy (Rayleigh dissipation function)
D =     + 1/2 * b_f * diff(d_fr,t)^2 ...
        + 1/2 * b_f * diff(d_fl,t)^2 ...
        + 1/2 * b_r * diff(d_rr,t)^2 ...
        + 1/2 * b_r * diff(d_rl,t)^2;

% Lagrangian
L = T - V;

% Lagrange Equations
% dt(ddq_k(L)) - dq_k(L) + ddq_k(D) = Q_k

q = [y p r x_CG y_CG z_CG].';
L_int = lagint(t, q, L, D);

Q = zeros(length(q),1);
E = L_int == Q;

[V,S] = odeToVectorField(E);


