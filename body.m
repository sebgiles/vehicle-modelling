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

% time derivatives on lagrange coordinates
Dy    = diff(y);
Dp    = diff(p);
Dr    = diff(r);
Dx_CG = diff(x_CG);
Dy_CG = diff(y_CG);
Dz_CG = diff(z_CG);


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

% ----- DYNAMIC PARAMETERS ------------------------------------------------
% gravitational accelearation
syms g
% vehicle mass
syms m  

% vehicle inertia tensor wrt body frame
syms Ixx Iyy Izz Ixz
I = [Ixx 0   Ixz;
     0   Iyy 0  ;
     Ixz 0   Izz ];

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
w_skew = diff(R,t)*(R.');
w_skew = w_skew(t);
% angular velocity of body frame wrt inertial
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
D(t) = D;
    
% Lagrangian
L = T - V;

% wheel contact points wrt inertial frame
p_fr = p_fr(t);
p_fl = p_fl(t);
p_rr = p_rr(t);
p_rl = p_rl(t);

w_fr = [ p_fr(1:2); 0];
w_fl = [ p_fl(1:2); 0];
w_rr = [ p_rr(1:2); 0];
w_rl = [ p_rl(1:2); 0];

p_fr(t) = p_fr;
p_fl(t) = p_fl;
p_rr(t) = p_rr;
p_rl(t) = p_rl;

w_fr(t) = w_fr;
w_fl(t) = w_fl;
w_rr(t) = w_rr;
w_rl(t) = w_rl;

% wheel contact points wrt body frame
W_fr = R\w_fr - p_CG;
W_fl = R\w_fl - p_CG;
W_rr = R\w_rr - p_CG;
W_rl = R\w_rl - p_CG;

% wheel contact point velocities wrt inertial frame
Dw_fr = diff(w_fr,t);
Dw_fl = diff(w_fl,t);
Dw_rr = diff(w_rr,t);
Dw_rl = diff(w_rl,t);

% lateral speed at contact points wrt inertial frame
Dw_lfr = (Rz\Dw_fr).'*[0;1;0];
Dw_lfl = (Rz\Dw_fl).'*[0;1;0];
Dw_lrr = (Rz\Dw_rr).'*[0;1;0];
Dw_lrl = (Rz\Dw_rl).'*[0;1;0];

%{
% wheel vertical forces for friction calculation
Z_fr = - k_f * d_fr;
Z_fl = - k_f * d_fl;
Z_rr = - k_r * d_rr;
Z_rl = - k_r * d_rl;

% dynamic friction coefficient
syms mu

% planar forces at contact points wrt inertial frame
f_fr = -mu*Z_fr*sign(Dw_lfr)*[sin(y);-cos(y);0];
f_fl = -mu*Z_fl*sign(Dw_lfl)*[sin(y);-cos(y);0];
f_rr = -mu*Z_rr*sign(Dw_lrr)*[sin(y);-cos(y);0];
f_rl = -mu*Z_rl*sign(Dw_lrl)*[sin(y);-cos(y);0];

%}

% planar forces at contact points wrt undercarriage frame
f_fr = [0;0;0];
f_fl = [0;0;0];
f_rr = [800;0;0];
f_rl = [800;0;0];

% planar forces at contact points wrt inertial frame
f_fr = [sin(y);-cos(y);0];
f_fl = [sin(y);-cos(y);0];
f_rr = [sin(y);-cos(y);0];
f_rl = [sin(y);-cos(y);0];

% planar forces at contact points wrt body frame
F_fr = R\f_fr;
F_fl = R\f_fl;
F_rr = R\f_rr;
F_rl = R\f_rl;

% generalized external forces
q = q(t);
Q = q; % just for init with right type and size
for i = 1:n_DOF
    Q(i) =        F_fr.' * fdiff(W_fr, q(i),t);
    Q(i) = Q(i) + F_fl.' * fdiff(W_fl, q(i),t);
    Q(i) = Q(i) + F_rr.' * fdiff(W_rr, q(i),t);
    Q(i) = Q(i) + F_rl.' * fdiff(W_rl, q(i),t);
end
q(t)=q;
Q = simplify(Q);
L_int = lagint(t, q, L, D);

% Lagrange Equations: dt(ddq_k(L)) - dq_k(L) + ddq_k(D) = Q_k

E = L_int == 0;


% make substitutions to transform into first order equations
[V,S] = odeToVectorField(E);
