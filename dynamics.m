% ----- DYNAMIC PARAMETERS ------------------------------------------------
% gravitational acceleration
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

% time derivatives on lagrange coordinates
Dy    = diff(y);
Dp    = diff(p);
Dr    = diff(r);
Dx_CG = diff(x_CG);
Dy_CG = diff(y_CG);
Dz_CG = diff(z_CG);

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

% % Alternative less general method from same source:
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
% Total potential energy
V = V_spring + V_g;

% damper-dissipated energy (Rayleigh dissipation function)
D =     + 1/2 * b_f * diff(d_fr,t)^2 ...
        + 1/2 * b_f * diff(d_fl,t)^2 ...
        + 1/2 * b_r * diff(d_rr,t)^2 ...
        + 1/2 * b_r * diff(d_rl,t)^2;
D(t) = D;
    
% Lagrangian
L = T - V;

% Lagrange Equations: dt(ddq_k(L)) - dq_k(L) + ddq_k(D) = Q_k
% the following are the left hand sides of the lagrange equations as shown
% above
L_int = lagint(t, q, L, D);
