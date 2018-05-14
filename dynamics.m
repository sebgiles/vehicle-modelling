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

% translational kinetic energy
T_trans = 1/2*m*(v_CG.')*v_CG;

% rotational kinetic energy
T_rot = 1/2*W.'*I*W;

% total kinetic energy
T = T_rot + T_trans;

% energy stored in suspension springs
V_spring = 1/2 * k_f * (d_fr^2 + d_fl^2) ...
         + 1/2 * k_r * (d_rl^2 + d_rr^2);
% gravitational potential energy
V_g = -m*g*z_CG;
% Total potential energy
V = V_spring + V_g;

% damper-dissipated energy (Rayleigh dissipation function)
D = 1/2 * b_f * diff(d_fr,t)^2 ...
  + 1/2 * b_f * diff(d_fl,t)^2 ...
  + 1/2 * b_r * diff(d_rr,t)^2 ...
  + 1/2 * b_r * diff(d_rl,t)^2;
    
% Lagrangian
L = simplify(T - V);

% Lagrange Equations: dt(d(dq_k)(L)) - dq_k(L) + ddq_k(D) = Q_k
% the following are the left hand sides of the lagrange equations as shown
% above
E_LHS = lagrange(t, q, L, D);
