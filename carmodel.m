function [BodyDynamicsFunction, WheelLoadsFunction, ContactPointVelocitiesFunction] = carmodel(linearized)
if nargin < 1
    linearized = false;
end
%% Car parametrization
disp 'defining car parameters'
% gravitational acceleration
syms g
% track widths
syms t_f t_r
% Front / Rear Axle Distance from CG
syms l_f l_r
% Front / Rear Roll center height
syms q_f q_r
% CG ride height
syms h_CG
% Front / Rear Steer Inertia
syms I_f I_r
% Front Rear Right/Left mechanical trail (left indipendent just in case...)
syms ts_fr ts_fl ts_rr rs_rl
% Front / Rear virtual Spring coefficients
syms k_f k_r
% Front / Rear virtual Damper coefficients
syms b_f b_r
% sprung mass
syms m
% unsprung mass (including wheels)
syms m_u
% sprung masses moment of inertia tensor wrt body frame
syms Ixx Iyy Izz Ixz
% unsprung masses moment of inertia about vertical axis through CG
syms I_u
% Nominal Wheel Radius
syms r_0
% Wheel rotational inertia
syms I_w
% have to update / change this
params = [T_f; T_r; l_f; l_r; Z_f; Z_r; h_CG; k_f; k_r; b_f; b_r; g; m; Ixx; Iyy; Izz; Ixz];
%% Inputs Definition
% Front / Rear steer torques
syms M_sf M_sr
% planar wheel forces
syms FX_fr FY_fr FX_fl FY_fl FX_rr FY_rr FX_rl FY_rl
% self aligning wheel torques
syms MZ_fr MZ_fl MZ_rr MZ_rl
%% TIME
syms t
%% CAR STATE COORDINATES
% body orientation angles & derivatives (ZYX Euler - YPR - Tait Bryan)
syms   y(t)  p(t)  r(t)
syms  Dy(t) Dp(t) Dr(t)
syms DDy   DDp   DDr
% CG position wrt inertial frame & derivatives
syms   x_CG(t)  y_CG(t)  z_CG(t)
syms  Dx_CG(t) Dy_CG(t) Dz_CG(t)
syms DDx_CG   DDy_CG   DDz_CG
% front / rear steer angles
syms   delta_f(t)   delta_r(t)
syms  Ddelta_f(t)  Ddelta_r(t)
syms DDdelta_f    DDdelta_r
% wheel rotation angles
syms   gamma_fr(t)   gamma_fl(t)   gamma_rr(t)   gamma_rl(t)
syms  Dgamma_fr(t)  Dgamma_fl(t)  Dgamma_rr(t)  Dgamma_rl(t)
syms DDgamma_fr    DDgamma_fl    DDgamma_rr    DDgamma_rl

Dgamma = [Dgamma_fr(t) Dgamma_fl(t) Dgamma_rr(t) Dgamma_rl(t)];
% various handy vector combinations of the above
q   = [  y;   p;   r;   x_CG;   y_CG;   z_CG; delta_f(t); delta_r(t); ...
    gamma_fr(t); gamma_fl(t); gamma_rr(t); gamma_rl(t)];
Dq  = [ Dy;  Dp;  Dr;  Dx_CG;  Dy_CG;  Dz_CG; Ddelta_f(t); Ddelta_r(t); ...
    Dgamma_fr(t); Dgamma_fl(t); Dgamma_rr(t); Dgamma_rl(t)];
DDq = [DDy; DDp; DDr; DDx_CG; DDy_CG; DDz_CG; DDdelta_f(t); DDdelta_r(t); ...
    DDgamma_fr(t); DDgamma_fl(t); DDgamma_rr(t); DDgamma_rl(t)];

% position of CG wrt inertial frame
p_CG = [x_CG; y_CG; z_CG];
%% ROTATION MATRICES
% yaw rotation matrix (ROTATION MATRIX from Inertial frame to underriage)
Rz=[cos(y) -sin(y) 0
    sin(y) cos(y)  0
    0      0       1];
if linearized
    % linearized roll rotation matrix
    Ry=[1 0 p
        0 1 0
        p 0 1];
    % linearized roll rotation matrix
    Rx=[1 0 0
        0 1 r
        0 r 1];
else
    % pitch rotation matrix
    Ry=[cos(p)  0  sin(p)
        0       1  0
        -sin(p) 0  cos(p)];
    % roll rotation matrix
    Rx=[1  0      0
        0  cos(r) -sin(r)
        0  sin(r) cos(r) ];
end
% Full rotation matrix from Inertial frame to body frame
% this matrix carries all information about vehicle attitude,
% it can be used to rotate vectors in body to inertial frame
R = Rz*Ry*Rx ;
%% Steering Rotation Matrices
% from undercarriage frame to wheel frames (no ackermann)
if linearized
    Sfr=[1      -delta_f 0
        delta_f 1        0
        0       0        1];
    Sfl=[1      -delta_f 0
        delta_f 1        0
        0       0        1];
    Srr=[1      -delta_r 0
        delta_r 1        0
        0       0        1];
    Srl=[1      -delta_r 0
        delta_r 1        0
        0       0        1];
else
    Sfr=[cos(delta_f) -sin(delta_f) 0
        sin(delta_f)  cos(delta_f)  0
        0             0             1 ];
    Sfl=[cos(delta_f) -sin(delta_f) 0
        sin(delta_f)  cos(delta_f)  0
        0             0             1 ];
    
    Srr=[cos(delta_r) -sin(delta_r) 0
        sin(delta_f)  cos(delta_r)  0
        0             0             1 ];
    Srl=[cos(delta_r) -sin(delta_r) 0
        sin(delta_f)  cos(delta_r)  0
        0             0             1 ];
end
%% SUSPENSION
% virtual spring length at no load
h_f = q_f + m*g/k*l_r/(l_r+l_f);
h_r = q_r + m*g/k*l_f/(l_r+l_f);
% spring to frame attachment point coordinates wrt body frame
P_fr = [ l_f;  t_f/2; h_CG-q_f];
P_fl = [ l_f; -t_f/2; h_CG-q_f];
P_rr = [-l_r;  t_r/2; h_CG-q_r];
P_rl = [-l_r; -t_r/2; h_CG-q_r];
P = [P_fr P_fl P_rr P_rl];
% Suspension attachment point coordinates wrt inertial frame
p = p_CG + R*P;

% suspension displacement
d = - [0 0 1]*p - [h_f h_f h_r h_r];

% suspension velocities
Dd = subs(diff(d,t), diff(q(t)), Dq(t));

%% ENERGY
% suspension damper-dissipated energy (Rayleigh dissipation function)
D = 1/2 * [b_f b_f b_r b_r].'*Dd.^2;  

% energy stored in suspension springs
U_spring = 1/2 * [k_f k_f k_r k_r].'*d.^2;

% gravitational potential energy
U_g = -m*g*z_CG;

% Total potential energy
U = U_spring + U_g;

% translational kinetic energy
T_trans = 1/2 * m   * (Dx_CG.^2 + Dy_CG.^2 + Dz_CG.^2) ...
        + 1/2 * m_u * (Dx_CG.^2 + Dy_CG.^2);

if linearized
    E=[ 0  -sin(y)  p*cos(y)
        0   cos(y)  p*sin(y)
        1        0        -p ];
else
    E=[ 0 -sin(y)  cos(p)*cos(y)
        0  cos(y)  cos(p)*sin(y)
        1       0        -sin(p) ];
end
% Rotational velocity vector wrt inertial frame
w = E * [Dy; Dp; Dr];

% Rotational velocity vector wrt body frame
W = R\w;

% Inertia Tensor
I=[ Ixx 0   Ixz;
    0   Iyy 0;
    Ixz 0   Izz];

% rotational kinetic energy
T_rot = 1/2*W.'*I*W + 1/2*I_u*Dy.^2;

T_w = 1/2 * I_w * sum(Dgamma.^2);

T_steer = 1/2 * I_f * (Ddelta_f.^2) + 1/2 * I_r * (Ddelta_r.^2);
 
% total kinetic energy
T = T_rot + T_trans + T_w + T_steer;
%% WHEELS
% wheel centers
w = p.*[1;1;0]-[0;0;r_0];

% wheel contact point velocities wrt inertial frame
Dw = subs(diff(w, t), diff(q(t),t), Dq(t));

% contact point velocities wrt corresponding wheel frames
v_ufr = Sfr\(Rz\Dw(:,1));
v_ufl = Sfl\(Rz\Dw(:,2));
v_urr = Srr\(Rz\Dw(:,3));
v_url = Srl\(Rz\Dw(:,4));

% wheel loads for friction calculations (obtained as suspension forces)
FZ = - k_f * d - b_f * Dd;

% planar forces wrt inertial frame
f_fr = Rz*Sfr*[FX_fr; FY_fr; 0];
f_fl = Rz*Sfl*[FX_fl; FY_fl; 0];
f_rr = Rz*Srr*[FX_rr; FY_rr; 0];
f_rl = Rz*Srl*[FX_rl; FY_rl; 0];

f = [f_fr;f_fl;f_rr;f_rl];
p = [];



%% LAGRANGE
% all external applied forces
f = [f_fr f_fl f_rr f_rl];
% respective application points
p = [w_fr w_fl w_rr w_rl] - [0;0;1]*[1 1 1]*r_0;
% generalized forces
Q = genforces(f(t),p(t),q(t));

% Potential energy is independent of coordinate derivatives, so second
% order time derivatives will only appear in the following term
LHS = diff(functionalDerivative(T, Dq),t);

% improve speed by replacing time derivatives with correpsonding symbols
LHS = subs(LHS, [diff(q); diff(Dq)], [Dq; DDq]);

% X1 ... X6 are placeholders for all other lagrange-equation terms (which
% do not contain second order derivatives)
X  = sym('X', [6, 1]);

% Solve for second order derivatives (vector space representation)
qdotdot = solve(LHS == X, DDq);

% convert struct to array
qdotdot = [qdotdot.DDy;    qdotdot.DDp;    qdotdot.DDr;
    qdotdot.DDx_CG; qdotdot.DDy_CG; qdotdot.DDz_CG];

%  calculate values for placeholders
RHS = subs(diff(functionalDerivative(U, Dq),t),diff(q), Dq) ...
    + functionalDerivative(T-U, q)                        ...
    - functionalDerivative(D, Dq)                         ...
    + Q;

% replace placeholders
qdotdot = subs(qdotdot, X, RHS);
%% Dynamic system definition
% state vector
x = [q;Dq];

% inputs vector
u = [FX_fr; FY_fr; FX_fl; FY_fl; FX_rr; FY_rr; FX_rl; FY_rl; steer];

% state derivative function
xdot = [Dq; qdotdot];

% Wheel load output function
FZ = [FZ_fr,FZ_fl,FZ_rr,FZ_rl];

% contact point velocities output function
CPV = [v_ufr,v_ufl,v_urr,v_url];
%% Function handles contruction and saving
% redefine lagrangian variables and derivatives so we don't need to worry
% about the time variable
syms y p r Dy Dp Dr x_CG y_CG z_CG Dx_CG Dy_CG Dz_CG

% replace with the new time independent coordinates
x = [y; p; r; x_CG; y_CG; z_CG; Dy; Dp; Dr; Dx_CG; Dy_CG; Dz_CG];

xdot_ = subs(xdot(t), [q(t); Dq(t)], x);
FZ_   = subs(  FZ(t), [q(t); Dq(t)], x);
CPV_  = subs( CPV(t), [q(t); Dq(t)], x);

disp 'writing body dynamics function to file'
BodyDynamicsFunction = matlabFunction(xdot_, 'Vars', {x, u, params},'File','C210BodyDynamicsFunction');

disp 'writing wheel loads function to file'
WheelLoadsFunction  = matlabFunction(FZ_, 'Vars', {x, params},'File','C220WheelLoadsFunction');

disp 'writing contact point velocities function to file'
ContactPointVelocitiesFunction = matlabFunction(CPV_, 'Vars', {x, steer, params},'File','C230ContactPointVelocitiesFunction');
end