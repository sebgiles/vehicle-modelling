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
% Front / Rear Roll center vertical offset from CG
syms q_f q_r
% CG no load height
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
% front steer moment
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

% various handy vector combinations of the above
q   = [  y;   p;   r;   x_CG;   y_CG;   z_CG];
Dq  = [ Dy;  Dp;  Dr;  Dx_CG;  Dy_CG;  Dz_CG];
DDq = [DDy; DDp; DDr; DDx_CG; DDy_CG; DDz_CG];

% position of CG wrt inertial frame
p_CG = [x_CG; y_CG; z_CG];

% velocity of CG wrt inertial frame
v_CG = [Dx_CG; Dy_CG; Dz_CG];
%% ROTATION MATRICES from Inertial frame to body frame              
% yaw rotation matrix
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
% from undercarriage frame to wheel frame (no ackermann)
if linearized
    Sr=[1       -delta_r 0
        delta_r 1        0
        0       0        1];
    
    Sf=[1       -delta_f 0
        delta_f 1        0
        0       0        1];
else
    Sr=[cos(delta_r) -sin(delta_r) 0
        sin(delta_f) cos(delta_r)  0
        0            0             1 ];
    
    Sf=[cos(delta_f) -sin(delta_f) 0
        sin(delta_f) cos(delta_f)  0
        0            0             1 ];
end
%% SUSPENSION                                                       
% virtual spring length at no load
h_f = h_CG - q_f;
h_r = h_CG - q_r;
% Suspension attachment point coordinates wrt body frame
P_fr = [ l_f;  t_f/2; q_f];
P_fl = [ l_f; -t_f/2; q_f];
P_rr = [-l_r;  t_r/2; q_r];
P_rl = [-l_r; -t_r/2; q_r];
% Suspension attachment point coordinates wrt inertial frame
p_fr = p_CG + R*P_fr ;
p_fl = p_CG + R*P_fl ;
p_rr = p_CG + R*P_rr ;
p_rl = p_CG + R*P_rl ;
% suspension travel
d_fr = [0 0 1]*p_fr + h_f;
d_fl = [0 0 1]*p_fl + h_f;
d_rr = [0 0 1]*p_rr + h_r;
d_rl = [0 0 1]*p_rl + h_r;
% suspension velocities
Dd_fr = subs(diff(d_fr,t), diff(q(t)), Dq(t));
Dd_fl = subs(diff(d_fl,t), diff(q(t)), Dq(t));
Dd_rr = subs(diff(d_rr,t), diff(q(t)), Dq(t));
Dd_rl = subs(diff(d_rl,t), diff(q(t)), Dq(t));
%% ENERGY                                                           
% suspension damper-dissipated energy (Rayleigh dissipation function)
D = 1/2 * b_f * Dd_fr^2 ...
  + 1/2 * b_f * Dd_fl^2 ...
  + 1/2 * b_r * Dd_rr^2 ...
  + 1/2 * b_r * Dd_rl^2;

% energy stored in suspension springs
U_spring = 1/2 * k_f * (d_fr^2 + d_fl^2) ...
         + 1/2 * k_r * (d_rl^2 + d_rr^2);

% gravitational potential energy
U_g = -m*g*z_CG;

% Total potential energy
U = U_spring + U_g;

% translational kinetic energy
T_trans = 1/2*m*(v_CG.')*v_CG;

if linearized
  E = [ 0  -sin(y)  p*cos(y)
        0   cos(y)  p*sin(y)
        1        0        -p ];
else
  E = [ 0 -sin(y)  cos(p)*cos(y)
        0  cos(y)  cos(p)*sin(y)
        1       0        -sin(p) ];
end
% Rotational velocity vector wrt inertial frame
w = E * [Dy; Dp; Dr];

% Rotational velocity vector wrt body frame
W = R\w;

% Inertia Tensor
I = [Ixx   0 Ixz;
       0 Iyy   0;
     Ixz   0 Izz];
 
% rotational kinetic energy
T_rot = 1/2*W.'*I*W;

% total kinetic energy
T = T_rot + T_trans;
%% ROAD                                                             
% vertical projection to road matrix 
P = [1 0 0; 
     0 1 0; 
     0 0 0];
 
% wheel contact points are obtained by projecting suspension attachment
% points on the road surface
w_fr = P*p_fr;
w_fl = P*p_fl;
w_rr = P*p_rr;
w_rl = P*p_rl;

% wheel contact point velocities wrt inertial frame
Dw_fr = subs(diff(w_fr, t), diff(q(t),t), Dq(t));
Dw_fl = subs(diff(w_fl, t), diff(q(t),t), Dq(t));
Dw_rr = subs(diff(w_rr, t), diff(q(t),t), Dq(t));
Dw_rl = subs(diff(w_rl, t), diff(q(t),t), Dq(t));

% contact point velocities wrt corresponding wheel frames
v_ufr = R_steer\(Rz\Dw_fr);
v_ufl = R_steer\(Rz\Dw_fl);
v_urr = Rz\Dw_rr;
v_url = Rz\Dw_rl;

% wheel loads for friction calculations (obtained as suspension forces)
FZ_fr = - k_f * d_fr - b_f * Dd_fr;
FZ_fl = - k_f * d_fl - b_f * Dd_fl;
FZ_rr = - k_r * d_rr - b_r * Dd_rr;
FZ_rl = - k_r * d_rl - b_r * Dd_rl;

% planar forces at contact points wrt inertial frame
f_fr = Rz*R_steer*[FX_fr; FY_fr; 0];
f_fl = Rz*R_steer*[FX_fl; FY_fl; 0];
f_rr = Rz*        [FX_rr; FY_rr; 0];
f_rl = Rz*        [FX_rl; FY_rl; 0];
%% LAGRANGE                                                         
% all external applied forces
f = [f_fr f_fl f_rr f_rl];
% respective application points
p = [w_fr w_fl w_rr w_rl];
% generalized forces 
Q = C071genforces(f(t),p(t),q(t));

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