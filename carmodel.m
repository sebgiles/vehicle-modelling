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
% syms ts_fr ts_fl ts_rr rs_rl % currently NOT USED
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

syms h_f h_r
% have to update / change this
params = [g; t_f; t_r; l_f; l_r; q_f; q_r; h_CG; I_f; I_r; k_f; k_r; 
    b_f; b_r; m; m_u; Ixx; Iyy; Izz; Ixz; I_u; r_0; I_w; h_f; h_r];
%% Inputs Definition
disp 'defining car input signals'
% Front / Rear steer torques
syms M_sf M_sr
% planar wheel forces
syms FX_fr FY_fr FX_fl FY_fl FX_rr FY_rr FX_rl FY_rl
% self aligning wheel torques
syms MZ_fr MZ_fl MZ_rr MZ_rl
% motor/brake torque
syms MW_fr MW_fl MW_rr MW_rl
%% TIME
syms t
%% CAR STATE COORDINATES
disp 'defining car coordinates'
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

Dgamma = [Dgamma_fr Dgamma_fl Dgamma_rr Dgamma_rl];
% various handy vector combinations of the above
q   = [  y;   p;   r;   x_CG;   y_CG;   z_CG; delta_f; delta_r; ...
    gamma_fr; gamma_fl; gamma_rr; gamma_rl];
Dq  = [ Dy;  Dp;  Dr;  Dx_CG;  Dy_CG;  Dz_CG; Ddelta_f; Ddelta_r; ...
    Dgamma_fr; Dgamma_fl; Dgamma_rr; Dgamma_rl];
DDq = [DDy; DDp; DDr; DDx_CG; DDy_CG; DDz_CG; DDdelta_f; DDdelta_r; ...
       DDgamma_fr; DDgamma_fl; DDgamma_rr; DDgamma_rl];

% position of CG wrt inertial frame
p_CG = [x_CG; y_CG; z_CG];
%% ROTATION MATRICES
disp 'defining matrices'

% yaw rotation matrix (ROTATION MATRIX from Inertial to undercarriage)
Rz=[cos(y) -sin(y) 0
    sin(y) cos(y)  0
    0      0       1];
if linearized
    % linearized pitch rotation matrix
    Ry=[1 0 p
        0 1 0
       -p 0 1];
    % linearized roll rotation matrix
    Rx=[1 0 0
        0 1 -r
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
disp 'defining suspension'

% virtual spring length at no load is computed only beofre simulation 
% h_f = q_f + 0.5*m*g/k_f*l_r/(l_r+l_f);
% h_r = q_r + 0.5*m*g/k_r*l_f/(l_r+l_f);
% spring to frame attachment point coordinates wrt body frame
P =[l_f      l_f      l_r      l_r
    t_f/2    -t_f/2   t_r/2    -t_r/2
    h_CG-q_f h_CG-q_f h_CG-q_r h_CG-q_r];
% Suspension attachment point coordinates wrt inertial frame
ps = p_CG + R*P;
% suspension displacement
d = - [0 0 1]*ps - [h_f h_f h_r h_r];
% suspension velocities
Dd = subs(diff(d,t), diff(q,t), Dq); 
%% ENERGY
disp 'defining energies'

% suspension damper-dissipated energy (Rayleigh dissipation function)
D        = 1/2 * Dd.^2 * [b_f b_f b_r b_r].';  

% energy stored in suspension springs
U_spring = 1/2 *  d.^2 * [k_f k_f k_r k_r].';

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
    E=[ 0 -sin(y)  cos(p)*cos(y) ;
        0  cos(y)  cos(p)*sin(y) ;
        1       0        -sin(p) ];
end

% Rotational velocity vector wrt inertial frame
w = E * [Dy(t); Dp(t); Dr(t)];

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
disp 'defining road interface'

% wheel contact point velocities wrt inertial frame
v = subs(diff(ps, t), diff(q,t), Dq);
v = v(t);
% total planar force wrt inertial frame
f= [Rz*Sfr*[FX_fr; FY_fr; 0] ...
    Rz*Sfl*[FX_fl; FY_fl; 0] ...
    Rz*Srr*[FX_rr; FY_rr; 0] ...
    Rz*Srl*[FX_rl; FY_rl; 0] ];
f=simplify(f(t));
ps=ps(t);
%% TORQUES ON BODY
disp 'defining torques'

% total motors/brakes reaction torques
M_motor = Rz*Sfr*[0; MW_fr; 0] + ...
          Rz*Sfl*[0; MW_fl; 0] + ... 
          Rz*Srr*[0; MW_rr; 0] + ...
          Rz*Srl*[0; MW_rl; 0] ;
M_motor = simplify(M_motor);
chi = [y(t);p(t);r(t)];
M_body = E \ M_motor;
%% TORQUES ON WHEELS
M_w = [MW_fr; MW_fl; MW_rr; MW_rl] - r_0 * [FX_fr; FX_fl; FX_rr; FX_rl];
P_w = [gamma_fr(t); gamma_fl(t); gamma_rr(t); gamma_rl(t)];
%% TORQUE ON STEERING
M_f = M_sf + MZ_fr + MZ_fl;
M_r = M_sr + MZ_rr + MZ_rl;
%% LAGRANGE
disp 'calculating lagrange magic'
% all external forces/torques
F_ext = [f(:); M_body(t); M_w;        M_f;        M_r];
% correpsonding application points/angles
P_ext = [ps(:);      chi; P_w; delta_f(t); delta_r(t)];
% generalized forces
Q = genforces(F_ext, P_ext, q(t));

% Potential energy is independent of coordinate derivatives, so second
% order time derivatives will only appear in the kinetic term
LHS = diff(functionalDerivative(T, Dq),t);
% improve speed by replacing time derivatives with correpsonding symbols
LHS = simplify(subs(LHS, [diff(q); diff(Dq)], [Dq; DDq]));
% X1 ... X12 are placeholders for all other lagrange-equation terms (which
% do not contain second order derivatives)
X  = sym('X', [12, 1]);
% Solve for second order derivatives (vector space representation)
qdotdot = solve(LHS == X, DDq);
% convert struct to array
qdotdot = [qdotdot.DDy;    qdotdot.DDp;    qdotdot.DDr;
           qdotdot.DDx_CG; qdotdot.DDy_CG; qdotdot.DDz_CG;
           qdotdot.DDdelta_f; qdotdot.DDdelta_r;
           qdotdot.DDgamma_fr; qdotdot.DDgamma_fl;
           qdotdot.DDgamma_rr; qdotdot.DDgamma_rl           ];
qdotdot = simplify(qdotdot);
% calculate values for placeholders
RHS = subs(diff(functionalDerivative(U, Dq),t),diff(q), Dq) ...
    + simplify(functionalDerivative(T-U, q))                ...
    - functionalDerivative(D, Dq)                           ...
    + Q;

% replace placeholders
qdotdot = subs(qdotdot, X, RHS);
%% Dynamic system definition
% inputs vector
u= [FX_fr FY_fr MZ_fr ...
    FX_fl FY_fl MZ_fl ...
    FX_rr FY_rr MZ_rr ...
    FX_rl FY_rl MZ_rl ...
    MW_fr MW_fl MW_rr MW_rl ...
    M_sf M_sr].';
% state derivative function
xdot = [Dq; qdotdot];
% wheel loads for friction calculations (obtained as suspension forces)
FZ = - k_f * d - b_f * Dd;
% contact point velocities wrt corresponding wheel frames
v_w = [ Sfr\(Rz\v(:,1)) Sfl\(Rz\v(:,2)) Srr\(Rz\v(:,3)) Srl\(Rz\v(:,4)) ];
%% Function handles contruction and saving
% redefine lagrangian variables and derivatives so we don't need to worry
% about the time variable
syms y p r Dy Dp Dr x_CG y_CG z_CG Dx_CG Dy_CG Dz_CG
syms gamma_fr gamma_fl gamma_rr gamma_rl 
syms Dgamma_fr Dgamma_fl Dgamma_rr Dgamma_rl 
syms delta_f delta_r Ddelta_f Ddelta_r 

% replace with the new time independent coordinates
x= [y; p; r; x_CG; y_CG; z_CG;
    delta_f; delta_r; gamma_fr; gamma_fl; gamma_rr; gamma_rl;
    Dy; Dp; Dr; Dx_CG; Dy_CG; Dz_CG;
    Ddelta_f; Ddelta_r; Dgamma_fr; Dgamma_fl; Dgamma_rr; Dgamma_rl ];

xdot_ = subs(xdot(t), [q(t); Dq(t)], x);
FZ_   = subs(  FZ(t), [q(t); Dq(t)], x);
CPV_  = subs( v_w(t), [q(t); Dq(t)], x);

disp 'writing body dynamics function to file'
delete BodyDynamicsFunction.m
BodyDynamicsFunction = matlabFunction(xdot_, 'Vars', {x, u, params},'File','BodyDynamicsFunction');

disp 'writing wheel loads function to file'
delete WheelLoadsFunction.m
WheelLoadsFunction  = matlabFunction(FZ_, 'Vars', {x, params},'File','WheelLoadsFunction');

disp 'writing contact point velocities function to file'
delete ContactPointVelocitiesFunction.m
ContactPointVelocitiesFunction = matlabFunction(CPV_, 'Vars', {x, params},'File','ContactPointVelocitiesFunction');
