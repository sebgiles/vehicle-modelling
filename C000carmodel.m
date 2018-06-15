function [BodyDynamicsFunction, WheelLoadsFunction, ContactPointVelocitiesFunction] = C000carmodel()


%% Car parametrization
disp 'defining car parameters'
% gravitational acceleration
syms g
% track widths
syms T_f T_r
% Front / Rear Axle Distance from CG
syms l_f l_r
% Front / Rear Roll center Height
syms Z_f Z_r
% CG no load height
syms h_CG
% Front / Rear caster Angle
syms
% Wheel Radius
syms
% spring coefficients
syms k_f k_r
% damper coefficients
syms b_f b_r
% vehicle mass
syms m
% vehicle inertia tensor wrt body frame
syms Ixx Iyy Izz Ixz

% would be cool to change this
params = [T_f; T_r; l_f; l_r; Z_f; Z_r; h_CG; k_f; k_r; b_f; b_r; g; m;
              Ixx; Iyy; Izz; Ixz];
          
%% Inputs Definition
% steer angle
syms steer
% planar wheel forces
syms FX_fr FY_fr FX_fl FY_fl FX_rr FY_rr FX_rl FY_rl
% self aligning wheel torques
syms MZ_fr MS_fl MZ_rr MZ_rl

F = [FX_fr; FY_fr; FX_fl; FY_fl; FX_rr; FY_rr; FX_rl; FY_rl];

% inputs vector
u = [F; steer];

%%
disp 'defining coordinate systems'
C030coordinates
disp 'evaluating suspension kinematics'
C040suspension
disp 'evaluating energy fucntions'
C050energy
disp 'defining road contact interface'
C060road
disp 'compiling lagrange equations'
C070lagrange

%% Dynamic system definition

% state vector
x = [q;Dq];

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