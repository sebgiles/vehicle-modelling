function [BodyDynamicsFunction, WheelLoadsFunction, ContactPointVelocitiesFunction] = C000carmodel()

%% Build the Algebraic Car model

disp 'defining car parameters'
C010parameters
C020inputs
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