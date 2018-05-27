function [CarDynamics, WheelLoads, ContactPointVelocities] = C000BuildModel()
usesym = 1;
linear = 0;

disp 'defining parameters'
tic
C010geometry
toc

disp 'defining kinematic relations'
tic
C020kinematics
toc

disp 'defining energies'
tic
C030energy
toc

disp 'defining external forces'
tic
C040forces
toc

disp 'elaborating langrange equations'
tic
C050lagrange
toc

disp 'wheel loads and contact point velocities'
tic
C060contactPoints
toc

disp 'making function handle'
tic


% redefine lagrangian variables and derivatives to eliminate time 
% dependency
syms y p r Dy Dp Dr x_CG y_CG z_CG Dx_CG Dy_CG Dz_CG


% system state vector
x = [y; p; r; x_CG; y_CG; z_CG; Dy; Dp; Dr; Dx_CG; Dy_CG; Dz_CG];

% inputs vector
u = [F; steer];

% state derivative function
xdot = [x(7:end); qdotdot];

% replace with the new time independent coordinates
LHS = subs(LHS, [q; Dq], x);
FZ = subs(FZ,[q; Dq], x);
CPV = subs(CPV,[q; Dq], x);

CarDynamics = matlabFunction(xdot, 'Vars', {x, u, parameters});
WheelLoads  = matlabFunction(FZ, 'Vars', {x, parameters});
ContactPointVelocities = matlabFunction(CPV, 'Vars', {x, steer, parameters});

toc

save('CarModel.mat', 'CarDynamics','WheelLoads','ContactPointVelocities');

end
