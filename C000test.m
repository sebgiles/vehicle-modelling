clear

disp 'defining parameters'
tic
C010parametrize
toc

disp 'defining kinematic relations'
tic
C02kinematics
toc

disp 'defining energies'
tic
C03energy
toc

disp 'defining external forces'
tic
C04forces
toc

disp 'elaborating langrange equations'
tic
C05lagrange
toc

disp 'making function handle'
tic
M = matlabFunction(xdot, 'Vars', {t,x});
toc

disp 'running simulation'
tic
C06simulate
toc
