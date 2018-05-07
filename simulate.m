% time interval in seconds
tspan = [0 3];

% parameter values, all units are metric
g    = 9.81;   % gravitational acceleration
T_f  = 1.5 ;   % front track 
T_r  = 1.5 ;   % rear track 
l_r  = 0.5 ;   % CG to rear axle 
l_f  = 1   ;   % CG to front axle 
Z_f  = 0.2 ;   % CG height wrt front axle
Z_r  = 0.3 ;   % CG height wrt rear axle
h_CG = 0.4 ;   % CG height wrt ground
m    = 320 ;   % vehicle sprung mass with driver
Ixx  = 30  ;   % moment of inertia about vehicle x axis
Iyy  = 60  ;   % moment of inertia about vehicle y axis
Izz =  100 ;   % moment of inertia about vehicle z axis
Ixz =  0   ;   % product of inertia between xz vehicle axes
k_f = 50000;   % front spring stiffness
k_r = 80000;   % rear spring stiffness
b_f = 2000 ;   % front damping rate
b_r = 2000 ;   % rear damping rate

V = subs(V);

% initial values
y_0     = 0; 
Dy_0    = 1;
r_0     = 0;
Dr_0    = 0;
p_0     = 0; % to match with rest position 
Dp_0    = 0;
z_CG_0  = -0.388; % to match with rest position
Dz_CG_0 = 0;
x_CG_0  = 0;
Dx_CG_0 = 0;
y_CG_0  = 0;
Dy_CG_0 = 0;

% group initial values, same order as in the S array
s_0 = zeros(length(S),1);
for i = 1:length(S)
    s_0(i) = eval([ char(S(i)) '_0;']);
end

% convert vectorspace model to matlab function for use by ode45
M = matlabFunction(V,'Vars',{'t','Y'});

% where the magic happens
sol = ode45(M,tspan,s_0);
clear tspan

names = cell(1,length(S)+1);
names{1} = 't';
for i = 1:length(S)
    names{i+1}=char(S(i));
end

sim = array2table([sol.x.' sol.y.'],'VariableNames',names);
tile(sim.t, [sim.y, sim.p, sim.r, sim.x_CG, sim.y_CG, sim.z_CG])
