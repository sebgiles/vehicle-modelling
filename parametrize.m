% time interval in seconds
tspan = [0 10];

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

% initial values for lagrangian variables
y_0     =       0; % yaw
Dy_0    =       0; % yaw rate
r_0     =       0; % roll
Dr_0    =       0; % roll rate
p_0     =       0; % pitch
Dp_0    =       0; % pitch rate
z_CG_0  =  -0.388; % vertical position
Dz_CG_0 =       0; % vertical speed
x_CG_0  =       0; % x position
Dx_CG_0 =       0; % x speed
y_CG_0  =       0; % y position
Dy_CG_0 =       0; % y speed

V = subs(V);
