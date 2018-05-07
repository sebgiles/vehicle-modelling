% time interval in seconds
tspan = [0 10];

% parameter values
V = subs(V, T_f,    1.5);   % front track [m]
V = subs(V, T_r,    1.5);   % rear track [m]
V = subs(V, l_r,    0.5);   % CG to rear axle [m]
V = subs(V, l_f,    1);     % CG to front axle [m]
V = subs(V, Z_f,    0.2);   % CG height wrt front axle
V = subs(V, Z_r,    0.3);   % CG height wrt rear axle
V = subs(V, h_CG,   0.4);   % CG height wrt ground
V = subs(V, g,      9.81);  % gravitational acceleration
V = subs(V, m,      320);   % vehicle sprung mass with driver
V = subs(V, I(1,1), 30);    % moment of inertia about vehicle x axis
V = subs(V, I(2,2), 60);    % moment of inertia about vehicle y axis
V = subs(V, I(3,3), 100);   % moment of inertia about vehicle z axis
I_xz = 0;                   % product of inertia between xz vehicle axes
V = subs(V, I(3,1), I_xz);
V = subs(V, I(1,3), I_xz);

V = subs(V, k_f,    50000); % front spring stiffness
V = subs(V, k_r,    80000); % rear spring stiffness
V = subs(V, b_f,    2000);  % front damping rate
V = subs(V, b_r,    2000);  % rear damping rate

V = subs(V, mu,    100);      % lateral dynamic friction coefficient


% initial values
y_0     = 0;
Dy_0    = -1;
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
