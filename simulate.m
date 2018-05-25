% substitute parameters and set initial values and simulation time
tspan = [0 10];

% initial values for lagrangian variables
y_0     =       0; % yaw
Dy_0    =       0; % yaw rate
r_0     =       0; % roll
Dr_0    =       0; % roll rate
p_0     =       0; % pitch
Dp_0    =       0; % pitch rate
z_CG_0  =  -0.388; % z position
Dz_CG_0 =       0; % z speed
x_CG_0  =       0; % x position
Dx_CG_0 =       0; % x speed
y_CG_0  =       0; % y position
Dy_CG_0 =       0; % y speed

% group initial values, same order as in the S array
s0 = zeros(length(S),1);
for i = 1:length(S)
    s0(i) = eval([ char(S(i)) '_0;']);
end

% convert vectorspace model to matlab function for use by ode45
M = matlabFunction(Vp,'Vars',{'t','Y'});

% where the magic happens
sol = ode45(M,tspan,s0);
clear tspan

names = cell(1,length(S)+1);
names{1} = 't';
for i = 1:length(S)
    names{i+1}=char(S(i));
end
sim = array2table([sol.x.' sol.y.'],'VariableNames',names);
clear sol

tile(sim.t, [sim.y/pi*180, sim.p/pi*180, sim.r/pi*180, ... 
                 sim.x_CG,     sim.y_CG,     sim.z_CG])
             
             
             sfondi grafici in bianco simulink
             oltre al grafico manda toworkspace
