
            % sfondi grafici in bianco simulink
             %oltre al grafico manda toworkspace

% substitute parameters and set initial values and simulation time
tspan = [0 0.5];

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
Dx_CG_0 = 0.00001; % x speed
y_CG_0  =       0; % y position
Dy_CG_0 =       0; % y speed

s0 = [ y_0;  p_0;  r_0;  x_CG_0;  y_CG_0;  z_CG_0;  
      Dy_0; Dp_0; Dr_0; Dx_CG_0; Dy_CG_0; Dz_CG_0];

% where the magic happens
sol = ode45(M,tspan,s0);
clear tspan

names = cell(1,length(x)+1);
names{1} = 't';
for i = 1:length(x)
    names{i+1}=char(x(i));
end
sim = array2table([sol.x.' sol.y.'],'VariableNames',names);
clear sol

tile(sim.t, [sim.y/pi*180, sim.p/pi*180, sim.r/pi*180, ... 
                 sim.x_CG,     sim.y_CG,     sim.z_CG])
             
             
