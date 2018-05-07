parametrize
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
tile(sim.t, [sim.y/pi*180, sim.p/pi*180, sim.r/pi*180, sim.x_CG, sim.y_CG, sim.z_CG])
