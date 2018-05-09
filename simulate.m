[Vp, s0, tspan] = parametrize(V,S);

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
tile(sim.t, [sim.y/pi*180, sim.p/pi*180, sim.r/pi*180, ... 
                 sim.x_CG,     sim.y_CG,     sim.z_CG])
