% one dimensional acceleration sanity check

syms t % sec

% velocity
syms v(t) % m/s

tyre = 'Hoosier FSAE 20.5x7.0-13 43129 @ 12 psi, 7 inch rim';

m = 140; % kg

% wheel angular velocity
N = 38.5; % rad/sec
R0 = 0.26; % m
RE = v/N;  % m

SL = R0/RE-1;   
SA = 0;         % rad
FZ = 600;       % N
IA = 0;         % rad
FX = PAC96(SL,SA,IA,FZ,tyre);

E = FX == m*diff(v);

% make substitutions to transform into first order equations
% the system is now of the form dS/dt = V(S)
[V,S] = odeToVectorField(E);

v_0 = 0.00001; % x speed

% group initial values, same order as in the S array
s0 = zeros(length(S),1);
for i = 1:length(S)
    s0(i) = eval([ char(S(i)) '_0;']);
end

tspan = [0 10];

% convert vectorspace model to matlab function for use by ode45
M = matlabFunction(V,'Vars',{'t','Y'});

% where the magic happens
sol = ode45(M,tspan,s0);

names = cell(1,length(S)+1);
names{1} = 't';
for i = 1:length(S)
    names{i+1}=char(S(i));
end
sim = array2table([sol.x.' sol.y.'],'VariableNames',names);

tile(sim.t, [sim.v]);
