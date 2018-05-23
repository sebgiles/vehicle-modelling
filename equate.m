tic
% Lagrange Equations: dt(ddq_k(L)) - dq_k(L) + ddq_k(D) = Q_k
E = E_LHS == Q;
toc

tic
% make substitutions to transform into first order equations
% the system is now of the form dS/dt = V(S)
[V,S] = odeToVectorField(E);
toc