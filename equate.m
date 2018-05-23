tic
% Lagrange Equations: dt(ddq_k(L)) - dq_k(L) + ddq_k(D) = Q_k
EQ = E_LHS(t) == Q;
toc

tic
% make substitutions to transform into first order equations
% the system is now of the form dS/dt = V(S)
%[V,S] = odeToVectorField(EQ);

V = solve(EQ, DDq);


toc
