% all external applied forces
f = [f_fr f_fl f_rr f_rl];
% respective application points
p = [w_fr w_fl w_rr w_rl];
% generalized forces 
Q = genforces(f(t),p(t),q(t));

% Potential energy is independent of coordinate derivatives, so second
% order time derivatives will only appear in the following term
LHS = diff(functionalDerivative(T, Dq),t);

% improve speed by replacing time derivatives with correpsonding symbols
LHS = subs(LHS, [diff(q); diff(Dq)], [Dq; DDq]);

% X1 ... X6 are placeholders for all other lagrange-equation terms (which
% do not contain second order derivatives)
X  = sym('X', [6, 1]);

% Solve for second order derivatives (vector space representation)
qdotdot = solve(LHS == X, DDq);

% convert struct to array
qdotdot = [qdotdot.DDy;    qdotdot.DDp;    qdotdot.DDr; 
           qdotdot.DDx_CG; qdotdot.DDy_CG; qdotdot.DDz_CG];

%  calculate values for placeholders
RHS = subs(diff(functionalDerivative(V, Dq),t),diffq, Dq) ...
    + functionalDerivative(T-V, q)                        ...
    - functionalDerivative(D, Dq)                         ...
    + Q;

% replace placeholders
qdotdot = subs(qdotdot, X, RHS);