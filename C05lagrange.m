% Potential energy is independent of coordinate derivatives, so second
% order time derivatives will only appear in the following term
LHS = diff(functionalDerivative(T, Dq),t);

% improve speed by replacing time derivatives with correpsonding symbols
LHS = subs(LHS, [diff(q); diff(Dq)], [Dq; DDq]);

% redefine lagrangian variables to eliminate time dependency
syms y p r Dy Dp Dr x_CG y_CG z_CG Dx_CG Dy_CG Dz_CG
x = [y; p; r; x_CG; y_CG; z_CG; Dy; Dp; Dr; Dx_CG; Dy_CG; Dz_CG];
% replace with the new time independent coordinates
LHS = subs(LHS, [q; Dq], x);

% X1 ... X6 are placeholders for all other lagrange-equation terms (which
% do not contain second order derivatives)
X  = sym('X', [6, 1]);

% Solve for second order derivatives (vector space representation)
VS = solve(LHS == X, DDq);

% convert struct to array
VS = [VS.DDy; VS.DDp; VS.DDr; VS.DDx_CG; VS.DDy_CG; VS.DDz_CG];

%  calculate values for placeholders
RHS = subs(diff(functionalDerivative(V, Dq),t),diffq, Dq) ...
    + functionalDerivative(T-V, q)                        ...
    - functionalDerivative(D, Dq)                         ...
    + Q;

% replace with the new time independent coordinates
RHS = (subs(RHS, [q;Dq], x));

% replace placeholders
VS = subs(VS, X, RHS);

% Equazioni differenziali
xdot = [x(7:end); VS];

% Lagrange Equations: dt(d(dq_k)(L)) - dq_k(L) + ddq_k(D) = Q_k
% the following are the left hand sides of the lagrange equations as shown
% above
% E_LHS = lagrange(t, q, Dq, DDq, L, D);
% EQ = E_LHS(t) == Q;
% [xdot,x] = odeToVectorField(EQ);