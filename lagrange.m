% compute left hand side of lagrange equations
function E_LHS = lagrange(t, q, L, D)

if nargin < 4
    D = sym(0);
end

n = length(q(t));

dq = sym('dq',[n,1]);

for i = 1:n
    dq(i) = symfun(['dq' num2str(i) '(t)'],t);
end

L = subs(L,diff(q,t), dq);
D = subs(D,diff(q,t), dq);

E_LHS = diff(functionalDerivative(L, dq),t) ...
        - functionalDerivative(L, q)        ...
        + functionalDerivative(D, dq);
    
E_LHS = subs(E_LHS, dq, diff(q,t));
end