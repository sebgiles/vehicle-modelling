% apply virtual work principle to find effect of external forces on
% langrangian coordinates
function Q = genforces(f,p,q)
% number of DOFs
n = length(q);
% number of external forces
[~, m] = size(f);

Q = sym(zeros(n,1));

for j = 1:m
    dp = functionalDerivative(p(j), q);
    Q = Q + f(j) * dp;
end
end
