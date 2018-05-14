% apply virtual work principle to find effect of external forces on
% langrangian coordinates
function Q = genforces(f,p,q)

% number of DOFs
n = length(q);
% number of external forces
[~, m] = size(f);

Q = sym(zeros(n,1));

for i = 1:n
    for j = 1:m
        Q(i) = Q(i) + f(:,j).' * functionalDerivative(p(:,j), q(i));
    end
end

Q = simplify(Q);