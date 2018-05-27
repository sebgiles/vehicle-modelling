% apply virtual work principle to find effect of external forces on
% langrangian coordinates
function Q = C041genforces(f,p,q)
  % number of DOFs
  n = length(q);
  % number of external forces
  [~, m] = size(f);

  Q = sym(zeros(n,1));

  for i = 1:n
    for j = 1:m
      dpxdq = functionalDerivative(p(1,j), q(i));
      dpydq = functionalDerivative(p(2,j), q(i));
      dpzdq = functionalDerivative(p(3,j), q(i));
      Q(i) = Q(i) + f(:,j).' * [dpxdq;dpydq;dpzdq];
    end
  end
end
