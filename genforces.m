% apply virtual work principle to find effect of external forces on
% langrangian coordinates
function Q = genforces(f,p,q,t)
  f=f(t);
  p=p(t);
  if isa(q,'symfun')
    q = q(t);
  end
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

      %disp (j + (i-1)*m)
    end
  end

  %Q = simplify(Q);
