% compute left hand side of lagrange equations
function E_LHS = lagrange(t, q, Dq, DDq, L, D)

  if isa(q,'symfun')
    q = q(t);
  end

  if isa(Dq,'symfun')
    Dq = Dq(t);
  end

  if nargin < 6
    D = sym(0);
  end

  ddLdtdq = diff(functionalDerivative(L, Dq),t);
  ddLdtdq = subs( ddLdtdq, [diff(q); diff(Dq)], [Dq; DDq]);

  E_LHS = ddLdtdq               ...
  - functionalDerivative(L, q)  ...
  + functionalDerivative(D, Dq);

end
