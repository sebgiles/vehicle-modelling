function LagInt = lagint(t, q, L, D)

if nargin < 4
    syms D(t)
end

if isa(q,'symfun')
	q = q(t);
end

LagInt = q; % just for initialization

for i = 1:length(q)
    dq      = fdiff(L,  q(i),           t);
    dqdot   = fdiff(L,  diff(q(i),t),   t);
    dD      = fdiff(D,  diff(q(i),t),   t);
    LagInt(i,1) = simplify(diff(dqdot,t)-dq+dD);
end
end