function [M,K] = bonciform(q,Dq,DDq,LHS,RHS);
n = length(q);
e  = eye(n);
R = simplify(subs(LHS,DDq,zeros(n,1)));
MDDq = LHS - R;
RHS = RHS - R
O = subs(RHS,Dq,zeros(n,1));
KDq = simplify(RHS - O);
for i=1:n
    M(:,i) = simplify(subs(MDDq,DDq,e(:,i)));
    K(:,i) = simplify(subs(KDq,q,e(:,i)));

end

%simplify(K*Dq+O-RHS);