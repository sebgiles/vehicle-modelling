syms k b l h s m I g
params = [k b l h s m I g].';
syms t
syms z(t) y(t) r(t)
syms Dz(t) Dy(t) Dr(t)
syms DDz DDy DDr

q   =    [y;  z;  r];
Dq  =   [Dy; Dz; Dr];
DDq =  [DDy;DDz;DDr];

CG = [y;z];
Rx = [cos(r) -sin(r)
      sin(r) cos(r)];
  
rc = [0; -h+s];
RC = CG + Rx*rc;

p = [-l/2 -h+s;
      l/2 -h+s];

P = CG + Rx*p;

d = [0 1]*P - (s - k*m*g);
Dd = subs(diff(d,t), diff(q,t), Dq); 

% energy stored in suspension springs
U_spring = 1/2 *  d.^2 * [k k].';

% gravitational potential energy
U_g = -m*g*z;

% Total potential energy
U = U_spring + U_g;

D = 1/2 * Dd.^2 * [b b].';  

T = 1/2* ( I*Dr.^2 + m*(Dy.^2 + Dz.^2));

syms FCG FWL FWR FRC
u = [FCG FWL FWR FRC];

Fe = [FCG FWL FWR FRC
     0   0   0   0  ];
Pe = [CG  P  RC];
Pe = Pe(t);
% generalized forces
Q = genforces(Fe(:), Pe(:), q(t));

% Potential energy is independent of coordinate derivatives, so second
% order time derivatives will only appear in the kinetic term
LHS = diff(functionalDerivative(T, Dq),t);
% improve speed by replacing time derivatives with correpsonding symbols
LHS = simplify(subs(LHS, [diff(q); diff(Dq)], [Dq; DDq]));
% X1 ... X12 are placeholders for all other lagrange-equation terms (which
% do not contain second order derivatives)
X  = sym('X', [3, 1]);
% Solve for second order derivatives (vector space representation)
qdotdot = solve(LHS == X, DDq);
% convert struct to array
qdotdot = [qdotdot.DDy;    qdotdot.DDz;    qdotdot.DDr];
qdotdot = simplify(qdotdot);
% calculate values for placeholders
RHS = subs(diff(functionalDerivative(U, Dq),t),diff(q), Dq) ...
    + simplify(functionalDerivative(T-U, q))                ...
    - functionalDerivative(D, Dq)                           ...
    + Q;
qdotdot = subs(qdotdot, X, RHS);
syms z y r Dy Dz Dr
x= [y;z;r;Dy;Dz;Dr];
xdot = [Dq; qdotdot];
xdot_ = subs(xdot(t), [q(t); Dq(t)], x);
smodel = matlabFunction(xdot_, 'Vars', {x, u, params},'File','suspmodel');