% T, V sono energia cinetica e potenziale
% solo T dipende dalle derivate delle variabili lagrangiane
% si conseguenza la parte che segue è l'unica che contiene derivate seconde
LHS = diff(functionalDerivative(T, Dq),t);

% sostituisco alle derivate i corrispondenti simboli (migliora velocità) 
LHS = subs( LHS, [diff(q); diff(Dq)], [Dq; DDq]);

% X1 ... X6 comprendono tutti i termini delle equazioni di lagrange che non
% dipendono dalle derivate seconde
X = sym('X', [6, 1]);
EQ = LHS == X;

% Esplicito le derivate seconde
VS = solve(EQ, DDq);
% converto lo struct ottenuto da solve in un vettore
VS = [VS.DDy; VS.DDp; VS.DDr; VS.DDx_CG; VS.DDy_CG; VS.DDz_CG];

RHS = subs(diff(functionalDerivative(V, Dq),t),diff(q), Dq) ...
    + functionalDerivative(T-V, q)                          ...
    - functionalDerivative(D, Dq)                           ...
    + Q;


% Lo stato del mio sistema dinamico è dato dalle 6 coordinate lagrangiane
% e dalle rispettive derivate [q; Dq]. 

% Equazioni differenziali
VS = [Dq; subs(VS, X, RHS)];



% Lagrange Equations: dt(d(dq_k)(L)) - dq_k(L) + ddq_k(D) = Q_k
% the following are the left hand sides of the lagrange equations as shown
% above
% E_LHS = lagrange(t, q, Dq, DDq, L, D);
% EQ = E_LHS(t) == Q;
