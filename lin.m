function l = lin(f,vars)
l = f;
syms linvar
for v = vars
    
    l = subs(l,v,linvar);
    l = taylor(l,linvar,'Order', 2,'OrderMode', 'relative');
    l = subs(l,linvar,v);

end

l = simplify(l);