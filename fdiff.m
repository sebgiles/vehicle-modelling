function dy = fdiff(y,x,t)
syms x_
if ~isa(y,'symfun')
    y(t) = y;
end

if isa(x,'symfun')
    x = x(t);
end

y_  = subs(y,x,x_);
dy  = diff(y_,x_);
dy  = subs(dy,x_,x);

dy  = dy(t);    
end