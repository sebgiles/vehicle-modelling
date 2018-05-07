function out = getel(x, index, t)
if isa(x,'symfun')
    x = x(t);
end
out = x(index,:);
end