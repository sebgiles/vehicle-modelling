% I use this function as a continuous, differentiable sign function

function l = logistic(x)
    l = 2/(1+exp(-4*x))-1;
return