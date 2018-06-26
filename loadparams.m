
% load car parametrization from file
try
    [varname, varval] = textread(paramsfile,'%s %f', 'commentstyle','shell');
catch
    warning(['Parameters file "', paramsfile, '", is invalid']);
    return
end

for i=1:length(varval)
    eval([varname{i} '=' num2str(varval(i)) ';']);
end

try
    h_f = q_f + m*g/k_f*l_r/(l_r+l_f);
    h_r = q_r + m*g/k_r*l_f/(l_r+l_f);
    parameters = [g; t_f; t_r; l_f; l_r; q_f; q_r; h_CG; I_f; I_r; k_f; k_r; b_f; b_r; m; m_u; Ixx; Iyy; Izz; Ixz; I_u; r_0; I_w; h_f; h_r];
catch
    warning(['Vehicle Parameters are  missing in "', paramsfile, '"']);
    return
end

for i=1:length(varval)
    eval(['clear ' varname{i}]);
end
clear h_f h_r

% load initial state from file
try
    [varname, varval] = textread(initialfile,'%s %f', 'commentstyle','shell');
catch
    warning(['Initial state file "', initialfile, '", is invalid']);
    return
end

for i=1:length(varval)
    eval([varname{i} '=' num2str(varval(i)) ';']);
end

try
    x0= [y; p; r; x_CG; y_CG; z_CG;
        delta_f; delta_r; gamma_fr; gamma_fl; gamma_rr; gamma_rl;
        Dy; Dp; Dr; Dx_CG; Dy_CG; Dz_CG;
        Ddelta_f; Ddelta_r; Dgamma_fr; Dgamma_fl; Dgamma_rr; Dgamma_rl ];
catch
    warning(['Initial state variables missing in "', initialfile, '"']);
    return
end

for i=1:length(varval)
    eval(['clear ' varname{i}]);
end
clear i varname varval