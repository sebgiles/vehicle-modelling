function [F_x, F_y, F_x0, F_y0] = PAC96(kappa, alpha, gamma, F_z, tyreID)
if nargin < 5
    tyreID = 'symbolic';
end

magicparam

df_z = (F_z -F_z0)./F_z0;

%pure long
S_Hx = (p_Hx1 + p_Hx2.*df_z).*lambda_Hx;

kappa_x = kappa + S_Hx;

S_Vx = F_z.* (p_Vx1 + p_Vx2.*df_z).*lambda_Vx.*lambda_mux;

K_xkappa = F_z.*(p_Kx1+p_Kx2.*df_z).*exp(-p_Kx3.*df_z).*lambda_Kxkappa;

E_x = (p_Ex1 + p_Ex2.*df_z+p_Ex3.*df_z.^2).*(1-p_Ex4.*sign(kappa_x)).*lambda_Ex;

mu_x = (p_Dx1 + p_Dx2.*df_z).*lambda_mux;

D_x = mu_x.*F_z;

C_x  = p_Cx1.*lambda_Cx;

B_x = K_xkappa./(C_x.*D_x);

F_x0 = D_x .* sin(C_x.*atan(B_x.*kappa_x  - E_x.* (B_x.*kappa_x - atan(B_x.*kappa_x) ))) + S_Vx;

% pure side F
gamma_y=gamma.*lambda_gammay;

S_Hy=(p_Hy1+p_Hy2.*df_z+p_Hy3.*gamma_y).*lambda_Hy;

alpha_y=alpha+S_Hy;

S_Vy=F_z.*(p_Vy1+p_Vy2.*df_z+(p_Vy3+p_Vy4.*df_z).*gamma_y).*lambda_Vy.*lambda_muy;

K_yalpha=p_Ky1.*F_z0.*sin(2.*atan((F_z)./(p_Ky2.*F_z0.*lambda_Fz0))).*(1-p_Ky3.*abs(gamma)).*lambda_Fz0.*lambda_Kyalpha;

E_y=(p_Ey1+p_Ey2.*df_z).*(1-(p_Ey3+p_Ey4.*gamma_y).*sign(alpha_y)).*lambda_Ey;

mu_y=(p_Dy1+p_Dy2.*df_z).*(1-p_Dy3.*(gamma_y).^2).*lambda_muy;

D_y=mu_y.*F_z;

C_y=p_Cy1.*lambda_Cy;

B_y=(K_yalpha)./(C_y.*D_y);

F_y0=D_y.*sin(C_y.*atan(B_y.*alpha_y-E_y.*(B_y.*alpha_y-atan(B_y.*alpha_y))))+S_Vy;

%pure side M

%TODO

% comb long

C_xalpha = r_Cx1;

B_xalpha = r_Bx1 .*  cos(atan(r_Bx2.*kappa));

S_Hxalpha = r_Hx1;

D_xalpha = F_x0 ./(cos(C_xalpha.*atan(B_xalpha.*S_Hxalpha)));

F_x = D_xalpha.*cos(C_xalpha.*atan(B_xalpha.*(alpha+S_Hxalpha)));

% comb lat
C_ykappa = r_Cy1;

D_Vykappa = mu_y.*F_z.*(r_Vy1+r_Vy2.*df_z + r_Vy3.*gamma).*cos(atan(r_Vy4.*alpha));

S_Vykappa = D_Vykappa.*sin(r_Vy5.*atan(r_Vy6.*kappa)).*lambda_Vykappa;

B_ykappa = r_By1.*cos(atan(r_By2.*(alpha-r_By3)))+S_Vykappa;

S_Hykappa = r_Hy1;

D_ykappa = F_y0 ./ (cos(C_ykappa.*atan(B_ykappa.*S_Hykappa)));

F_y = D_ykappa.*cos(C_ykappa.*atan(B_ykappa.*(kappa+S_Hykappa)))+S_Vykappa;

end
