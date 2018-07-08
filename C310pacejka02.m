function [F_x, F_y, M_z] = C310pacejka02(kappa, alpha, gamma, F_z, tyre)
T = tyre;

F_z0 = T.FNOMIN;

df_z = (F_z - F_z0)./F_z0;

%pure long
S_Hx = (T.PHX1 + T.PHX2.*df_z).*T.LHX;

kappa_x = kappa + S_Hx;

gamma_x = gamma.*T.LGAX;

S_Vx =  F_z.* (T.PVX1 + T.PVX2.*df_z).*T.LVX.*T.LMUX;

K_x = F_z.*(T.PKX1+T.PKX2.*df_z).*exp(T.PKX3.*df_z).*T.LKX;

E_x = (T.PEX1 + T.PEX2.*df_z+T.PEX3.*df_z.^2).*(1-T.PEX4.*sign(kappa_x)).*T.LEX;

mu_x = (T.PDX1 + T.PDX2.*df_z).*(1-T.PDX3.*gamma_x.^2).*T.LMUX;

D_x = mu_x.*F_z;

C_x  = T.PCX1.*T.LCX;

B_x = K_x./(C_x.*D_x);

F_x0 = D_x .* sin(C_x.*atan(B_x.*kappa_x  - E_x.* (B_x.*kappa_x - atan(B_x.*kappa_x) ))) + S_Vx;

% pure side F
gamma_y=gamma.*T.LGAY;

S_Hy = (T.PHY1+T.PHY2.*df_z).*T.LHY + T.PHY3.*gamma_y;

alpha_y=alpha+S_Hy;

S_Vy = F_z.*((T.PVY1+T.PVY2.*df_z).*T.LVY+(T.PVY3+T.PVY4.*df_z).*gamma_y).*T.LMUY;

K_y0=T.PKY1.*F_z0.*sin(2.*atan((F_z)./(T.PKY2.*F_z0.*T.LFZO))).*T.LFZO.*T.LKY;

K_y=K_y0.*(1-T.PKY3.*abs(gamma));

E_y=(T.PEY1+T.PEY2.*df_z).*(1-(T.PEY3+T.PEY4.*gamma_y).*sign(alpha_y)).*T.LEY;

mu_y=(T.PDY1+T.PDY2.*df_z).*(1-T.PDY3.*(gamma_y).^2).*T.LMUY;

D_y=mu_y.*F_z;

C_y=T.PCY1.*T.LCY;

B_y=(K_y)./(C_y.*D_y);

F_y0=D_y.*sin(C_y.*atan(B_y.*alpha_y-E_y.*(B_y.*alpha_y-atan(B_y.*alpha_y))))+S_Vy;

%TODO: Aligning

% comb long

B_xalpha = T.RBX1 .*  cos(atan(T.RBX2.*kappa));

C_xalpha = T.RCX1;

E_xalpha = T.REX1+T.REX2.*df_z;

S_Hxalpha = T.RHX1;

D_xalpha = F_x0 ./(cos(C_xalpha.*atan(B_xalpha.*S_Hxalpha - E_xalpha.*(B_xalpha.*S_Hxalpha - atan(B_xalpha.*S_Hxalpha)))));

alpha_s = alpha + S_Hxalpha;

F_x = D_xalpha.*cos(C_xalpha.*atan(B_xalpha.*alpha_s-E_xalpha.*(B_xalpha.*alpha_s-atan(B_xalpha.*alpha_s))));

% comb lat

E_ykappa = T.REY1 + T.REY2.*df_z;

S_Hykappa = T.RHY1;

B_ykappa = T.RBY1.*cos(atan(T.RBY2.*(alpha-T.RBY3))).*T.LYKA;

C_ykappa = T.RCY1;

D_ykappa = F_y0 ./ (cos(C_ykappa.*atan(B_ykappa.*S_Hykappa - E_ykappa.*(B_ykappa.*S_Hykappa-atan(B_ykappa.*S_Hykappa)))));

D_Vykappa = mu_y.*F_z.*(T.RVY1+T.RVY2.*df_z + T.RVY3.*gamma).*cos(atan(T.RVY4.*alpha));

S_Vykappa = D_Vykappa.*sin(T.RVY5.*atan(T.RVY6.*kappa)).*T.LVYKA;

kappa_s = kappa + S_Hykappa;

F_y = D_ykappa.*cos(C_ykappa.*atan(B_ykappa.*kappa_s - E_ykappa.*(B_ykappa.*kappa_s-atan(B_ykappa.*kappa_s))))+S_Vykappa;

%% TODO: implement self aligning torque equations
M_z = -F_y*0.05;

end
