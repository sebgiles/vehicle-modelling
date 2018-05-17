function [F_x, F_y, F_x0, F_y0] = PAC96(kappa, alpha, gamma, F_z, usesym)
if nargin > 4
    if usesym ==0
        F_z0 = 663.947280;
        R_0 = 0.26;
        p_Cx1 =  1.350000   ;
        p_Dx1 =   2.550700  ;
        p_Dx2 =   -0.234080   ;
        p_Ex1 =   0.403270  ;
        p_Ex2  =  1.164900   ;
        p_Ex3 =  -1.085800   ;
        p_Ex4 =  0.853630           ;
        p_Kx1 =    64.347300  ;
        p_Kx2 =   0.000013  ;
        p_Kx3 =   0.048640   ;
        p_Vx1 =   -0.150120  ;
        p_Vx2 =   0.090500  ;
        p_Hx1 =   0.005465   ;
        p_Hx2=    -0.003318 ;
        
        r_Cx1 =   1.118541  ;
        r_Bx1 =   23.880780  ;
        r_Bx2 =   -24.289412  ;
        r_Hx1 =   0.000000  ;
        
        p_Cy1 =   1.377528  ;
        p_Dy1 =   2.486064  ;
        p_Dy2 =   -0.150167  ;
        p_Dy3 =   -1.888950  ;
        p_Ey1 =   -0.000043  ;
        p_Ey2 =   0.000007  ;
        p_Ey3 =   -3683.9043  ;
        p_Ey4 =   -15729.780  ;
        p_Ky1 =   -114.087070  ;
        p_Ky2 =   -3.621914  ;
        p_Ky3 =   2.518860  ;
        p_Hy1 =   0.002182  ;
        p_Hy2 =   -0.001398  ;
        p_Hy3=    -0.119546 ;
        p_Vy1 =   0.026184  ;
        p_Vy2 =   -0.029205  ;
        p_Vy3 =   0.034106  ;
        p_Vy4 =   0.424675  ;
        
        r_By1 =   16.610029  ;
        r_By2 =   25.105729  ;
        r_By3 =   0.044141  ;
        r_Cy1 =   0.992126  ;
        r_Hy1 =   -0.002106   ;
        r_Vy1 =  -0.053669   ;
        r_Vy2 =  0.037279   ;
        r_Vy3 =  0.511559   ;
        r_Vy4 =  2.000000   ;
        r_Vy5 =  6.000000   ;
        r_Vy6=   -2.000000  ;
        
        lambda_Hx =1;
        lambda_mux  =1;
        lambda_Kxkappa  =1;
        lambda_Ex  =1;
        lambda_Cx   =1;
        lambda_Vx =1;
        
        lambda_Hy  =1;
        lambda_Kyalpha  =1;
        lambda_Ey  =1;
        lambda_muy  =1;
        lambda_Cy  =1;
        lambda_Vykappa  =1;
        lambda_Vy  =1;
        lambda_gammay =1;
        
        lambda_Fz0 =1;
    end
else
    
    syms F_z0
    syms p_Cx1 p_Dx1 p_Dx2 p_Ex1 p_Ex2 p_Ex3 p_Ex4 p_Kx1 p_Kx2 p_Kx3 p_Vx1 p_Vx2 p_Hx1 p_Hx2
    syms r_Cx1 r_Bx1 r_Bx2 r_Hx1
    syms p_Cy1 p_Dy1 p_Dy2 p_Dy3 p_Ey1 p_Ey2 p_Ey3 p_Ey4 p_Ky2 p_Ky1 p_Ky3 p_Vy1 p_Vy2 p_Vy3 p_Vy4 p_Hy1 p_Hy2 p_Hy3
    syms r_By1 r_By2 r_By3 r_Cy1 r_Hy1 r_Vy1 r_Vy2 r_Vy3 r_Vy4 r_Vy5 r_Vy6
    syms lambda_Hx lambda_mux lambda_Kxkappa lambda_Ex lambda_Cx  lambda_Vx
    syms lambda_Hy lambda_Kyalpha lambda_Ey lambda_muy lambda_Cy lambda_Vykappa lambda_Vy lambda_gammay
    syms lambda_Fz0
end
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