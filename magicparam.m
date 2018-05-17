function OUT = magicparam(IN);

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

OUT=subs(IN);

end