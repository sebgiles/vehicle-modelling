function CPV_ = C230ContactPointVelocitiesFunction(in1,steer,in3)
%C230CONTACTPOINTVELOCITIESFUNCTION
%    CPV_ = C230CONTACTPOINTVELOCITIESFUNCTION(IN1,STEER,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    29-May-2018 18:24:47

Dp = in1(8,:);
Dr = in1(9,:);
Dx_CG = in1(10,:);
Dy = in1(7,:);
Dy_CG = in1(11,:);
T_f = in3(1,:);
T_r = in3(2,:);
Z_f = in3(5,:);
Z_r = in3(6,:);
l_f = in3(3,:);
l_r = in3(4,:);
p = in1(2,:);
r = in1(3,:);
y = in1(1,:);
t2 = cos(y);
t3 = sin(y);
t4 = cos(steer);
t5 = sin(steer);
t6 = t2.^2;
t7 = t3.^2;
t8 = sin(r);
t9 = sin(p);
t10 = cos(p);
t11 = cos(r);
t12 = t6+t7;
t13 = 1.0./t12;
t14 = Dx_CG.*t2.*t4.*2.0;
t15 = Dy_CG.*t3.*t4.*2.0;
t16 = Dx_CG.*t3.*t5.*2.0;
t17 = Dr.*T_f.*t5.*t6.*t8;
t18 = Dr.*Z_f.*t5.*t7.*t11.*2.0;
t19 = Dy.*Z_f.*t4.*t7.*t8.*2.0;
t20 = Dr.*T_f.*t5.*t7.*t8;
t21 = Dr.*Z_f.*t5.*t6.*t11.*2.0;
t22 = Dy.*Z_f.*t4.*t6.*t8.*2.0;
t23 = Dp.*Z_f.*t4.*t6.*t10.*t11.*2.0;
t24 = Dp.*T_f.*t4.*t6.*t8.*t10;
t25 = Dr.*T_f.*t4.*t6.*t9.*t11;
t26 = Dp.*Z_f.*t4.*t7.*t10.*t11.*2.0;
t27 = Dp.*T_f.*t4.*t7.*t8.*t10;
t28 = Dr.*T_f.*t4.*t7.*t9.*t11;
t29 = Dx_CG.*t2.*2.0;
t30 = Dy_CG.*t3.*2.0;
t31 = Dy.*Z_r.*t6.*t8.*2.0;
t32 = Dp.*l_r.*t6.*t9.*2.0;
t33 = Dy.*Z_r.*t7.*t8.*2.0;
t34 = Dp.*l_r.*t7.*t9.*2.0;
t35 = Dp.*Z_r.*t6.*t10.*t11.*2.0;
t36 = Dp.*T_r.*t6.*t8.*t10;
t37 = Dr.*T_r.*t6.*t9.*t11;
t38 = Dp.*Z_r.*t7.*t10.*t11.*2.0;
t39 = Dp.*T_r.*t7.*t8.*t10;
t40 = Dr.*T_r.*t7.*t9.*t11;
t41 = Dy_CG.*t2.*t4.*2.0;
t42 = Dx_CG.*t2.*t5.*2.0;
t43 = Dy_CG.*t3.*t5.*2.0;
t44 = Dy.*Z_f.*t5.*t6.*t8.*2.0;
t45 = Dy.*l_f.*t4.*t7.*t10.*2.0;
t46 = Dy.*Z_f.*t5.*t7.*t8.*2.0;
t47 = Dy.*l_f.*t4.*t6.*t10.*2.0;
t48 = Dp.*Z_f.*t5.*t6.*t10.*t11.*2.0;
t49 = Dy.*Z_f.*t4.*t6.*t9.*t11.*2.0;
t50 = Dp.*T_f.*t5.*t6.*t8.*t10;
t51 = Dr.*T_f.*t5.*t6.*t9.*t11;
t52 = Dy.*T_f.*t4.*t6.*t8.*t9;
t53 = Dp.*Z_f.*t5.*t7.*t10.*t11.*2.0;
t54 = Dy.*Z_f.*t4.*t7.*t9.*t11.*2.0;
t55 = Dp.*T_f.*t5.*t7.*t8.*t10;
t56 = Dr.*T_f.*t5.*t7.*t9.*t11;
t57 = Dy.*T_f.*t4.*t7.*t8.*t9;
t58 = Dx_CG.*t3.*2.0;
t59 = Dr.*Z_r.*t6.*t11.*2.0;
t60 = Dr.*T_r.*t6.*t8;
t61 = Dr.*Z_r.*t7.*t11.*2.0;
t62 = Dy.*l_r.*t6.*t10.*2.0;
t63 = Dr.*T_r.*t7.*t8;
t64 = Dy.*l_r.*t7.*t10.*2.0;
CPV_ = reshape([t13.*(t14+t15+t16+t17+t18+t19+t20+t21+t22+t23+t24+t25+t26+t27+t28-Dy_CG.*t2.*t5.*2.0-Dy.*T_f.*t4.*t6.*t11-Dy.*T_f.*t4.*t7.*t11-Dp.*l_f.*t4.*t6.*t9.*2.0-Dp.*l_f.*t4.*t7.*t9.*2.0-Dy.*l_f.*t5.*t6.*t10.*2.0-Dy.*l_f.*t5.*t7.*t10.*2.0-Dy.*T_f.*t5.*t6.*t8.*t9-Dy.*T_f.*t5.*t7.*t8.*t9-Dr.*Z_f.*t4.*t6.*t8.*t9.*2.0-Dr.*Z_f.*t4.*t7.*t8.*t9.*2.0-Dy.*Z_f.*t5.*t6.*t9.*t11.*2.0-Dy.*Z_f.*t5.*t7.*t9.*t11.*2.0).*(1.0./2.0),t13.*(t41+t42+t43+t44+t45+t46+t47+t48+t49+t50+t51+t52+t53+t54+t55+t56+t57-Dx_CG.*t3.*t4.*2.0-Dr.*T_f.*t4.*t6.*t8-Dr.*T_f.*t4.*t7.*t8-Dy.*T_f.*t5.*t6.*t11-Dy.*T_f.*t5.*t7.*t11-Dr.*Z_f.*t4.*t6.*t11.*2.0-Dr.*Z_f.*t4.*t7.*t11.*2.0-Dp.*l_f.*t5.*t6.*t9.*2.0-Dp.*l_f.*t5.*t7.*t9.*2.0-Dr.*Z_f.*t5.*t6.*t8.*t9.*2.0-Dr.*Z_f.*t5.*t7.*t8.*t9.*2.0).*(1.0./2.0),0.0,t13.*(-t14-t15-t16+t17-t18-t19+t20-t21-t22-t23+t24+t25-t26+t27+t28+Dy_CG.*t2.*t5.*2.0-Dy.*T_f.*t4.*t6.*t11-Dy.*T_f.*t4.*t7.*t11+Dp.*l_f.*t4.*t6.*t9.*2.0+Dp.*l_f.*t4.*t7.*t9.*2.0+Dy.*l_f.*t5.*t6.*t10.*2.0+Dy.*l_f.*t5.*t7.*t10.*2.0-Dy.*T_f.*t5.*t6.*t8.*t9-Dy.*T_f.*t5.*t7.*t8.*t9+Dr.*Z_f.*t4.*t6.*t8.*t9.*2.0+Dr.*Z_f.*t4.*t7.*t8.*t9.*2.0+Dy.*Z_f.*t5.*t6.*t9.*t11.*2.0+Dy.*Z_f.*t5.*t7.*t9.*t11.*2.0).*(-1.0./2.0),t13.*(t41+t42+t43+t44+t45+t46+t47+t48+t49-t50-t51-t52+t53+t54-t55-t56-t57-Dx_CG.*t3.*t4.*2.0+Dr.*T_f.*t4.*t6.*t8+Dr.*T_f.*t4.*t7.*t8+Dy.*T_f.*t5.*t6.*t11+Dy.*T_f.*t5.*t7.*t11-Dr.*Z_f.*t4.*t6.*t11.*2.0-Dr.*Z_f.*t4.*t7.*t11.*2.0-Dp.*l_f.*t5.*t6.*t9.*2.0-Dp.*l_f.*t5.*t7.*t9.*2.0-Dr.*Z_f.*t5.*t6.*t8.*t9.*2.0-Dr.*Z_f.*t5.*t7.*t8.*t9.*2.0).*(1.0./2.0),0.0,t13.*(t29+t30+t31+t32+t33+t34+t35+t36+t37+t38+t39+t40-Dy.*T_r.*t6.*t11-Dy.*T_r.*t7.*t11-Dr.*Z_r.*t6.*t8.*t9.*2.0-Dr.*Z_r.*t7.*t8.*t9.*2.0).*(1.0./2.0),t13.*(t58+t59+t60+t61+t62+t63+t64-Dy_CG.*t2.*2.0-Dy.*T_r.*t6.*t8.*t9-Dy.*T_r.*t7.*t8.*t9-Dy.*Z_r.*t6.*t9.*t11.*2.0-Dy.*Z_r.*t7.*t9.*t11.*2.0).*(-1.0./2.0),0.0,t13.*(t29+t30+t31+t32+t33+t34+t35-t36-t37+t38-t39-t40+Dy.*T_r.*t6.*t11+Dy.*T_r.*t7.*t11-Dr.*Z_r.*t6.*t8.*t9.*2.0-Dr.*Z_r.*t7.*t8.*t9.*2.0).*(1.0./2.0),t13.*(t58+t59-t60+t61+t62-t63+t64-Dy_CG.*t2.*2.0+Dy.*T_r.*t6.*t8.*t9+Dy.*T_r.*t7.*t8.*t9-Dy.*Z_r.*t6.*t9.*t11.*2.0-Dy.*Z_r.*t7.*t9.*t11.*2.0).*(-1.0./2.0),0.0],[3,4]);
