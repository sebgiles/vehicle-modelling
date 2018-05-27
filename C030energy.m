% translational kinetic energy
T_trans = 1/2*m*(v_CG.')*v_CG;

% Inertia Tensor
I = [Ixx   0 Ixz;
       0 Iyy   0;
     Ixz   0 Izz];
% rotational kinetic energy
T_rot = 1/2*W.'*I*W;

% total kinetic energy
T = T_rot + T_trans;

% energy stored in suspension springs
V_spring = 1/2 * k_f * (d_fr^2 + d_fl^2) ...
         + 1/2 * k_r * (d_rl^2 + d_rr^2);

% gravitational potential energy
V_g = -m*g*z_CG;

% Total potential energy
V = V_spring + V_g;

% damper-dissipated energy (Rayleigh dissipation function)
D = 1/2 * b_f * Dd_fr^2 ...
  + 1/2 * b_f * Dd_fl^2 ...
  + 1/2 * b_r * Dd_rr^2 ...
  + 1/2 * b_r * Dd_rl^2;

%T=simplify(T);
%V=simplify(V);
