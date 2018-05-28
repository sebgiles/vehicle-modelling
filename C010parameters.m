% track widths
syms T_f T_r

% axle positions wrt body frame
% "0=CG, (x,y,z) -> (forward,right,down)"
syms l_f l_r
syms Z_f Z_r

% CG no load height (negative)
syms h_CG

% spring coefficients
syms k_f k_r

% damper coefficients
syms b_f b_r
% ----- DYNAMIC PARAMETERS ------------------------------------------------
% gravitational acceleration
syms g
% vehicle mass
syms m
% vehicle inertia tensor wrt body frame
syms Ixx Iyy Izz Ixz

params = [T_f; T_r; l_f; l_r; Z_f; Z_r; h_CG; k_f; k_r; b_f; b_r; g; m;
              Ixx; Iyy; Izz; Ixz];