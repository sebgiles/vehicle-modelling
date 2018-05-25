usesym = 1;
linear = 1;


if usesym

    %-------- KINEMATICS PARAMETERS --------------------------------

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

else

    % parameter values, all units are metric
    g    = 9.81;   % gravitational acceleration
    T_f  = 1.5 ;   % front track
    T_r  = 1.5 ;   % rear track
    l_r  =  0.5;   % CG to rear axle
    l_f  =    1;   % CG to front axle
    Z_f  = 0.3 ;   % CG height wrt front axle
    Z_r  = 0.3 ;   % CG height wrt rear axle
    h_CG = 0.4 ;   % CG height wrt ground
    m    = 320 ;   % vehicle sprung mass with driver
    Ixx  = 30  ;   % moment of inertia about vehicle x axis
    Iyy  = 60  ;   % moment of inertia about vehicle y axis
    Izz =  100 ;   % moment of inertia about vehicle z axis
    Ixz =  0   ;   % product of inertia between xz vehicle axes
    k_f = 50000;   % front spring stiffness
    k_r = 80000;   % rear spring stiffness
    b_f = 2000 ;   % front damping rate
    b_r = 2000 ;   % rear damping rate

end
