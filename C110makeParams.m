function parameters = makeParams(filename)

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


parameters = [T_f; T_r; l_f; l_r; Z_f; Z_r; h_CG; k_f; k_r; b_f; b_r; g; m;
    Ixx; Iyy; Izz; Ixz];

%% Save to file
% If filename is specified then save parameters to file for use with
% CarLib Simulink models
if nargin > 0
    if (filename(end-3:end) ~= '.mat')
        filename = [filename,'.mat'];
    end
    save(filename, 'parameters');
end