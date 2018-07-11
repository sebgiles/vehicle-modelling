function toolbox(Mb)
syms yaw pitch roll
syms M_yaw_yaw    M_yaw_pitch    M_yaw_roll
syms M_pitch_yaw  M_pitch_pitch  M_pitch_roll
syms M_roll_yaw   M_roll_pitch   M_roll_roll
syms m
M11 = [M_yaw_yaw ,   M_yaw_pitch,    M_yaw_roll
    M_pitch_yaw , M_pitch_pitch , M_pitch_roll
    M_roll_yaw  , M_roll_pitch  , M_roll_roll];
for i = 1:3
    for j =1:3
        disp( latex(M11(i,j)==Mb(i,j)));
    end
end