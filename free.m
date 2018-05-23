freerolling = abs(SL) <= 0.005;
nocamber = IA < 0.01;
nosteer = SA < 0.001;

stationary = abs(dN) <= 0.01;
st = abs(dSA) <= 0.0002;
fr = nocamber&nosteer&freerolling&st;


%calcola RL come proporzionale a FZ
%calcola lo SR alla TIRF
%calcola RE alla TIRF
%caloclar R0

% serve la relazione R0 = RE + f(FZ)
% poi SL = R0*N/Vx-1
