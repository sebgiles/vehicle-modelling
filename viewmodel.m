function viewmodel(file)
SA_x =  pi/180*[0 1 2 5 10 20];
SL_x =   -1 :  0.001 :    0.2;

SA_y =  pi/180* (0:0.1:20);
SL_y =   [-1 -0.5 -0.1 0 0.1 0.3];

FZ   = 700;


T = loadTIR(file);

for j = 1:length(FZ)
    
    subplot(length(FZ),2,2*j-1);
    plot([])
    hold on
    title(['FZ = ', num2str(FZ(j)), ' N - Long']);
    for k = 1:length(SA_x)
        F_x = C310pacejka02(SL_x, SA_x(k), 0, FZ(j), T);
        plot(SL_x, F_x);
    end
    grid on
    legend(split(num2str(rad2deg(SA_x))))
    
    subplot(length(FZ),2,2*j);
    plot([])
    hold on
    title(['FZ = ', num2str(FZ(j)), ' N - Lat']);
    for h = 1:length(SL_y)
        [~, F_y] = C310pacejka02(SL_y(h), SA_y, 0, FZ(j), T);
        plot(rad2deg(SA_y), -F_y);
    end
    grid on
    legend(split(num2str(SL_y)))
    
end
pause


end