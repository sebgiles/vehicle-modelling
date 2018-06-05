function viewmodels(folder)
SA_x =  -pi/4 : pi/12 : pi/4;
SL_x =   -1.2 :  0.05 :    4;

SA_y =  -pi/4 : 0.025  : pi/4;
SL_y =   [-1 -0.6 -0.2 -0.1 0 0.1 0.2 0.3 0.6];

FZ =      0 :  700  : 2100;
FZ(1) = 0.0001;

files = split(ls(folder));

for i = 1:length(files)-1
    
    try
        T = loadTIR([folder,'/',files{i}]);
    catch
        fprintf('failed to load\n');
        continue
    end
    
    clf
    
    for j = 1:length(FZ)
        
        subplot(length(FZ),2,2*j-1);
        plot([])
        hold on
        title([files{i}, ' - FZ = ', num2str(FZ(j)), ' N - Long']);
        for k = 1:length(SA_x)
            F_x = C310pacejka02(SL_x, SA_x(k), 0, FZ(j), T);
            plot(SL_x, F_x);
        end
        grid on
        legend(split(num2str(rad2deg(SA_x))))
        
        subplot(length(FZ),2,2*j);
        plot([])
        hold on
        title([files{i}, ' - FZ = ', num2str(FZ(j)), ' N - Lat']);
        for h = 1:length(SL_y)
            [~, F_y] = C310pacejka02(SL_y(h), SA_y, 0, FZ(j), T);
            plot(rad2deg(SA_y), F_y);
        end
        grid on
        legend(split(num2str(SL_y)))
        
    end
    pause
    
end
close
end