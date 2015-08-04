PLOT_RESOLUTION = 30;
diMatrix = zeros(1,PLOT_RESOLUTION);
deMatrix = zeros(1,PLOT_RESOLUTION);
NAME_BASE = 'geof';
MIN_NUMBER = 1;
MAX_NUMBER = 10;

%areaMetricVector = zeros(1,MAX_NUMBER-MIN_NUMBER+1);

for i = MIN_NUMBER:MAX_NUMBER
    name = [NAME_BASE num2str(i) '_corrected.txt'];
    distance
    diMatrix(i-MIN_NUMBER+1,:) = results;
    %areaMetricVector(i-MIN_NUMBER+1) = areaMetric;
    density
    deMatrix(i-MIN_NUMBER+1,:) = results;
end

figure(1)
boxplot(diMatrix, (1:PLOT_RESOLUTION)/PLOT_RESOLUTION*180,'labelorientation','inline','notch','off')
title('Distance Error')
xlabel('Time (s)')
ylabel('Error')
axis([0 31 0 2.5])


figure(2)
boxplot(deMatrix, (1:PLOT_RESOLUTION)/PLOT_RESOLUTION*180,'labelorientation','inline','notch','off')
title('Density Error')
xlabel('Time (s)')
ylabel('Error')
axis([0 31 0 2.5])

% figure(3)
% boxplot(areaMetricVector)
% title('Operational Area Ratio')
% ylabel('Ratio')