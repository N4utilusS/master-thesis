PLOT_RESOLUTION = 30;
diMatrix = zeros(1,PLOT_RESOLUTION);
deMatrix = zeros(1,PLOT_RESOLUTION);
NAME_BASE = 'random';
MIN_NUMBER = 1;
MAX_NUMBER = 10;

areaMetricVector = zeros(1,MAX_NUMBER-MIN_NUMBER+1);

for i = MIN_NUMBER:MAX_NUMBER
    name = [NAME_BASE num2str(i) '_corrected.txt'];
    distance
    diMatrix(i,:) = results;
    areaMetricVector(i-MIN_NUMBER+1) = areaMetric;
    density
    deMatrix(i,:) = results;
end

figure(1)
boxplot(diMatrix, (1:PLOT_RESOLUTION)/PLOT_RESOLUTION*180,'labelorientation','inline')
title('Distance Error')
xlabel('Time (s)')
ylabel('Error')


figure(2)
boxplot(deMatrix, (1:PLOT_RESOLUTION)/PLOT_RESOLUTION*180,'labelorientation','inline')
title('Density Error')
xlabel('Time (s)')
ylabel('Error')


figure(3)
boxplot(areaMetricVector)
title('Operational Area Ratio')
ylabel('Ratio')