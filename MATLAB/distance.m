%clear all
AMOUNT_OF_ROBOTS = 8;
NOMINAL_DISTANCE = 0.35;
DISTANCE_RATIO = 0.43;
TRANSLATE_X = 0.734;
TRANSLATE_Y = 3.87;
PLOT_RESOLUTION = 30;

% Rectangle properties
MIN_X = -0.1575;
MIN_Y = -0.19;
MAX_X = 0.1575;
MAX_Y = 0.19;

% Import the data
data = importdata(name);
%loadData
data = data(:,2:end);

% Keep only the needed column of the matrix
index = 1:AMOUNT_OF_ROBOTS*7;
index = mod(index-5,7) == 0 | mod(index-6,7) == 0; % 1 where the column must be kept
data = data(:,index); % The matrix rows now only contains the needed data: x1,y1,x2,y2,...,xn,yn.

% Loop
xIndices = mod(1:AMOUNT_OF_ROBOTS*2, 2) == 1;
yIndices = mod(1:AMOUNT_OF_ROBOTS*2, 2) == 0;

x = (data(:, xIndices) - TRANSLATE_X) * DISTANCE_RATIO;
y = (data(:, yIndices) - TRANSLATE_Y) * DISTANCE_RATIO;

distancesToCenter = sqrt(x.^2 + y.^2); % Distance to center for all robots at time step t.

% Compute the distance from the rectangle
% http://stackoverflow.com/questions/5254838/calculating-distance-between-a-point-and-a-rectangular-box-nearest-point
dx = cat(3, MIN_X - x, zeros(size(x,1),AMOUNT_OF_ROBOTS), x - MAX_X);
dy = cat(3, MIN_Y - y, zeros(size(x,1),AMOUNT_OF_ROBOTS), y - MAX_Y);

dx = max(dx, [], 3);
dy = max(dy, [], 3);
distancesToRectangle = sqrt(dx.^2 + dy.^2);

% Area metric
areaMetric = sum(abs(distancesToRectangle(end,:) - NOMINAL_DISTANCE) <= 0.05)/AMOUNT_OF_ROBOTS;

% Compute the error value for this time step
relErrors = abs((distancesToRectangle - NOMINAL_DISTANCE)/NOMINAL_DISTANCE);
tempResults = mean(relErrors, 2);

% Downscale to 180 ticks (3 minutes, 1 tick per second)
results = zeros(1,PLOT_RESOLUTION);
newIndices = (1:length(tempResults)) / length(tempResults) * PLOT_RESOLUTION; % Put linspace instead?
maxi = 1;
counter = 0;

for j = 1:length(tempResults)
    if newIndices(j) <= maxi
        results(maxi) = results(maxi) + tempResults(j);
        counter = counter + 1;
    else
        results(maxi) = results(maxi)/counter;
        counter = 0;
        maxi = maxi + 1;
        
        results(maxi) = results(maxi) + tempResults(j);
        counter = counter + 1;
    end
end

results(end) = results(end)/counter;

plot(1:length(results), results)
title('Distance Error')
xlabel('Time (s)')
ylabel('Error')