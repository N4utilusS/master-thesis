%clear all
AMOUNT_OF_ROBOTS = 8;
DISTANCE_RATIO = 0.43;
TRANSLATE_X = 0.734;
TRANSLATE_Y = 3.87;
PLOT_RESOLUTION = 30;

% Import the data
data = importdata(name);
%loadData
data = data(:,2:end);

% Keep only the needed column of the matrix
index = 1:AMOUNT_OF_ROBOTS*7;
index = mod(index-5,7) == 0 | mod(index-6,7) == 0; % 1 where the column must be kept
data = data(:,index); % The matrix rows now only contains the needed data: x1,y1,x2,y2,...,xn,yn.

% Compute the reference angle
refAngle = 2*pi/AMOUNT_OF_ROBOTS; % in radians

% Get the absolute angles
xIndices = mod(1:AMOUNT_OF_ROBOTS*2, 2) == 1;
yIndices = mod(1:AMOUNT_OF_ROBOTS*2, 2) == 0;

x = (data(:, xIndices) - TRANSLATE_X) * DISTANCE_RATIO;
y = (data(:, yIndices) - TRANSLATE_Y) * DISTANCE_RATIO;
% 
% x = [0.5 * cos((1:AMOUNT_OF_ROBOTS) * 2*pi/AMOUNT_OF_ROBOTS); 0.5 * cos((1:AMOUNT_OF_ROBOTS) * 2*pi/AMOUNT_OF_ROBOTS)];
% y = [0.5 * sin((1:AMOUNT_OF_ROBOTS) * 2*pi/AMOUNT_OF_ROBOTS); 0.5 * sin((1:AMOUNT_OF_ROBOTS) * 2*pi/AMOUNT_OF_ROBOTS)];
% 
% x = [0 1];
% y = [1 0];

absoluteAngles = atan2(y,x);
absoluteAngles = sort(absoluteAngles, 2);

% Relative angles
relativeAngles = [diff(absoluteAngles, 1, 2) absoluteAngles(:,1)+2*pi-absoluteAngles(:,end)];

relErrors = abs((relativeAngles - refAngle)/refAngle);
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
title('Density Error')
xlabel('Time (s)')
ylabel('Error')