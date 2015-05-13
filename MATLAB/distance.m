AMOUNT_OF_ROBOTS = 6;
NOMINAL_DISTANCE = 0.3;

% Rectangle properties
MIN_X = -0.2;
MIN_Y = -0.2;
MAX_X = 0.2;
MAX_Y = 0.2;

% Import the data
data = importdata('experiment.txt');
data = data(:,2:end);

% Keep only the needed column of the matrix
index = 1:AMOUNT_OF_ROBOTS*7;
index = mod(index-5,7) == 0 | mod(index-6,7) == 0; % 1 where the column must be kept
data = data(:,index); % The matrix rows now only contains the needed data: x1,y1,x2,y2,...,xn,yn.

% Loop
xIndices = mod(1:AMOUNT_OF_ROBOTS*2, 2) == 0;
yIndices = mod(1:AMOUNT_OF_ROBOTS*2, 2) == 1;
results = zeros(1,size(data,1));

for t = 1:size(data,1) % For every time step (to be reduced after!)

    x = data(t, xIndices);
    y = data(t, yIndices);
    
    distancesToCenter = sqrt(x.^2 + y.^2); % Distance to center for all robots at time step t.
    
    % Compute the distance from the rectangle
    % http://stackoverflow.com/questions/5254838/calculating-distance-between-a-point-and-a-rectangular-box-nearest-point
    dx = max([MIN_X - x; zeros(AMOUNT_OF_ROBOTS); x - MAX_X]);
    dy = max([MIN_Y - y; zeros(AMOUNT_OF_ROBOTS); y - MAX_Y]);
    distancesToRectangle = sqrt(dx.^2 + dy.^2);
    
    % Compute the error value for this time step
    relErrors = abs((distancesToRectangle - NOMINAL_DISTANCE)/NOMINAL_DISTANCE);
    results(t) = mean(relErrors);
end

plot(1:size(data,1), results)
