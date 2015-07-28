%clear all
clear data stepData index
AMOUNT_OF_ROBOTS = 8;
SMOOTH = false; % Take average of values on several timesteps.
SMOOTH_INTENSITY = 7; % Amount of timestep to make average.
TAGS = [8 9 3 2 6 4 5 7]; % Fill with tags of robots used (ATS tags).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
TAGS = sort(TAGS);
%name = 'geolong';
display([name ' ==> Importing data'])
%data = importdata([name '.txt'],' ');
loadData
stepData = zeros(1,AMOUNT_OF_ROBOTS*7);

display([name ' ==> Processing data'])
for t = 1:size(data,1)
    for n = 2:7:size(data,2)
        index = find(data(t,n) == TAGS)-1; % index (0->#tag-1) of the tag in the tag vector, empty matrix if not in tag vector
        if ~isempty(index)
            stepData(index*7 + (1:7)) = data(t,n:n+6); % replace previous values
        end
    end
    data(t,2:end) = stepData; % put corrected values inside final matrix
end

if SMOOTH
    display([name ' ==> Smoothing data'])
    for t = size(data,1):-1:1
        data(t,:) = mean(data(max(t-SMOOTH_INTENSITY+1,1):t,:),1);
    end
end

display([name ' ==> Saving data'])
dlmwrite([name '_corrected.txt'],data,'delimiter',' ')

display([name ' ==> DONE'])