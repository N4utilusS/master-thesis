clear all
AMOUNT_OF_ROBOTS = 8;
SMOOTH = true;
SMOOTH_INTENSITY = 5;
TAGS = [8 9 3 2 6 4 5 7];
TAGS = sort(TAGS);

display('==> Importing data')
data = importdata('results4.txt');
stepData = zeros(1,AMOUNT_OF_ROBOTS*7);

display('==> Processing data')
for t = 1:size(data,1)
    for n = 2:7:size(data,2)
        index = find(data(t,n) == TAGS)-1;
        if ~isempty(index)
            stepData(index*7 + (1:7)) = data(t,n:n+6);
        end
    end
    data(t,2:end) = stepData;
end

if SMOOTH
    display('==> Smoothing data')
    for t = SMOOTH_INTENSITY:size(data,1)
        data(t,:) = mean(data(t-SMOOTH_INTENSITY+1:t,:),1);
    end
end

display('==> Saving data')
dlmwrite('myFile.txt',data,'delimiter',' ')

display('==> DONE')