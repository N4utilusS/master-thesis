clear all
NAME_BASE = 'geo';
MIN_NUMBER = 1;
MAX_NUMBER = 5;

for i = MIN_NUMBER:MAX_NUMBER
    name = [NAME_BASE num2str(i)];
    repairResultFile
end