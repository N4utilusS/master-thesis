clear all
NAME_BASE = 'results';
MIN_NUMBER = 0;
MAX_NUMBER = 10;

for i = MIN_NUMBER:MAX_NUMBER
    name = [NAME_BASE num2str(i)];
    repairResultFile
end