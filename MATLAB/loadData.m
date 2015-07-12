%http://www.mathworks.com/matlabcentral/answers/54068-how-to-read-data-file-with-rows-of-different-length
%AMOUNT_OF_ROBOTS = 8;
fid = fopen([name '.txt']);
textLine = fgets(fid); % Read first line.
lineCounter = 1;

columnsAmount = AMOUNT_OF_ROBOTS * 7 + 1;
while ischar(textLine)
    %fprintf('\nLine #%d of text = %s\n', lineCounter, textLine);
	% get into numbers array.
	numbers = sscanf(textLine, '%f ');
    numbersMax = min(columnsAmount, length(numbers));
	data(lineCounter,:) = [numbers(1:numbersMax)' -ones(1, columnsAmount-length(numbers))];
	
	% Read the next line.
    textLine = fgets(fid);
	lineCounter = lineCounter + 1;
end
fclose(fid);