close all;
clear;
clc;

s = serialport('COM7', 250000);

for i = 1:10
    inputStr = readline(s);
end

for i = 1:1:10000
    % Parse input data.
    while s.NumBytesAvailable > 0
        inputStr = readline(s);
    end
    splitStr = split(inputStr, ',');
    inputData = str2double(splitStr);

    % Draw the vector.
    smag = sqrt(inputData(17) .^ 2 + inputData(18) .^ 2);
    scatter(inputData(17), inputData(18), 'k');
    hold on;
end
clear s;