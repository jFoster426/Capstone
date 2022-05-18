close all;
clear;
clc;

s = serialport('COM7', 250000);

for i = 1:10
    inputStr = readline(s);
end

pt = [0 0 0];
dir = [0 0 1 1];
h1 = quiver3(pt(1),pt(2),pt(3), dir(1),dir(2),dir(3),'LineWidth',2);
hold on;
h2 = quiver3(pt(1),pt(2),pt(3), dir(1),dir(2),dir(3));
xlim([-1 1])
ylim([-1 1])
zlim([-1 1])

% Init variable to store last measurement time.
oldTime = 0;
roll = 0;
pitch = 0;

for i = 1:1:10000
    % Parse input data.
    while s.NumBytesAvailable > 0
        inputStr = readline(s);
    end
    splitStr = split(inputStr, ',');
    inputData = str2double(splitStr);
    % Get accelerometer roll and pitch.

    if (inputData(10) < 0)
        accRoll = atan(inputData(9) / sqrt(inputData(8) .^ 2 + inputData(10) .^ 2)) * (pi / 180);
        accPitch = -atan(inputData(8) / sqrt(inputData(9) .^ 2 + inputData(10) .^ 2)) * (pi / 180);
        accRoll = (pi)*(pi/180)-accRoll;
    else
        accRoll = atan(inputData(9) / sqrt(inputData(8) .^ 2 + inputData(10) .^ 2)) * (pi / 180);
        accPitch = -atan(inputData(8) / sqrt(inputData(9) .^ 2 + inputData(10) .^ 2)) * (pi / 180);
    end
    % Get gyroscope roll and pitch.
    roll = roll + inputData(11) * (pi / 180) * ((inputData(1) - oldTime) / 1000);
    pitch = pitch + inputData(12) * (pi / 180) * ((inputData(1) - oldTime) / 1000);

    % Comp filter algorithm.
    roll = 0.98 * roll + 0.02 * (accRoll * 180 / pi);
    pitch = 0.98 * pitch + 0.02 * (accPitch * 180 / pi);

    disp(accRoll);

    % Draw the vectors.
    xfm1 = makehgtform('xrotate', roll, 'yrotate', pitch);
    xfm2 = makehgtform('xrotate', accRoll*180/pi,'yrotate', accPitch*180/pi);
%     xfm2 = makehgtform('xrotate',inputData(2) * pi/2,'yrotate',inputData(3) * pi/2);
    newdir1 = xfm1 * dir';
    newdir2 = xfm2 * dir';
    h1.UData = newdir1(1);
    h2.UData = newdir2(1);
    h1.VData = newdir1(2);
    h2.VData = newdir2(2);
    h1.WData = newdir1(3);
    h2.WData = newdir2(3);
    drawnow;
    % Store the last measurement time.
    oldTime = inputData(1);
end

clear s;