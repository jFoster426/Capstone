%% File format.
% time,faccx,faccy,faccz,fgyrx,fgyry,fgyrz,saccx,saccy,saccz,sgyrx,sgyry,sgyrz,load
% 1    2     3     4     5     6     7     8     9     10    11    12    13    14

%% Clear everything and restart.
close all;
clear;
clc;

%% Initialize Arduino in Serial comms.
inputStr = "";
i = 6;
s = 0;

while i > 5
    i = 0;
    s = serialport('COM7', 250000);
    s.Timeout = 1;
    while strcmp(inputStr, "Completed initialization.") == 0
        inputStr = readline(s);
        disp(inputStr);
        i = i + 1;
        if i > 5
            clear s;
            disp("Retrying...");
            break;
        end
    end
    pause(1);
end

disp("Arduino initialized successfully.");

%% Open main application window.
fh = figure(1);
fh.WindowState = 'maximized';
statusTextAH = axes(fh, 'position', [0, 0, 0.3, 0.1]);
statusTxt = text(statusTextAH, 0.2, 0.2, "Arduino initialized successfully.", 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', 'FontSize', 24);
statusTextAH.Visible = 'off';
pause(2);
delete(statusTxt);
statusTxt = text(statusTextAH, 0.2, 0.2, "Measurement not started.", 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', 'FontSize', 24);

%% Wait for measurement start.
while strcmp(inputStr, "START") == 0
    % Wait for valid start.
    inputStr = readline(s);
end

delete(statusTxt);
statusTxt = text(statusTextAH, 0.2, 0.2, "Measurement in progress.", 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', 'FontSize', 24);

initialMeasurement = 1;

%% Create plot showing the normal vectors.
normalVectorAH = axes('Position', [0.05, 0.55, 0.4, 0.4]);
pt = [0 0 0];
dir = [0 0 1 1];
h1 = quiver3(pt(1), pt(2), pt(3), dir(1), dir(2), dir(3), 'LineWidth', 2);
hold on;
h2 = quiver3(pt(1), pt(2), pt(3), dir(1), dir(2), dir(3), 'LineWidth', 2);
xlim([-1 1])
ylim([-1 1])
zlim([-1 1])
title("Live IMU Normal Vectors");

% Init variable to store last measurement time.
oldTime = 0;
sRoll = 0;
sPitch = 0;

% Allocate memory for all the data.
MEAS_SIZE = 100000;
inputData = zeros(MEAS_SIZE, 14);

for bufIdx = 1:MEAS_SIZE
    % Read and parse the input data (Read most recent data).
    while s.NumBytesAvailable > 0
        inputStr = readline(s);
    end
    splitStr = split(inputStr, ',');
    inputData(bufIdx, :) = str2double(splitStr)';

    currentData = inputData(bufIdx, :);

    % Break if measurement is completed.
    if strcmp(inputStr, "END") == 1
        break;
    end

    % Discard blank lines.
    if size(currentData) < 14
        disp("Blank line.");
        continue;
    end
    
    % Discard invalid lines.
    if isnan(currentData)
        disp("Invalid line.");
        continue;
    end

    % Get foot accelerometer roll and pitch.
    if currentData(4) > 0
        fAccRoll = atan2(currentData(3), currentData(4));
        fAccPitch = atan2(-currentData(2), sqrt(currentData(3) .^ 2 + currentData(4) .^ 2));
    else
        fAccRoll = atan2(currentData(3), currentData(4));
        fAccPitch = atan2(currentData(2), sqrt(currentData(3) .^ 2 + currentData(4) .^ 2));
    end

    % Get shin accelerometer roll and pitch.
    if currentData(10) > 0
        sAccRoll = atan2(currentData(9), currentData(10));
        sAccPitch = atan2(-currentData(8), sqrt(currentData(9) .^ 2 + currentData(10) .^ 2));
    else
        sAccRoll = atan2(currentData(9), currentData(10));
        sAccPitch = atan2(currentData(8), sqrt(currentData(9) .^ 2 + currentData(10) .^ 2));
    end

    % Set the normal vectors properly on the initial measurement to avoid initial calbration.
    if initialMeasurement == 1
        initialMeasurement = 0;
        fRoll = fAccRoll;
        fPitch = fAccPitch;
        sRoll = sAccRoll;
        sPitch = sAccPitch;
    end

    % Get foot gyroscope roll and pitch.
    fRoll = fRoll + currentData(5) * (pi / 180) * ((currentData(1) - oldTime) / 1000);
    fPitch = fPitch + currentData(5) * (pi / 180) * ((currentData(1) - oldTime) / 1000);

    % Get shin gyroscope roll and pitch.
    sRoll = sRoll + currentData(11) * (pi / 180) * ((currentData(1) - oldTime) / 1000);
    sPitch = sPitch + currentData(12) * (pi / 180) * ((currentData(1) - oldTime) / 1000);

    % Comp filter algorithm.
    fRoll = 0.98 * fRoll + 0.02 * fAccRoll;
    fPitch = 0.98 * fPitch + 0.02 * fAccPitch;
    sRoll = 0.98 * sRoll + 0.02 * sAccRoll;
    sPitch = 0.98 * sPitch + 0.02 * sAccPitch;

    % Draw the vectors.
    fxfm = makehgtform('xrotate', fRoll, 'yrotate', fPitch);
    sxfm = makehgtform('xrotate', sRoll, 'yrotate', sPitch);
    fnewdir = fxfm * dir';
    snewdir = sxfm * dir';
    h1.UData = fnewdir(1);
    h2.UData = snewdir(1);
    h1.VData = fnewdir(2);
    h2.VData = snewdir(2);
    h1.WData = fnewdir(3);
    h2.WData = snewdir(3);
    drawnow;
    % Store the last measurement time.
    oldTime = currentData(1);
end

%% Measurement has been completed.
delete(statusTxt);
statusTxt = text(statusTextAH, 0.2, 0.2, "Measurement is complete.", 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', 'FontSize', 24);

pause(1);
close all;

%% Post-analysis plots.
