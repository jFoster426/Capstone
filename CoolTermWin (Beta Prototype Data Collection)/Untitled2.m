% Reset configuration.
close all;
clear all;
clc;

% Prompt the user to select the data file that they wish to open.
[file, path] = uigetfile({ '*.csv;*.txt', 'STRASTech Data Files (*.csv,*.txt)'; }, 'Select a Data Set');
fullfilepath = strcat(path, file);

M = csvread(fullfilepath, 1);

time = M(:, 1);

d = zeros(size(time));

% Remove salt and pepper noise for all IMU data.
for j = 2:13
    for i = 2:size(time)-1
        d(i) = M(i, j) - M(i - 1, j);
        if (abs(d(i) - d(i - 1)) > 500)
            M(i, j) = (M(i - 1, j) + M(i + 1, j)) ./ 2;
        end
    end
end

faccx = M(:, 2);
faccx = faccx .* (2.0 / 32768.0);
faccy = M(:, 3);
faccy = faccy .* (2.0 / 32768.0);
faccz = M(:, 4);
faccz = faccz .* (2.0 / 32768.0);
fgyrx = M(:, 5);
fgyrx = fgyrx .* (2000.0 / 32768.0);
fgyry = M(:, 6);
fgyry = fgyry .* (2000.0 / 32768.0);
fgyrz = M(:, 7);
fgyrz = fgyrz .* (2000.0 / 32768.0);

saccx = M(:, 8);
saccx = saccx .* (2.0 / 32768.0);
saccy = M(:, 9);
saccy = saccy .* (2.0 / 32768.0);
saccz = M(:, 10);
saccz = saccz .* (2.0 / 32768.0);
sgyrx = M(:, 11);
sgyrx = sgyrx .* (2000.0 / 32768.0);
sgyry = M(:, 12);
sgyry = sgyry .* (2000.0 / 32768.0);
sgyrz = M(:, 13);
sgyrz = sgyrz .* (2000.0 / 32768.0);

load = M(:, 14);

% Normalize the time to start at 0 ms.
for i = 2:size(time)
    time(i) = time(i) - time(1);
end
time(1) = 0;

% Plot load cell data.
figure(1);
plot(time, -load, 'LineWidth', 2);
title('Torque vs. Time');
xlabel('Time (ms)');
ylabel('Torque About the Ankle (ft-lbs)');

% Initialize variables.
fRoll = zeros(size(time));
fPitch = zeros(size(time));
sRoll = zeros(size(time));
sPitch = zeros(size(time));
fAccRoll = zeros(size(time));
fAccPitch = zeros(size(time));
sAccRoll = zeros(size(time));
sAccPitch = zeros(size(time));

% Calculate angles for accelerometer and gyroscope independently.
for i = 1:size(time)
    % Get foot accelerometer roll and pitch.
    fAccRoll(i) = atan2(faccy(i), faccz(i));
    fAccPitch(i) = atan2(faccx(i), sqrt(faccy(i) .^ 2 + faccz(i) .^ 2));

    % Get shin accelerometer roll and pitch.
    sAccRoll(i) = atan2(saccy(i), saccz(i));
    sAccPitch(i) = atan2(saccx(i), sqrt(saccy(i) .^ 2 + saccz(i) .^ 2));

    if i == 1
        % Set initial roll and pitch according to accelerometer only.
        fRoll(1) = fAccRoll(1);
        fPitch(1) = fAccPitch(1);
        sRoll(1) = sAccRoll(1);
        sPitch(1) = sAccPitch(1);
    else
        % Get gyroscope roll and pitch.
        fRoll(i) = fRoll(i - 1) + fgyrx(i) * (pi / 180) * ((time(i) - time(i - 1)) / 1000);
        fPitch(i) = fPitch(i - 1) + fgyry(i) * (pi / 180) * ((time(i) - time(i - 1)) / 1000);
        sRoll(i) = sRoll(i - 1) + sgyrx(i) * (pi / 180) * ((time(i) - time(i - 1)) / 1000);
        sPitch(i) = sPitch(i - 1) + sgyry(i) * (pi / 180) * ((time(i) - time(i - 1)) / 1000);
    end
end

% Cancel pitch (ROLL) flipping issue -180 / 180
for i = 1:size(time) - 1
    diffSRoll = sAccRoll(i + 1) - sAccRoll(i);
    diffFRoll = fAccRoll(i + 1) - fAccRoll(i);
    if diffSRoll > pi
        for j = i+1:size(time)
            sAccRoll(j) = sAccRoll(j) - 2 * pi;
        end
    end
    if diffSRoll < -pi
        for j = i+1:size(time)
            sAccRoll(j) = sAccRoll(j) + 2 * pi;
        end
    end
    if diffFRoll > pi
        for j = i+1:size(time)
            fAccRoll(j) = fAccRoll(j) - 2 * pi;
        end
    end
    if diffFRoll < -pi
        for j = i+1:size(time)
            fAccRoll(j) = fAccRoll(j) + 2 * pi;
        end
    end
end

% Calculate the joint angle using sensor fusion (complementary filter algorithm).
for i = 1:size(time)
    % Comp filter algorithm.
    a = 1.00;

    while (fRoll(i) - fAccRoll(i) > 360)
        fAccRoll(i) = fAccRoll(i) - 360;
    end
    while (fRoll(i) - fAccRoll(i) < -360)
        fAccRoll(i) = fAccRoll(i) + 360;
    end
    
    fRoll(i) = a * fRoll(i) + (1 - a) * fAccRoll(i);
    fPitch(i) = a * fPitch(i) + (1 - a) * fAccPitch(i);
    sRoll(i) = a * sRoll(i) + (1 - a) * sAccRoll(i);
    sPitch(i) = a * sPitch(i) + (1 - a) * sAccPitch(i);
end

% Convert from radians to degrees.
fPitch = fPitch * 180 / pi;
fRoll = fRoll * 180 / pi;
sPitch = sPitch * 180 / pi;
sRoll = sRoll * 180 / pi;

% Draw the individual pitch and roll angles vs. time plot.
figure(2);
hold on;
plot(time, fRoll, 'Linewidth', 2);
plot(time, fPitch, 'Linewidth', 2);
plot(time, sRoll, 'Linewidth', 2);
plot(time, sPitch, 'Linewidth', 2);
title('Pitch and Roll Axes vs. Time');
legend('Foot IMU Roll', 'Foot IMU Pitch', 'Shin IMU Roll', 'Shin IMU Pitch');
xlabel('Time (ms)');
ylabel('Angle (degrees)');

% Calculate the joint angle in degrees.
roll = fRoll - sRoll;
pitch = fPitch - sPitch;

% Plot the joint angle for both pitch and roll.
figure(3);
plot(time, roll, 'Linewidth', 2, 'Color', 'red');
hold on;
plot(time, pitch, 'Linewidth', 2);
title('Joint Angle vs. Time');
% Note: Pitch and Roll were flipped.
legend('Pitch', 'Roll');
xlabel('Time (ms)');
ylabel('Angle (degrees)');

% Integrate to find the velocity in m/s.
vAngRoll = zeros(size(time));
vAngPitch = zeros(size(time));
for i = 2:size(time)
    vAngPitch(i) = (pitch(i) - pitch(i - 1)) / ((time(i) - time(i - 1)) / 1000);
    vAngRoll(i) = (roll(i) - roll(i - 1)) / ((time(i) - time(i - 1)) / 1000);
end

% Plot the angular velocity for both pitch and roll.
figure(4);
plot(time, vAngRoll, 'Linewidth', 2);
hold on;
plot(time, vAngPitch, 'Linewidth', 2);
vAngPitch = lowpass(vAngPitch, 0.002);
vAngRoll = lowpass(vAngRoll, 0.002);
plot(time, vAngRoll, 'Linewidth', 2);
hold on;
plot(time, vAngPitch, 'Linewidth', 2);
title('Angular Velocity vs. Time');
% Note: Pitch and Roll were flipped.
legend('Pitch', 'Roll', 'Pitch (Filtered)', 'Roll (Filtered)');
xlabel('Time (ms)');
ylabel('Angular Velocity (degrees/s)');
