filename = fileread('filename.txt');

%% Open the input csv file.
%Import the options of the csv file
opts = detectImportOptions(filename);
%Defines the row location of channel variable name
opts.VariableNamesLine = 1;
%Specifies that the data is comma seperated
opts.Delimiter = ','; %Specifies that the data is comma seperated

T = readtable(filename, opts, 'ReadVariableNames', true);

%% Parse the input table.
time = table2array(T(:, "time"));

faccx = table2array(T(:,"faccx"));
faccy = table2array(T(:,"faccy"));
faccz = table2array(T(:,"faccz"));
fgyrx = table2array(T(:,"fgyrx"));
fgyry = table2array(T(:,"fgyry"));
fgyrz = table2array(T(:,"fgyrz"));

saccx = table2array(T(:,"saccx"));
saccy = table2array(T(:,"saccy"));
saccz = table2array(T(:,"saccz"));
sgyrx = table2array(T(:,"sgyrx"));
sgyry = table2array(T(:,"sgyry"));
sgyrz = table2array(T(:,"sgyrz"));

load = table2array(T(:, "load"));

%% Normalize the time to start at 0 ms.
for i = 2:size(time)
    time(i) = time(i) - time(1);
end
time(1) = 0;

%% Fix the load cell data to remove the jump.
for i = 1:size(load)
    if (load(i) > 8388608)
        load(i) = load(i) - 8388608;
    else
        load(i) = load(i) + 8388608;
    end
end

%% Calibrate the load cell data to measure torque in ft-lbs
loadCellPounds = (load - 8065000) ./ 24400;

momentArm = 5/12; % feet, moment arm along base of foot plate
torque = momentArm*loadCellPounds; % Calculated torque in ft-lbs

calibratedLoad = torque;

%% Draw the torque vs. time plot.
fh = figure(1);
fh.WindowState = 'maximized';
tiledlayout(2, 2);
nexttile;
p = plot(time, -calibratedLoad, 'LineWidth', 2);
p.Color = '#A2142F';
title("Torque vs. Time");
xlabel("Time (ms)");
ylabel("Torque About the Ankle (ft-lbs)");

fRoll = zeros(size(time));
fPitch = zeros(size(time));
sRoll = zeros(size(time));
sPitch = zeros(size(time));

%% Calculate the joint angle using the IMU data.
for i = 1:size(time)
    % Get foot accelerometer roll and pitch.
    if faccz(i) > 0
        fAccRoll = atan2(faccy(i), faccz(i));
        fAccPitch = atan2(-faccx(i), sqrt(faccy(i) .^ 2 + faccz(i) .^ 2));
    else
        fAccRoll = atan2(faccy(i), faccz(i));
        fAccPitch = atan2(faccx(i), sqrt(faccy(i) .^ 2 + faccz(i) .^ 2));
    end

    % Get shin accelerometer roll and pitch.
    if saccz(i) > 0
        sAccRoll = atan2(saccy(i), saccz(i));
        sAccPitch = atan2(-saccx(i), sqrt(saccy(i) .^ 2 + saccz(i) .^ 2));
    else
        sAccRoll = atan2(saccy(i), saccz(i));
        sAccPitch = atan2(saccx(i), sqrt(saccy(i) .^ 2 + saccz(i) .^ 2));
    end

    if i == 1
        % Set initial roll and pitch according to accelerometer only.
        fRoll(1) = fAccRoll;
        fPitch(1) = fAccPitch;
        sRoll(1) = sAccRoll;
        sPitch(1) = sAccPitch;
    else
        % Get gyroscope roll and pitch.
        fRoll(i) = fRoll(i - 1) + fgyrx(i) * (pi / 180) * ((time(i) - time(i - 1)) / 1000);
        fPitch(i) = fPitch(i - 1) + fgyry(i) * (pi / 180) * ((time(i) - time(i - 1)) / 1000);
        sRoll(i) = sRoll(i - 1) + sgyrx(i) * (pi / 180) * ((time(i) - time(i - 1)) / 1000);
        sPitch(i) = sPitch(i - 1) + sgyry(i) * (pi / 180) * ((time(i) - time(i - 1)) / 1000);
    end

    % Comp filter algorithm.
    fRoll(i) = 0.98 * fRoll(i) + 0.02 * fAccRoll;
    fPitch(i) = 0.98 * fPitch(i) + 0.02 * fAccPitch;
    sRoll(i) = 0.98 * sRoll(i) + 0.02 * sAccRoll;
    sPitch(i) = 0.98 * sPitch(i) + 0.02 * sAccPitch;
end

% Convert from radians to degrees.
fPitch = fPitch * 180 / pi;
fRoll = fRoll * 180 / pi;
sPitch = sPitch * 180 / pi;
sRoll = sRoll * 180 / pi;

fRoll = fRoll + 90;
sRoll = sRoll - 90;

%% Draw the individual pitch and roll angles vs. time plot.
nexttile;
p = plot(time, fRoll, 'Linewidth', 2);
p.Color = '#0072BD';
hold on;
p = plot(time, fPitch, 'Linewidth', 2);
p.Color = '#A2142F';
p = plot(time, sRoll, 'Linewidth', 2);
p.Color = '#D95319';
p = plot(time, sPitch, 'Linewidth', 2);
p.Color = '#4DBEEE';
title("Pitch and Roll Axes vs. Time");
% Note: Pitch and Roll were flipped.
legend("Foot IMU Pitch", "Foot IMU Roll", "Shin IMU Pitch", "Shin IMU Roll");
xlabel("Time (ms)");
ylabel("Angle (degrees)");

%% Calculate the joint angle in degrees.
roll = fRoll - sRoll; % SHIT CHANGING HERE
pitch = fPitch - sPitch;

%% Plot the joint angle for both pitch and roll.
nexttile;
% Note: Sign flip.
p = plot(time, -roll, 'Linewidth', 2);
p.Color = '#0072BD';
hold on;
% Note: Sign flip.
p = plot(time, -pitch, 'Linewidth', 2);
p.Color = '#A2142F';
title("Joint Angle vs. Time");
% Note: Pitch and Roll were flipped.
legend("Pitch", "Roll");
xlabel("Time (ms)");
ylabel("Angle (degrees)");

%% Integrate to find the velocity in m/s.
vAngRoll = zeros(size(time));
vAngPitch = zeros(size(time));
for i = 2:size(time)
    vAngPitch(i) = (pitch(i) - pitch(i - 1)) / ((time(i) - time(i - 1)) / 1000);
    vAngRoll(i) = (roll(i) - roll(i - 1)) / ((time(i) - time(i - 1)) / 1000);
end

%% Plot the angular velocity for both pitch and roll.
nexttile;
p = plot(time, vAngRoll, 'Linewidth', 1);
p.Color = '#999999';
hold on;
p = plot(time, vAngPitch, 'Linewidth', 1);
p.Color = '#999999';
vAngPitch = lowpass(vAngPitch, 0.003);
vAngRoll = lowpass(vAngRoll, 0.003);
p = plot(time, vAngRoll, 'Linewidth', 2);
p.Color = '#0072BD';
hold on;
p = plot(time, vAngPitch, 'Linewidth', 2);
p.Color = '#A2142F';
title("Angular Velocity vs. Time");
% Note: Pitch and Roll were flipped.
legend("", "", "Pitch", "Roll");
xlabel("Time (ms)");
ylabel("Angular Velocity (degrees/s)");