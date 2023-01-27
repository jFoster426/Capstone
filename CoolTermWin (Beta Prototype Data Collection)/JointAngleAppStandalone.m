% Calibration file
% fullfilepath = 'C:\Users\juddf\Documents\Capstone\CoolTermWin (Beta Prototype Data Collection)\wrist\CalibrateWrist.txt';
fullfilepath = 'C:\Users\juddf\Documents\Capstone\CoolTermWin (Beta Prototype Data Collection)\CalibrateAnkle.txt';

C = csvread(fullfilepath, 1);

fgyrxcal = C(:, 5);
fgyrxcal = fgyrxcal .* (2000.0 / 32768.0);
fgyrycal = C(:, 6);
fgyrycal = fgyrycal .* (2000.0 / 32768.0);
fgyrzcal = C(:, 7);
fgyrzcal = fgyrzcal .* (2000.0 / 32768.0);

sgyrxcal = C(:, 11);
sgyrxcal = sgyrxcal .* (2000.0 / 32768.0);
sgyrycal = C(:, 12);
sgyrycal = sgyrycal .* (2000.0 / 32768.0);
sgyrzcal = C(:, 13);
sgyrzcal = sgyrzcal .* (2000.0 / 32768.0);

fgyrxo = mean(fgyrxcal);
fgyryo = mean(fgyrycal);
fgyrzo = mean(fgyrzcal);

sgyrxo = mean(sgyrxcal);
sgyryo = mean(sgyrycal);
sgyrzo = mean(sgyrzcal);

app.CalibratedLamp.Color = [0 1 0];

% sgyrxo = 0;
% sgyryo = 0;
% sgyrzo = 0;
% 
% fgyrxo = 0;
% fgyryo = 0;
% fgyrzo = 0;


% Data file
fullfilepath = 'C:\Users\juddf\Documents\Capstone\CoolTermWin (Beta Prototype Data Collection)\left_ankle.txt';
% fullfilepath = 'C:\Users\juddf\Documents\Capstone\CoolTermWin (Beta Prototype Data Collection)\wrist\CoolTerm Capture (wrist.stc) 2022-11-08 13-25-43-181.txt';

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

load = (-M(:, 14) + -M(:, 15)) ./ 2;

fgyrx = fgyrx - fgyrxo;
fgyry = fgyry - fgyryo;
fgyrz = fgyrz - fgyrzo;
sgyrx = sgyrx - sgyrxo;
sgyry = sgyry - sgyryo;
sgyrz = sgyrz - sgyrzo;

% Normalize the time to start at 0 ms.
for i = 2:size(time)
    time(i) = time(i) - time(1);
    load(i) = load(i) - load(1);
end
time(1) = 0;
load(1) = 0;

load = lowpass(load, 0.01);

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

% Calculate the joint angle in degrees.
jointAngleOffset = 0;
roll = sRoll - fRoll + jointAngleOffset;
pitch = sPitch - fPitch + jointAngleOffset;

% Integrate to find the velocity in m/s.
vAngRoll = zeros(size(time));
vAngPitch = zeros(size(time));
for i = 2:size(time)
    vAngPitch(i) = (pitch(i) - pitch(i - 1)) / ((time(i) - time(i - 1)) / 1000);
    vAngRoll(i) = (roll(i) - roll(i - 1)) / ((time(i) - time(i - 1)) / 1000);
end

vAngPitchFiltered = lowpass(vAngPitch, 0.002);
vAngRollFiltered = lowpass(vAngRoll, 0.002);

app.DataSetLamp.Color = [0 1 0];

% pitch = pitch - 100;

figure(1);
% plot(time, fPitch, 'Color', [0 0.4470 0.7410], 'LineWidth', 2);
% plot(time, sPitch, 'Color', [0 0.4470 0.7410], 'LineWidth', 2);
% plot(time, fRoll, 'Color', [0 0.4470 0.7410], 'LineWidth', 2);
% plot(time, sRoll, 'Color', [0 0.4470 0.7410], 'LineWidth', 2);
% plot(time, pitch, 'Color', [0 0.4470 0.7410], 'LineWidth', 2);
% plot(time, roll, 'Color', [0 0.4470 0.7410], 'LineWidth', 2);
% plot(time, vAngRoll, 'Color', [0 0.4470 0.7410], 'LineWidth', 2);
% plot(time, vAngPitch, 'Color', [0 0.4470 0.7410], 'LineWidth', 2);
% plot(time, vAngRollFiltered, 'Color', [0 0.4470 0.7410], 'LineWidth', 2);
% plot(time, vAngPitchFiltered, 'Color', [0 0.4470 0.7410], 'LineWidth', 2);

% ankle = 21.88g  / tick, kg = measurement / 45.704
% wrist = 21.88mg / tick, kg = measurement / 45704

scaleFactor = 45.704;

load = load ./ scaleFactor;



plot(time, load, 'Color', [0 0.4470 0.7410], 'LineWidth', 2);

xlabel('Time (ms)');
ylabel('Force Applied by Clinician (kg)');
% ylabel('Joint Angle (Degrees)');

grid on;

ylim([-5 20]);

set(gca,'FontSize',20);

%  title('Joint Angle vs. Time');
title('Muscle Resistance vs. Time');

ax=gca; ax.XAxis.Exponent = 0;