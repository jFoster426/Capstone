close all;
clear;
clc;

filename = 'dataset6_90degshin.txt';

%Import the options of the csv file
opts = detectImportOptions(filename);
%Defines the row location of channel variable name
opts.VariableNamesLine = 1;
%Specifies that the data is comma seperated
opts.Delimiter = ',';

T = readtable(filename, opts, 'ReadVariableNames', true);

t = table2array(T(:,"time"));

%% Parse input table

faccx = table2array(T(:,"faccx"));
faccy = table2array(T(:,"faccy"));
faccz = table2array(T(:,"faccz"));
fgyrx = table2array(T(:,"fgyrx"));
fgyry = table2array(T(:,"fgyry"));
fgyrz = table2array(T(:,"fgyrz"));
fmagx = table2array(T(:,"fmagx"));
fmagy = table2array(T(:,"fmagy"));
fmagz = table2array(T(:,"fmagz"));

saccx = table2array(T(:,"saccx"));
saccy = table2array(T(:,"saccy"));
saccz = table2array(T(:,"saccz"));
sgyrx = table2array(T(:,"sgyrx"));
sgyry = table2array(T(:,"sgyry"));
sgyrz = table2array(T(:,"sgyrz"));
smagx = table2array(T(:,"smagx"));
smagy = table2array(T(:,"smagy"));
smagz = table2array(T(:,"smagz"));

%% Integrate Gyroscope to get angular displacement

fang = zeros(3, size(t, 1));
for i = 2:size(t,1)
    fang(:, i) = fang(:, i - 1) + [fgyrx(i); fgyry(i); fgyrz(i)];
    fang(:, i) = fang(:, i) .* (t(i) - t(i - 1)) ./ 1000;
end

fangint = zeros(3, size(t, 1));
for i = 2:size(t,1)
    fangint(:, i) = fangint(:, i - 1) + fang(:, i - 1);
end

sang = zeros(3, size(t, 1));
for i = 2:size(t,1)
    sang(:, i) = sang(:, i - 1) + [sgyrx(i); sgyry(i); sgyrz(i)];
    sang(:, i) = sang(:, i) .* (t(i) - t(i - 1)) ./ 1000;
end

sangint = zeros(3, size(t, 1));
for i = 2:size(t,1)
    sangint(:, i) = sangint(:, i - 1) + sang(:, i - 1);
end

figure(1);
plot(t, fangint(1, :), 'r');
hold on;
plot(t, fangint(2, :), 'k');
plot(t, fangint(3, :), 'b');

figure(2);
plot(t, sangint(1, :), 'r');
hold on;
plot(t, sangint(2, :), 'k');
plot(t, sangint(3, :), 'b');

figure(3);
plot(t, sangint(1, :) - fangint(2, :), 'r');
hold on;
plot(t, sangint(2, :) + fangint(1, :), 'k');
plot(t, sangint(3, :) - fangint(3, :), 'b');

sanginthp = zeros(size(sangint));
fanginthp = zeros(size(fangint));

for i = 1:size(fanginthp, 1)
    sanginthp(i, :) = highpass(sangint(i, :), 0.007);
    fanginthp(i, :) = highpass(fangint(i, :), 0.007);
end

figure(4);
plot(t, sanginthp(1, :) - fanginthp(2, :), 'r');
hold on;
plot(t, sanginthp(2, :) + fanginthp(1, :), 'k');
plot(t, sanginthp(3, :) - fanginthp(3, :), 'b');

pause;

%% Accelerometer dot product

filt_freq = 0.99;

faccxlp = lowpass(faccx, filt_freq);
faccylp = lowpass(faccy, filt_freq);
facczlp = lowpass(faccz, filt_freq);

saccxlp = lowpass(saccx, filt_freq);
saccylp = lowpass(saccy, filt_freq);
sacczlp = lowpass(saccz, filt_freq);

pt = [0 0 0];
figure(5);
for i = 1:size(t, 1)
    faccm(i) = sqrt(faccxlp(i).^2 + faccylp(i).^2 + facczlp(i).^2);
    saccm(i) = sqrt(saccxlp(i).^2 + saccylp(i).^2 + sacczlp(i).^2);
    acc_ang(i) = (180 ./ pi) .* acos(dot([faccxlp(i), faccylp(i), facczlp(i)], [saccxlp(i), saccylp(i), sacczlp(i)]) ./ (faccm(i) * saccm(i)));
    fdir = [faccxlp(i), faccylp(i), facczlp(i)];
    sdir = [saccxlp(i), saccylp(i), sacczlp(i)];
    fh = quiver3(pt(1),pt(2),pt(3), fdir(1),fdir(2),fdir(3));
    hold on;
    sh = quiver3(pt(1),pt(2),pt(3), sdir(1),sdir(2),sdir(3));
    xlim([-2 2]);
    ylim([-2 2]);
    zlim([-2 2]);
    drawnow;
    hold off;
end

figure(6);
plot(t, acc_ang);

figure(7);
plot(t, faccm, 'r');
hold on;
plot(t, saccm, 'b');
