close all;
clear;
clc;

filename = 'dataset9.txt';

%Import the options of the csv file
opts = detectImportOptions(filename);
%Defines the row location of channel variable name
opts.VariableNamesLine = 1;
%Specifies that the data is comma seperated
opts.Delimiter = ','; %Specifies that the data is comma seperated

T = readtable(filename, opts, 'ReadVariableNames', true);

%% Parse input table
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

pt = [0 0 0];
dir = [0 0 1 1];
h1 = quiver3(pt(1),pt(2),pt(3), dir(1),dir(2),dir(3),'LineWidth',2);
hold on;
h2 = quiver3(pt(1),pt(2),pt(3), dir(1),dir(2),dir(3),'Linewidth',2);
xlim([-1 1])
ylim([-1 1])
zlim([-1 1])

% Init variable to store last measurement time.
oldTime = 0;
sroll = 0;
spitch = 0;
froll = 0;
fpitch = 0;

% Init joint angle variable.
ang = zeros(size(time, 1), 1);

for i = 1:size(time, 1)
    % Get accelerometer roll and pitch.
    saccRoll = atan2(saccy(i),sqrt(saccx(i) .^ 2 + saccz(i) .^ 2)) * (pi / 180);
    saccPitch = atan2(saccx(i),sqrt(saccy(i) .^ 2 + saccz(i) .^ 2)) * (pi / 180);
    faccRoll = atan2(faccy(i),sqrt(faccx(i) .^ 2 + faccz(i) .^ 2)) * (pi / 180);
    faccPitch = -atan2(faccx(i),sqrt(faccy(i) .^ 2 + faccz(i) .^ 2)) * (pi / 180);
    % Get gyroscope roll and pitch.
    sroll = sroll + sgyrx(i) * (pi / 180) * ((time(i) - oldTime) / 1000);
    spitch = spitch + sgyry(i) * (pi / 180) * ((time(i) - oldTime) / 1000);
    froll = froll + fgyrx(i) * (pi / 180) * ((time(i) - oldTime) / 1000);
    fpitch = fpitch + fgyry(i) * (pi / 180) * ((time(i) - oldTime) / 1000);

    % Comp filter algorithm.
    sroll = 0.98 * sroll + 0.02 * (saccRoll * 180 / pi);
    spitch = 0.98 * spitch + 0.02 * (saccPitch * 180 / pi);
    froll = 0.98 * froll + 0.02 * (faccRoll * 180 / pi);
    fpitch = 0.98 * fpitch + 0.02 * (faccPitch * 180 / pi);

    % Draw the vectors.
    xfm1 = makehgtform('xrotate', sroll, 'yrotate', spitch);
    xfm2 = makehgtform('xrotate', froll, 'yrotate', fpitch);
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
    oldTime = time(i);
    
    % Get joint angle.
    fvec = [newdir1(1) newdir1(2) newdir1(3)];
    svec = [newdir2(1) newdir2(2) newdir2(3)];

    ang(i) = acos(dot(fvec, svec) ./ (norm(fvec) .* norm(svec))) .* 180 ./ pi;
end

figure(2);
plot(time, ang);