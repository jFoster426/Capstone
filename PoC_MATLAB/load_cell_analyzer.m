%% Close everything and restart.
close all;
clear;
clc;

%% Open the input csv file.
filename = "../PoC_Data/loadcell_50lb.txt";
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

plot(load);

lwr = 584;
uppr = 1450;

av = 0;
for i = lwr:uppr
    av = av + load(i);
end
av = av / (uppr - lwr + 1);
disp(av);
hold on;


%% Gathered data
% 3     8.32E+06
% 5     8.36E+06
% 8	    8.42E+06
% 10    8.49E+06
% 12    8.55E+06
% 15    8.59E+06
% 20    8.68E+06
% 25    8.84E+06
% 30    8.97E+06
% 35    9.12E+06
% 40    9.25E+06
% 45    9.33E+06
% 50    9.43E+06
