% Multi-CSV Plotter for PPG and Pressure
% Creates one figure per CSV with dual y-axes.
clear; clc; close all;

% Select multiple CSV files
[files, path] = uigetfile('*.csv', 'Select CSV Files', 'MultiSelect', 'on');

if isequal(files,0)
    disp('No files selected.');
    return;
end

% Ensure files is a cell array when only one file is chosen
if ischar(files)
    files = {files};
end

for i = 1:length(files)
    filename = fullfile(path, files{i});
    fprintf("Loading %s ...\n", files{i});

    % Read CSV
    data = readtable(filename);

    % Validate columns
    requiredCols = ["Time","Pressure","rawPPGSignal"];
    if ~all(ismember(requiredCols, data.Properties.VariableNames))
        warning("File %s is missing one of the required columns.", files{i});
        continue;
    end

    % Extract
    t = data.Time;
    ppg = data.rawPPGSignal;
    pressure = data.Pressure;

    % --- Plot ---
    figure('Name', files{i}, 'NumberTitle', 'off');

    yyaxis left
    plot(t, ppg, 'LineWidth', 1);
    ylabel('PPG Signal')

    yyaxis right
    plot(t, pressure, 'LineWidth', 1);
    ylabel('Pressure')

    xlabel('Time')
    title(sprintf('PPG & Pressure: %s', files{i}), 'Interpreter', 'none');
    grid on;

    % Allow zooming/panning naturally
    zoom on;
end
