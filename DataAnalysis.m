% Blood Pressure Detection CSV Comparison Tool
% This script compares multiple CSV files with different detection algorithms
% for systolic blood pressure measurement

clear; clc; close all;

%% Configuration
% Select CSV files to analyze
[files, path] = uigetfile('*.csv', 'Select CSV files to compare', 'MultiSelect', 'on');

if isequal(files, 0)
    disp('No files selected. Exiting.');
    return;
end

% Ensure files is a cell array
if ~iscell(files)
    files = {files};
end

%% Load and Process Data
numFiles = length(files);
fileData = cell(numFiles, 1);
allDetectors = {};

fprintf('Loading %d files...\n', numFiles);

for i = 1:numFiles
    filename = fullfile(path, files{i});
    fprintf('  Loading: %s\n', files{i});
    
    % Read the CSV file with preserved column names
    data = readtable(filename, 'VariableNamingRule', 'preserve');
    
    % Store data
    fileData{i}.filename = files{i};
    fileData{i}.data = data;
    
    % Filter out data where pressure is less than 10 (changed from 5)
    validIdx = data.Pressure >= 10;
    fileData{i}.time = data.Time(validIdx) / 1000; % Convert to seconds
    fileData{i}.pressure = data.Pressure(validIdx);
    fileData{i}.ppg = data.PPGSignal(validIdx);
    
    % Get detector columns (all columns except Time, Pressure, PPGSignal)
    allCols = data.Properties.VariableNames;
    detectorCols = allCols(~ismember(allCols, {'Time', 'Pressure', 'PPGSignal'}));
    fileData{i}.detectors = detectorCols;
    
    % Find detection points (first non-zero value for each detector)
    % Use containers.Map instead of struct to allow dots in names
    fileData{i}.detections = containers.Map();
    for j = 1:length(detectorCols)
        detectorName = detectorCols{j};
        detectorValues = data.(detectorName)(validIdx);
        
        % Find first non-zero index
        nonZeroIdx = find(detectorValues ~= 0, 1);
        detection = struct();
        if ~isempty(nonZeroIdx)
            detection.time = fileData{i}.time(nonZeroIdx);
            detection.pressure = fileData{i}.pressure(nonZeroIdx);
            detection.value = detectorValues(nonZeroIdx);
            detection.detected = true;
        else
            detection.time = NaN;
            detection.pressure = NaN;
            detection.value = NaN;
            detection.detected = false;
        end
        fileData{i}.detections(detectorName) = detection;
    end
    
    % Collect all unique detectors
    allDetectors = union(allDetectors, detectorCols);
end

fprintf('Found %d unique detectors across all files.\n\n', length(allDetectors));

%% Create Main Tabbed Figure
% Get screen size and position window properly
screenSize = get(0, 'ScreenSize'); % [left, bottom, width, height]
figWidth = 1600;
figHeight = 900;

% Center the figure on screen
figLeft = max(1, (screenSize(3) - figWidth) / 2);
figBottom = max(1, (screenSize(4) - figHeight) / 2);

mainFig = figure('Name', 'Blood Pressure Analysis', ...
                 'Position', [figLeft, figBottom, figWidth, figHeight], ...
                 'NumberTitle', 'off');

% Create tab group
tabGroup = uitabgroup(mainFig);

% Create tabs for each file
for i = 1:numFiles
    % Use filename without extension for tab title
    [~, tabName, ~] = fileparts(files{i});
    tab = uitab(tabGroup, 'Title', tabName);
    createFileTab(tab, fileData{i}, i);
end

% Create comparison tabs
compTab = uitab(tabGroup, 'Title', 'Detector Comparison');
createComparisonTab(compTab, fileData, allDetectors);

blTab = uitab(tabGroup, 'Title', 'Baseline Analysis');
createBLAnalysisTab(blTab, fileData, allDetectors);

drvTab = uitab(tabGroup, 'Title', 'Derivative Analysis');
createDRVAnalysisTab(drvTab, fileData, allDetectors);

fprintf('\nAnalysis complete! Use the tabs to navigate between views.\n');

%% Helper Functions

function createFileTab(parentTab, fileInfo, fileNum)
    % Create single axes for overlayed plot with dual y-axes
    ax = axes('Parent', parentTab, 'Position', [0.08, 0.15, 0.78, 0.75]);
    
    % Plot PPG on left y-axis
    yyaxis left
    hold on; grid on;
    h2 = plot(fileInfo.time, fileInfo.ppg, 'r-', 'LineWidth', 1.0);
    ylabel('PPG Signal', 'FontSize', 12);
    ax.YColor = 'r';
    
    % Plot Pressure on right y-axis
    yyaxis right
    h1 = plot(fileInfo.time, fileInfo.pressure, 'b-', 'LineWidth', 1.5);
    ylabel('Pressure (mmHg)', 'FontSize', 12);
    ax.YColor = 'b';
    
    % Plot detection lines and markers
    colors = lines(length(fileInfo.detectors));
    legendHandles = [h1, h2];
    legendLabels = {'Pressure', 'PPG'};
    
    for j = 1:length(fileInfo.detectors)
        detName = fileInfo.detectors{j};
        if isKey(fileInfo.detections, detName)
            detection = fileInfo.detections(detName);
            detTime = detection.time;
            detPressure = detection.pressure;
            
            if detection.detected && ~isnan(detTime)
                % Detected - show line and marker with pressure value
                xline(detTime, '--', 'Color', colors(j,:), 'LineWidth', 1.5);
                h = plot(detTime, detPressure, 'o', 'Color', colors(j,:), ...
                     'MarkerSize', 10, 'LineWidth', 2);
                legendHandles(end+1) = h;
                legendLabels{end+1} = sprintf('%s: %.1f mmHg', detName, detPressure);
            else
                % Not detected - create invisible dummy plot for legend
                h = plot(NaN, NaN, 'o', 'Color', colors(j,:), 'MarkerSize', 10, 'LineWidth', 2);
                legendHandles(end+1) = h;
                legendLabels{end+1} = sprintf('%s: No detection', detName);
            end
        end
    end
    
    xlabel('Time (s)', 'FontSize', 12);
    title(sprintf('%s', fileInfo.filename), 'FontSize', 14, 'Interpreter', 'none');
    
    % Create legend with collected handles and labels (limit to MaxNumEntries)
    lgd = legend(legendHandles, legendLabels, 'Location', 'eastoutside', 'FontSize', 9);
    lgd.NumColumns = 1;
end

function createComparisonTab(parentTab, fileData, allDetectors)
    numFiles = length(fileData);
    numDetectors = length(allDetectors);
    
    % Create file name labels
    fileLabels = cell(numFiles, 1);
    for i = 1:numFiles
        [~, fileLabels{i}, ~] = fileparts(fileData{i}.filename);
    end
    
    % Create matrix to store pressures
    detectionPressures = nan(numDetectors, numFiles);
    
    % Fill matrix
    for i = 1:numFiles
        for j = 1:numDetectors
            detName = allDetectors{j};
            if isKey(fileData{i}.detections, detName)
                detection = fileData{i}.detections(detName);
                detectionPressures(j, i) = detection.pressure;
            end
        end
    end
    
    % Create axes
    ax2 = axes('Parent', parentTab, 'Position', [0.08, 0.165, 0.38, 0.3]);
    ax3 = axes('Parent', parentTab, 'Position', [0.08, 0.66, 0.86, 0.3]);
    ax4 = axes('Parent', parentTab, 'Position', [0.56, 0.165, 0.38, 0.3]);
    
    % Plot 2: Detection Pressures with custom colormap
    axes(ax2);
    imagesc(detectionPressures);
    
    % Create custom colormap: blue (low) -> green (80-140) -> red (high)
    % Define color ranges
    cmap2 = createBPColormap();
    colormap(ax2, cmap2);
    
    % Set color limits to fixed range
    clim([60, 160]);
    colorbar;
    
    title('Detected Systolic Pressure (mmHg)', 'FontSize', 14);
    xlabel('File', 'FontSize', 12);
    ylabel('Detector', 'FontSize', 12);
    set(gca, 'YTick', 1:numDetectors, 'YTickLabel', allDetectors, 'FontSize', 8);
    shortLabels = cellfun(@(s) shortenLabel(s), fileLabels, 'UniformOutput', false);
    set(gca, 'XTick', 1:numFiles, 'XTickLabel', shortLabels, 'XTickLabelRotation', 45, 'FontSize', 8);
    
    % Add text labels with adaptive color
    for i = 1:numDetectors
        for j = 1:numFiles
            if ~isnan(detectionPressures(i, j))
                % Determine text color based on pressure value
                pressure = detectionPressures(i, j);
                textColor = getTextColorForPressure(pressure);
                
                text(j, i, sprintf('%.1f', pressure), ...
                     'HorizontalAlignment', 'center', 'FontSize', 8, ...
                     'Color', textColor, 'FontWeight', 'bold');
            end
        end
    end
    
    % Plot 3: Mean and Std Dev of Pressures
    axes(ax3);
    meanPressures = mean(detectionPressures, 2, 'omitnan');
    stdPressures = std(detectionPressures, 0, 2, 'omitnan');
    errorbar(1:numDetectors, meanPressures, stdPressures, 'o-', 'LineWidth', 1.5);
    grid on;
    title('Mean Detected Pressure ± Std Dev', 'FontSize', 14);
    xlabel('Detector', 'FontSize', 12);
    ylabel('Pressure (mmHg)', 'FontSize', 12);
    set(gca, 'XTick', 1:numDetectors, 'XTickLabel', allDetectors, 'XTickLabelRotation', 45);
    
    % Plot 4: Detection Success Rate
    axes(ax4);
    successRate = sum(~isnan(detectionPressures), 2) / numFiles * 100;
    bar(successRate);
    grid on;
    title('Detection Success Rate', 'FontSize', 14);
    xlabel('Detector', 'FontSize', 12);
    ylabel('Success Rate (%)', 'FontSize', 12);
    set(gca, 'XTick', 1:numDetectors, 'XTickLabel', allDetectors, 'XTickLabelRotation', 45);
    ylim([0 110]);
end

function out = shortenLabel(s)
    if strlength(s) > 8
        out = ['...' extractAfter(s, strlength(s)-8)];
    else
        out = s;
    end
end

function cmap = createBPColormap()
    % Create custom colormap for blood pressure
    % Blue (< 80) -> Green (80-140) -> Red (> 140)
    
    % Define color points
    % Low range (60-80): Blue shades
    lowColors = [0.0 0.0 0.8;    % Dark blue at 60
                 0.3 0.5 1.0];   % Light blue at 80
    
    % Good range (80-140): Green/Yellow shades
    goodColors = [0.4 0.8 0.4;   % Green at 80
                  0.6 0.9 0.3;   % Yellow-green
                  0.9 0.9 0.2];  % Yellow at 140
    
    % High range (140-160): Red shades
    highColors = [1.0 0.5 0.0;   % Orange at 140
                  1.0 0.0 0.0];  % Red at 160
    
    % Interpolate colors
    nLow = 20;   % Number of colors for 60-80 range
    nGood = 60;  % Number of colors for 80-140 range
    nHigh = 20;  % Number of colors for 140-160 range
    
    cmapLow = interp1(linspace(0, 1, size(lowColors, 1)), lowColors, linspace(0, 1, nLow));
    cmapGood = interp1(linspace(0, 1, size(goodColors, 1)), goodColors, linspace(0, 1, nGood));
    cmapHigh = interp1(linspace(0, 1, size(highColors, 1)), highColors, linspace(0, 1, nHigh));
    
    cmap = [cmapLow; cmapGood; cmapHigh];
end

function textColor = getTextColorForPressure(pressure)
    % Return 'black' or 'white' based on pressure value
    % Use black text for light colors (good range and certain extremes)
    
    if pressure >= 80 && pressure <= 140
        % Good range - use black text (green/yellow background)
        textColor = 'black';
    elseif pressure < 75
        % Very low - dark blue, use white
        textColor = 'white';
    elseif pressure > 145
        % Very high - dark red, use white
        textColor = 'white';
    else
        % Transition zones - use black for lighter shades
        textColor = 'black';
    end
end

function createBLAnalysisTab(parentTab, fileData, allDetectors)
    % Find all BL detectors
    blDetectors = allDetectors(startsWith(allDetectors, 'BL_'));
    
    if isempty(blDetectors)
        % Create text message
        annotation(parentTab, 'textbox', [0.3, 0.4, 0.4, 0.2], ...
                  'String', 'No BL detectors found in the data.', ...
                  'FontSize', 14, 'HorizontalAlignment', 'center', ...
                  'EdgeColor', 'none');
        return;
    end
    
    % Parse BL parameters
    numBL = length(blDetectors);
    params = zeros(numBL, 4); % W, T, D, C
    
    for i = 1:numBL
        tokens = regexp(blDetectors{i}, 'BL_W(\d+)_T([\d.]+)_D(\d+)_C(\d+)', 'tokens');
        if ~isempty(tokens)
            params(i, :) = [str2double(tokens{1}{1}), str2double(tokens{1}{2}), ...
                           str2double(tokens{1}{3}), str2double(tokens{1}{4})];
        end
    end
    
    % Collect detection pressures
    numFiles = length(fileData);
    detPressures = nan(numBL, numFiles);
    
    for i = 1:numFiles
        for j = 1:numBL
            detName = blDetectors{j};
            if isKey(fileData{i}.detections, detName)
                detection = fileData{i}.detections(detName);
                detPressures(j, i) = detection.pressure;
            end
        end
    end
    
    % Create axes
    ax1 = axes('Parent', parentTab, 'Position', [0.08, 0.55, 0.38, 0.38]);
    ax2 = axes('Parent', parentTab, 'Position', [0.56, 0.55, 0.38, 0.38]);
    ax3 = axes('Parent', parentTab, 'Position', [0.08, 0.08, 0.38, 0.38]);
    ax4 = axes('Parent', parentTab, 'Position', [0.56, 0.08, 0.38, 0.38]);
    
    % Effect of Window Size (W)
    axes(ax1);
    uniqueW = unique(params(:, 1));
    meanByW = zeros(length(uniqueW), 1);
    stdByW = zeros(length(uniqueW), 1);
    for i = 1:length(uniqueW)
        idx = params(:, 1) == uniqueW(i);
        meanByW(i) = mean(detPressures(idx, :), 'all', 'omitnan');
        stdByW(i) = std(detPressures(idx, :), 0, 'all', 'omitnan');
    end
    errorbar(uniqueW, meanByW, stdByW, 'o-', 'LineWidth', 2, 'MarkerSize', 8);
    grid on;
    xlabel('Window Size (W)', 'FontSize', 12);
    ylabel('Mean Detected Pressure (mmHg)', 'FontSize', 12);
    title('Effect of Window Size', 'FontSize', 14);
    
    % Effect of Threshold (T)
    axes(ax2);
    uniqueT = unique(params(:, 2));
    meanByT = zeros(length(uniqueT), 1);
    stdByT = zeros(length(uniqueT), 1);
    for i = 1:length(uniqueT)
        idx = params(:, 2) == uniqueT(i);
        meanByT(i) = mean(detPressures(idx, :), 'all', 'omitnan');
        stdByT(i) = std(detPressures(idx, :), 0, 'all', 'omitnan');
    end
    errorbar(uniqueT, meanByT, stdByT, 'o-', 'LineWidth', 2, 'MarkerSize', 8);
    grid on;
    xlabel('Threshold Multiplier (T)', 'FontSize', 12);
    ylabel('Mean Detected Pressure (mmHg)', 'FontSize', 12);
    title('Effect of Threshold', 'FontSize', 14);
    
    % Effect of Minimum Deviation (D)
    axes(ax3);
    uniqueD = unique(params(:, 3));
    meanByD = zeros(length(uniqueD), 1);
    stdByD = zeros(length(uniqueD), 1);
    for i = 1:length(uniqueD)
        idx = params(:, 3) == uniqueD(i);
        meanByD(i) = mean(detPressures(idx, :), 'all', 'omitnan');
        stdByD(i) = std(detPressures(idx, :), 0, 'all', 'omitnan');
    end
    errorbar(uniqueD, meanByD, stdByD, 'o-', 'LineWidth', 2, 'MarkerSize', 8);
    grid on;
    xlabel('Minimum Deviation (D)', 'FontSize', 12);
    ylabel('Mean Detected Pressure (mmHg)', 'FontSize', 12);
    title('Effect of Minimum Deviation', 'FontSize', 14);
    
    % Effect of Consecutive Readings (C)
    axes(ax4);
    uniqueC = unique(params(:, 4));
    meanByC = zeros(length(uniqueC), 1);
    stdByC = zeros(length(uniqueC), 1);
    for i = 1:length(uniqueC)
        idx = params(:, 4) == uniqueC(i);
        meanByC(i) = mean(detPressures(idx, :), 'all', 'omitnan');
        stdByC(i) = std(detPressures(idx, :), 0, 'all', 'omitnan');
    end
    errorbar(uniqueC, meanByC, stdByC, 'o-', 'LineWidth', 2, 'MarkerSize', 8);
    grid on;
    xlabel('Consecutive Readings (C)', 'FontSize', 12);
    ylabel('Mean Detected Pressure (mmHg)', 'FontSize', 12);
    title('Effect of Consecutive Readings', 'FontSize', 14);
end

function createDRVAnalysisTab(parentTab, fileData, allDetectors)
    % Find all DRV detectors
    drvDetectors = allDetectors(startsWith(allDetectors, 'DRV_'));
    
    if isempty(drvDetectors)
        % Create text message
        annotation(parentTab, 'textbox', [0.3, 0.4, 0.4, 0.2], ...
                  'String', 'No DRV detectors found in the data.', ...
                  'FontSize', 14, 'HorizontalAlignment', 'center', ...
                  'EdgeColor', 'none');
        return;
    end
    
    % Parse DRV parameters
    numDRV = length(drvDetectors);
    params = zeros(numDRV, 2); % W, T
    
    for i = 1:numDRV
        tokens = regexp(drvDetectors{i}, 'DRV_W(\d+)_T([\d.]+)', 'tokens');
        if ~isempty(tokens)
            params(i, :) = [str2double(tokens{1}{1}), str2double(tokens{1}{2})];
        end
    end
    
    % Collect detection pressures
    numFiles = length(fileData);
    detPressures = nan(numDRV, numFiles);
    
    for i = 1:numFiles
        for j = 1:numDRV
            detName = drvDetectors{j};
            if isKey(fileData{i}.detections, detName)
                detection = fileData{i}.detections(detName);
                detPressures(j, i) = detection.pressure;
            end
        end
    end
    
    % Create axes
    ax1 = axes('Parent', parentTab, 'Position', [0.1, 0.3, 0.35, 0.5]);
    ax2 = axes('Parent', parentTab, 'Position', [0.55, 0.3, 0.35, 0.5]);
    
    % Effect of Window Size (W)
    axes(ax1);
    uniqueW = unique(params(:, 1));
    meanByW = zeros(length(uniqueW), 1);
    stdByW = zeros(length(uniqueW), 1);
    for i = 1:length(uniqueW)
        idx = params(:, 1) == uniqueW(i);
        meanByW(i) = mean(detPressures(idx, :), 'all', 'omitnan');
        stdByW(i) = std(detPressures(idx, :), 0, 'all', 'omitnan');
    end
    errorbar(uniqueW, meanByW, stdByW, 'o-', 'LineWidth', 2, 'MarkerSize', 8);
    grid on;
    xlabel('Window Size (W)', 'FontSize', 12);
    ylabel('Mean Detected Pressure (mmHg)', 'FontSize', 12);
    title('Effect of Window Size', 'FontSize', 14);
    
    % Effect of Threshold (T)
    axes(ax2);
    uniqueT = unique(params(:, 2));
    meanByT = zeros(length(uniqueT), 1);
    stdByT = zeros(length(uniqueT), 1);
    for i = 1:length(uniqueT)
        idx = params(:, 2) == uniqueT(i);
        meanByT(i) = mean(detPressures(idx, :), 'all', 'omitnan');
        stdByT(i) = std(detPressures(idx, :), 0, 'all', 'omitnan');
    end
    errorbar(uniqueT, meanByT, stdByT, 'o-', 'LineWidth', 2, 'MarkerSize', 8);
    grid on;
    xlabel('Threshold (T)', 'FontSize', 12);
    ylabel('Mean Detected Pressure (mmHg)', 'FontSize', 12);
    title('Effect of Threshold', 'FontSize', 14);
end