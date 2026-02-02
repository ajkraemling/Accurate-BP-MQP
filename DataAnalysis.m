% Blood Pressure Detection CSV Comparison Tool
% This script compares multiple CSV files with different detection algorithms
% for systolic blood pressure measurement
% Now supports multiple runs within a single CSV file
% Uses first AUS_PULSE_HEARD as ground truth for error analysis
clear; clc; close all;

fprintf('=== STARTING BLOOD PRESSURE ANALYSIS ===\n');

%% Configuration
% Select CSV files to analyze

selectedFiles = {};

while true
    [files, path] = uigetfile('*.csv', ...
                              'Select CSV files (Cancel to finish)', ...
                              'MultiSelect', 'on');
    if isequal(files, 0)
        break;  % Done selecting directories
    end

    % Ensure files is a cell array
    if ~iscell(files)
        files = {files};
    end

    % Append all selected files with full path
    for i = 1:length(files)
        selectedFiles{end+1} = fullfile(path, files{i});
    end
end

if isempty(selectedFiles)
    disp('No files selected. Exiting.');
    return;
end

files = selectedFiles;

%% Load and Process Data
allRuns = {};
runCounter = 1;

fprintf('Loading %d files...\n', length(files));

for fileIdx = 1:length(files)
    filename = files{fileIdx};   % already full path

    % Extract name for display
    [~, shortName, ext] = fileparts(filename);
    fprintf(' Loading: %s%s\n', shortName, ext);
    
    
    % Read the CSV file with preserved column names
    opts = detectImportOptions(filename);
    opts.VariableNamesLine = 1;        % Force MATLAB to treat first row as headers
    data = readtable(filename, opts);
    
    fprintf('  Initial data has %d rows\n', height(data));

    % Split data into runs
    run = {data(data.Pressure >= 30, :)};
 
    fprintf('  Found %d run(s) in this file\n', length(run));
    
    % Process each run
    for runIdx = 1:length(run)
        runData = run{runIdx};
        
        % Create run info structure
        runInfo = struct();

        [fullPath, shortName, ext] = fileparts(filename);
        [~, folderName] = fileparts(fullPath);
        displayBase = fullfile(folderName, [shortName, ext]);

        if length(run) == 1
            runInfo.filename = filename;
            runInfo.displayName = displayBase;
        else
            runInfo.filename = filename;
            runInfo.displayName = sprintf('%s (Run %d)', displayBase, runIdx);
        end
        runInfo.fileIdx = fileIdx;
        runInfo.runIdx = runIdx;
        runInfo.data = runData;
        
        % Initialize optional fields
        runInfo.hasRawPPG = false;
        runInfo.hasAusPulse = false;
        runInfo.pulseHeardIndices = [];
        runInfo.hasGroundTruth = false;
        runInfo.groundTruthPressure = NaN;
        

        % Check for MAP oscillometric columns
fprintf('  Checking for MAP columns in data...\n');
fprintf('  Available columns: %s\n', strjoin(runData.Properties.VariableNames, ', '));

mapCols = {'Osc_Amp', 'Osc_SBP', 'MAP', 'Osc_DBP'};
hasCols = ismember(mapCols, runData.Properties.VariableNames);
fprintf('  Looking for: %s\n', strjoin(mapCols, ', '));
fprintf('  Found: %s\n', strjoin(mapCols(hasCols), ', '));

if all(ismember(mapCols, runData.Properties.VariableNames))
    runInfo.hasMAP = true;
    runInfo.oscAmp = runData.Osc_Amp * 50;
    runInfo.oscSBP = runData.Osc_SBP;
    runInfo.MAP = runData.MAP;
    runInfo.oscDBP = runData.Osc_DBP;
    fprintf('  *** FOUND MAP DATA ***\n');
    fprintf('  Osc_Amp range (raw): %.4f to %.4f\n', min(runData.Osc_Amp), max(runData.Osc_Amp));
    fprintf('  Osc_Amp range (x50): %.2f to %.2f\n', min(runInfo.oscAmp), max(runInfo.oscAmp));
    
    % Find MAP detection point
    mapDetIdx = find(runData.MAP ~= 0, 1);
    if ~isempty(mapDetIdx)
        runInfo.mapDetectionIdx = mapDetIdx;
        runInfo.mapDetectedPressure = runData.Pressure(mapDetIdx);
        runInfo.mapValue = runData.MAP(mapDetIdx);
        runInfo.mapSBPValue = runData.Osc_SBP(mapDetIdx);
        runInfo.mapDBPValue = runData.Osc_DBP(mapDetIdx);
        fprintf('  MAP: %.1f, SBP: %.1f, DBP: %.1f at pressure %.1f mmHg\n', ...
            runInfo.mapValue, runInfo.mapSBPValue, runInfo.mapDBPValue, runInfo.mapDetectedPressure);
    end
else
    runInfo.hasMAP = false;
    fprintf('  MAP data not found - missing columns: %s\n', strjoin(mapCols(~hasCols), ', '));
end
        
        % Check for rawPPGSignal column
        if ismember('rawPPGSignal', runData.Properties.VariableNames)
            rawPPG = runData.rawPPGSignal;
            if (ismember("PPGSignal", runData.Properties.VariableNames))
                runInfo.ppg = runData.PPGSignal;
            else
                runInfo.ppg = rawPPG;
            end
            rawPPG = rawPPG - mean(rawPPG, 'omitnan') + mean(runInfo.ppg, 'omitnan');
            runInfo.rawPPG = rawPPG;
            runInfo.hasRawPPG = true;
        elseif ismember('PPG', runData.Properties.VariableNames)
            rawPPG = runData.PPG;
            if (ismember("PPGSignal", runData.Properties.VariableNames))
                runInfo.ppg = runData.PPGSignal;
            else
                runInfo.ppg = rawPPG;
            end
            rawPPG = rawPPG - mean(rawPPG, 'omitnan') + mean(runInfo.ppg, 'omitnan');
            runInfo.rawPPG = rawPPG;
            runInfo.hasRawPPG = true;
        end

        % Data is already filtered (pressure >= 10) by splitIntoRuns
        if (ismember("Time", runData.Properties.VariableNames))
            runInfo.time = runData.Time / 1000; % Convert to seconds
        elseif ((ismember("Timestamp", runData.Properties.VariableNames)))
            runInfo.time = runData.Timestamp / 1000;
        end

        if (ismember("PPGSignal", runData.Properties.VariableNames))
            runInfo.ppg = runData.PPGSignal;
        else
            if isfield(runInfo, "rawPPG")
                runInfo.ppg = runInfo.rawPPG;
            else
                runInfo.ppg = 0;
            end
        end
        runInfo.pressure = runData.Pressure;
        
        % Check for AUS_PULSE_HEARD column
        ausColExists = ismember('AUS_PULSE_HEARD', runData.Properties.VariableNames);
        fprintf('  Checking for AUS_PULSE_HEARD: %d\n', ausColExists);
        
        if ausColExists
            runInfo.ausPulseHeard = runData.AUS_PULSE_HEARD;
            runInfo.pulseHeardIndices = find(runData.AUS_PULSE_HEARD == 1);
            runInfo.hasAusPulse = true;
            fprintf('  *** FOUND AUS_PULSE_HEARD with %d pulses ***\n', length(runInfo.pulseHeardIndices));
            
            % Extract ground truth (first pulse heard)
            if ~isempty(runInfo.pulseHeardIndices)
                firstPulseIdx = runInfo.pulseHeardIndices(1);
                runInfo.groundTruthPressure = runInfo.pressure(firstPulseIdx);
                runInfo.groundTruthTime = runInfo.time(firstPulseIdx);
                runInfo.hasGroundTruth = true;
                fprintf('  *** GROUND TRUTH: %.1f mmHg at %.2f s ***\n', ...
                    runInfo.groundTruthPressure, runInfo.groundTruthTime);
            end
        else
            fprintf('  AUS_PULSE_HEARD not found\n');
        end

        % Extract SBP from filename if present
        runInfo.hasSBPReference = false;
        runInfo.sbpReference = NaN;
        
        [~, fname, ~] = fileparts(filename);
        tokens = regexp(fname, '^SBP(\d+)_', 'tokens');
        if ~isempty(tokens)
            runInfo.hasSBPReference = true;
            runInfo.sbpReference = str2double(tokens{1}{1});
            fprintf('  *** FOUND SBP REFERENCE: %d mmHg from filename ***\n', runInfo.sbpReference);
        end
        
        % Get detector columns (exclude system columns and AUS_PULSE_HEARD)
        % Get detector columns (exclude system columns and AUS_PULSE_HEARD)
        allCols = runData.Properties.VariableNames;
        excludeCols = {'Time', 'Timestamp', 'Pressure', 'PPGSignal', 'PPG', 'rawPPGSignal', 'BaselineBeat', ...
              'AUS_PULSE_HEARD', 'Osc_Amp', 'MAP', 'Osc_DBP', 'Est_DBP'};
        detectorCols = allCols(~ismember(allCols, excludeCols));
        runInfo.detectors = detectorCols;
        if ismember('BaselineBeat', runData.Properties.VariableNames)
            runInfo.baselineBeat = runData.BaselineBeat;
            runInfo.hasBaselineBeat = true;
        else
            runInfo.baselineBeat = [];
            runInfo.hasBaselineBeat = false;
        end
        runInfo.post = allCols(ismember(allCols, {'Osc_SBP', 'MAP', 'Osc_DBP', 'Est_DBP', 'EnsembleSystolic'}));
        
        % NEW: Store the actual values from the last row
        runInfo.postValues = struct();
        for k = 1:numel(runInfo.post)
            colName = runInfo.post{k};
            if ismember(colName, runData.Properties.VariableNames)
                val = runData{end, colName};  % Get last value
                
                % Unwrap if needed
                if iscell(val)
                    val = val{1};
                end
                if ischar(val) || isstring(val)
                    val = str2double(val);
                end
                
                % Store in struct
                if isnumeric(val) && isscalar(val) && ~isnan(val)
                    runInfo.postValues.(colName) = val;
                end
            end
        end
        fprintf('  Found %d detector columns\n', length(detectorCols));
        
        % Find detection points for algorithm detectors only
        % Find detection points for algorithm detectors only
        % Find detection points for algorithm detectors only
        runInfo.detections = containers.Map();
        for j = 1:length(detectorCols)
            detectorName = detectorCols{j};
            detectorValues = runData.(detectorName);
            
            % Find first non-zero value - this IS the detected pressure
            nonZeroIdx = find(detectorValues ~= 0, 1);
            detection = struct();
            
            if ~isempty(nonZeroIdx)
                detectedPressure = detectorValues(nonZeroIdx);  % The detector's value IS the pressure
                
                % Use the time when the detector column transitions from 0 to non-zero
                detection.time = runInfo.time(nonZeroIdx);
                detection.pressure = detectedPressure;
                detection.value = detectedPressure;
                detection.detected = true;
                
                % Calculate error from ground truth if available
                % Calculate error from ground truth if available
                if runInfo.hasGroundTruth
                    detection.error = detection.pressure - runInfo.groundTruthPressure;
                    detection.absError = abs(detection.error);
                else
                    detection.error = NaN;
                    detection.absError = NaN;
                end
                
                % Calculate error from SBP reference if available
                if runInfo.hasSBPReference
                    detection.sbpError = detection.pressure - runInfo.sbpReference;
                    detection.sbpAbsError = abs(detection.sbpError);
                else
                    detection.sbpError = NaN;
                    detection.sbpAbsError = NaN;
                end
            else
                % All zeros - no detection
                detection.time = NaN;
                detection.pressure = NaN;
                detection.value = NaN;
                detection.detected = false;
                detection.error = NaN;
                detection.absError = NaN;
            end
            runInfo.detections(detectorName) = detection;
        end
        
        % Add to runs list
        allRuns{runCounter} = runInfo;
        runCounter = runCounter + 1;
    end
end

% Get all unique detectors
allDetectors = {};
for i = 1:length(allRuns)
    allDetectors = union(allDetectors, allRuns{i}.detectors);
end

fprintf('\nFound %d total run(s) across all files.\n', length(allRuns));
fprintf('Found %d unique detectors across all runs.\n', length(allDetectors));

% Count runs with ground truth
runsWithGroundTruth = sum(cellfun(@(r) r.hasGroundTruth, allRuns));
fprintf('Found %d run(s) with ground truth data.\n\n', runsWithGroundTruth);

% Count runs with SBP reference
runsWithSBPRef = sum(cellfun(@(r) r.hasSBPReference, allRuns));
fprintf('Found %d run(s) with SBP reference in filename.\n\n', runsWithSBPRef);

%% Create Main Tabbed Figure
screenSize = get(0, 'ScreenSize');
figWidth = 1600;
figHeight = 900;
figLeft = max(1, (screenSize(3) - figWidth) / 2);
figBottom = max(1, (screenSize(4) - figHeight) / 2);

mainFig = figure('Name', 'Blood Pressure Analysis', ...
    'Position', [figLeft, figBottom, figWidth, figHeight], ...
    'NumberTitle', 'off');

% Create tab group
tabGroup = uitabgroup(mainFig);

% Create comparison tabs
compTab = uitab(tabGroup, 'Title', 'Detector Comparison');
createComparisonTab(compTab, allRuns, allDetectors);

% Create ground truth error analysis tab if applicable
if runsWithGroundTruth > 0
    gtTab = uitab(tabGroup, 'Title', 'Ground Truth Analysis');
    createGroundTruthTab(gtTab, allRuns, allDetectors);
end

% Create SBP reference error analysis tab if applicable
if runsWithSBPRef > 0
    sbpTab = uitab(tabGroup, 'Title', 'SBP Reference Analysis');
    createSBPReferenceTab(sbpTab, allRuns, allDetectors);
end

blTab = uitab(tabGroup, 'Title', 'Baseline Analysis');
createBLAnalysisTab(blTab, allRuns, allDetectors);
blTab_v2 = uitab(tabGroup, 'Title', 'Baseline Analysis Error from Ground Truth');
createBLAnalysisTab_v2(blTab_v2, allRuns, allDetectors);

drvTab = uitab(tabGroup, 'Title', 'Derivative Analysis');
createDRVAnalysisTab(drvTab, allRuns, allDetectors);

% Create tabs for each run
for i = 1:length(allRuns)
    [~, baseName, ~] = fileparts(allRuns{i}.filename);
    if allRuns{i}.runIdx > 0
        tabName = sprintf('%s-R%d', baseName, allRuns{i}.runIdx);
    else
        tabName = baseName;
    end
    tab = uitab(tabGroup, 'Title', tabName);
    createFileTab(tab, allRuns{i});
end

fprintf('\nAnalysis complete! Use the tabs to navigate between views.\n');

%% Helper Functions
function createFileTab(parentTab, runInfo)
    ax = axes('Parent', parentTab, 'Position', [0.08, 0.15, 0.78, 0.75]);
    
    % Safety check
    if ~isfield(runInfo, 'hasAusPulse')
        runInfo.hasAusPulse = false;
        runInfo.pulseHeardIndices = [];
    end
    
    % Debug MAP data availability
    fprintf('Creating tab for run, hasAusPulse=%d, numPulses=%d\n', ...
        runInfo.hasAusPulse, length(runInfo.pulseHeardIndices));
    
    if isfield(runInfo, 'hasMAP')
        fprintf('  runInfo.hasMAP field exists: %d\n', runInfo.hasMAP);
        if runInfo.hasMAP && isfield(runInfo, 'oscAmp')
            fprintf('  oscAmp field exists, range: %.2f to %.2f\n', ...
                min(runInfo.oscAmp), max(runInfo.oscAmp));
        end
    else
        fprintf('  runInfo.hasMAP field does NOT exist\n');
    end
    
    % Plot PPG on left y-axis
    yyaxis left
    hold on; grid on;
    
    % --- Raw PPG (optional)
    if runInfo.hasRawPPG
        h3 = plot(runInfo.time, runInfo.rawPPG, 'Color', [1 0.6 0.6], 'LineWidth', 0.8);
    end
    
    % --- Primary filtered PPG signal
    h2 = plot(runInfo.time, runInfo.ppg, 'r-', 'LineWidth', 1.0);

    % --- Plot Oscillometric Amplitude Envelope (Osc_Amp) ---
    h_env = [];
    if isfield(runInfo, 'hasMAP') && runInfo.hasMAP && isfield(runInfo, 'oscAmp')
        fprintf('  PLOTTING Osc_Amp envelope (scaled x50)\n');
        h_env = plot(runInfo.time, runInfo.oscAmp, 'Color', [0 0.7 0], ...
                     'LineWidth', 1.5);
    else
        fprintf('  NOT plotting Osc_Amp - hasMAP=%d, oscAmp exists=%d\n', ...
            isfield(runInfo, 'hasMAP') && runInfo.hasMAP, ...
            isfield(runInfo, 'oscAmp'));
    end

    % Set left y-axis label and color
    ylabel('PPG and Raw PPG', 'FontSize', 12);
    ax.YColor = 'r';
    
    % Plot Pressure on right y-axis
    yyaxis right
    hold on;
    h1 = plot(runInfo.time, runInfo.pressure, 'b-', 'LineWidth', 1.5);
    ylabel('Pressure (mmHg)', 'FontSize', 12);
    ax.YColor = 'b';

    % --- Plot Baseline Beat markers on pressure ---
    % if isfield(runInfo, 'hasBaselineBeat') && runInfo.hasBaselineBeat
    %     beatIdx = find(runInfo.baselineBeat == 1);
    % 
    %     if ~isempty(beatIdx)
    %         beatTimes = runInfo.time(beatIdx);
    %         beatPressures = runInfo.pressure(beatIdx);
    % 
    %         h_baseline = plot(beatTimes, beatPressures, 'ks', ...
    %             'MarkerSize', 5, ...
    %             'MarkerFaceColor', 'k', ...
    %             'LineWidth', 1.2);
    %     end
    % end

    
    % Plot ground truth marker if available
    hasGroundTruth = false;
    % if runInfo.hasGroundTruth
    %     hasGroundTruth = true;
    %     % Plot ground truth line
    %     h_gt = xline(runInfo.groundTruthTime, ':', 'Color', [0 0.5 0], 'LineWidth', 2.5);
    %     plot(runInfo.groundTruthTime, runInfo.groundTruthPressure, 'p', ...
    %         'Color', [0 0.5 0], 'MarkerSize', 14, 'LineWidth', 2, 'MarkerFaceColor', [0 0.8 0]);
    % end
    
    % Plot AUS_PULSE_HEARD markers if available
    hasAusPulseData = false;
    % if runInfo.hasAusPulse && ~isempty(runInfo.pulseHeardIndices)
    %     hasAusPulseData = true;
    %     fprintf('  Plotting %d stethoscope pulses\n', length(runInfo.pulseHeardIndices));
    % 
    %     pulseTimes = runInfo.time(runInfo.pulseHeardIndices);
    %     pulsePressures = runInfo.pressure(runInfo.pulseHeardIndices);
    %     pulsePPG = runInfo.ppg(runInfo.pulseHeardIndices);
    % 
    %     % Plot X markers on pressure (right axis)
    %     h_pulse_pressure = plot(pulseTimes, pulsePressures, 'kx', 'MarkerSize', 6, 'LineWidth', 1.5);
    % 
    %     % Switch to left axis for PPG markers
    %     yyaxis left
    %     plot(pulseTimes, pulsePPG, 'kx', 'MarkerSize', 6, 'LineWidth', 1.5);
    %     if runInfo.hasRawPPG
    %         pulseRawPPG = runInfo.rawPPG(runInfo.pulseHeardIndices);
    %         plot(pulseTimes, pulseRawPPG, 'kx', 'MarkerSize', 6, 'LineWidth', 1.5);
    %     end
    % 
    %     % Switch back to right axis
    %     yyaxis right
    % end
    
    % Sort detectors by detection pressure
    detectorPressures = zeros(length(runInfo.detectors), 1);
    for j = 1:length(runInfo.detectors)
        detName = runInfo.detectors{j};
        if isKey(runInfo.detections, detName)
            detection = runInfo.detections(detName);
            if detection.detected
                detectorPressures(j) = detection.pressure;
            else
                detectorPressures(j) = inf;
            end
        else
            detectorPressures(j) = inf;
        end
    end
    [~, sortIdx] = sort(detectorPressures);
    sortedDetectors = runInfo.detectors(sortIdx);
    
    % Build legend
    colors = lines(length(sortedDetectors));
    if runInfo.hasRawPPG
        legendHandles = [h1, h2, h3];
        legendLabels = {'Pressure', 'PPG (Filtered)', 'PPG (Raw)'};
    else
        legendHandles = [h1, h2];
        legendLabels = {'Pressure', 'PPG'};
    end
    
    % Add Osc_Amp envelope if present
    if ~isempty(h_env)
        legendHandles(end+1) = h_env;
        legendLabels{end+1} = '(x50) Oscillometric Envelope (Osc\_Amp)';
    end
    
    % Add ground truth to legend
    if hasGroundTruth
        legendHandles(end+1) = h_gt;
        legendLabels{end+1} = sprintf('Ground Truth: %.1f mmHg', runInfo.groundTruthPressure);
    end
    
    % Add stethoscope to legend
    if hasAusPulseData
        legendHandles(end+1) = h_pulse_pressure;
        legendLabels{end+1} = sprintf('Stethoscope');
    end

    % Add baseline beat legend entry
    if exist('h_baseline', 'var')
        legendHandles(end+1) = h_baseline;
        legendLabels{end+1} = 'Baseline Pressure Beat';
    end
    
    % Plot detector lines
    % for j = 1:length(sortedDetectors)
    %     detName = sortedDetectors{j};
    %     safeName = strrep(detName, '_', '\_');
    %     if isKey(runInfo.detections, detName)
    %         detection = runInfo.detections(detName);
    %         detTime = detection.time;
    %         detPressure = detection.pressure;
    % 
    %         if detection.detected && ~isnan(detTime)
    %             xline(detTime, '--', 'Color', colors(j,:), 'LineWidth', 1.5);
    %             h = plot(detTime, detPressure, 'o', 'Color', colors(j,:), ...
    %                 'MarkerSize', 10, 'LineWidth', 2);
    %             legendHandles(end+1) = h;
    % 
    %             % Add error to legend if ground truth available
    %             if hasGroundTruth
    %                 legendLabels{end+1} = sprintf('%s: %.1f mmHg (Δ=%.1f)', ...
    %                     safeName, detPressure, detection.error);
    %             else
    %                 legendLabels{end+1} = sprintf('%s: %.1f mmHg', safeName, detPressure);
    %             end
    %         else
    %             h = plot(NaN, NaN, 'o', 'Color', colors(j,:), 'MarkerSize', 10, 'LineWidth', 2);
    %             legendHandles(end+1) = h;
    %             legendLabels{end+1} = sprintf('%s: No detection', safeName);
    %         end
    %     end
    % end

    % Create xlabel with blood pressure values
    xlabelStr = 'Time (s)';
    
    if isfield(runInfo, 'post') && iscell(runInfo.post) && ~isempty(runInfo.post) && ...
       isfield(runInfo, 'postValues') && ~isempty(fieldnames(runInfo.postValues))
        
        parts = {};
        
        % Loop through each column name in runInfo.post
        for k = 1:numel(runInfo.post)
            colName = runInfo.post{k};
            
            % Check if we have a value stored for this column
            if isfield(runInfo.postValues, colName)
                val = runInfo.postValues.(colName);
                parts{end+1} = sprintf('%s: %.1f', colName, val);
            end
        end
        
        if ~isempty(parts)
            xlabelStr = sprintf('%s\n%s', xlabelStr, strjoin(parts, ' | '));
        end
    end
    
    xlabel(xlabelStr, 'FontSize', 12, 'Interpreter', 'none');
    
    titleStr = sprintf('%s', runInfo.displayName);
    if hasGroundTruth
        titleStr = sprintf('%s - Ground Truth: %.1f mmHg', titleStr, runInfo.groundTruthPressure);
    end
    title(titleStr, 'FontSize', 14, 'Interpreter', 'none');
    
    lgd = legend(legendHandles, legendLabels, ...
        'Location', 'eastoutside', ...
        'FontSize', 9);
    lgd.NumColumns = 1;
    lgd.ItemHitFcn = @(src, evt) toggleVisibility(evt);
end

function toggleVisibility(evt)
    obj = evt.Peer;
    if strcmp(obj.Visible, 'on')
        obj.Visible = 'off';
    else
        obj.Visible = 'on';
    end
end


function createComparisonTab(parentTab, allRuns, allDetectors)
    numRuns = length(allRuns);
    numDetectors = length(allDetectors);
    
    runLabels = cell(numRuns, 1);
    for i = 1:numRuns
        [~, baseName, ~] = fileparts(allRuns{i}.filename);
        if allRuns{i}.runIdx > 0
            runLabels{i} = sprintf('%s-R%d', baseName, allRuns{i}.runIdx);
        else
            runLabels{i} = baseName;
        end
    end
    
    % BUILD THE DETECTION PRESSURES MATRIX
    detectionPressures = nan(numDetectors, numRuns);
    for i = 1:numRuns
        for j = 1:numDetectors
            detName = allDetectors{j};
            if isKey(allRuns{i}.detections, detName)
                detection = allRuns{i}.detections(detName);
                if detection.detected
                    detectionPressures(j, i) = detection.pressure;
                end
            end
        end
    end
    
    % Two plots: Mean pressure on top, success rate on bottom (full width)
    ax3 = axes('Parent', parentTab, 'Position', [0.08, 0.63, 0.86, 0.32]);
    ax4 = axes('Parent', parentTab, 'Position', [0.08, 0.18, 0.86, 0.32]);
    
    % Mean and Std
    axes(ax3);
    meanPressures = mean(detectionPressures, 2, 'omitnan');
    stdPressures = std(detectionPressures, 0, 2, 'omitnan');
    
    fprintf('\nMean pressures:\n');
    for j = 1:numDetectors
        fprintf('%s: %.2f ± %.2f\n', allDetectors{j}, meanPressures(j), stdPressures(j));
    end
    
    errorbar(1:numDetectors, meanPressures, stdPressures, 'o-', 'LineStyle', 'none', 'LineWidth', 1.5, 'MarkerSize', 8);
    grid on;
    title('Mean Detected Pressure ± Std Dev', 'FontSize', 14);
    % xlabel('Detector', 'FontSize', 12);
    ylabel('Pressure (mmHg)', 'FontSize', 12);
    shortLabels = shortenDetectorNames(allDetectors);
    set(gca, 'XTick', 1:numDetectors, 'XTickLabel', shortLabels,'TickLabelInterpreter', 'none', 'XTickLabelRotation', 90, 'FontSize', 8);
    
    % Better y-axis limits that show the variation
    minVal = min(meanPressures - stdPressures, [], 'omitnan');
    maxVal = max(meanPressures + stdPressures, [], 'omitnan');
    if ~isnan(minVal) && ~isnan(maxVal) && maxVal > minVal
        yRange = maxVal - minVal;
        ylim([max(0, minVal - 0.1*yRange), maxVal + 0.1*yRange]);
    end
    
    % Success Rate (full width bottom)
    axes(ax4);
    successRate = sum(~isnan(detectionPressures), 2) / numRuns * 100;
    bar(successRate);
    grid on;
    title('Detection Success Rate', 'FontSize', 14);
    % xlabel('Detector', 'FontSize', 12);
    ylabel('Success Rate (%)', 'FontSize', 12);
    shortLabels = shortenDetectorNames(allDetectors);
    set(gca, 'XTick', 1:numDetectors, 'XTickLabel', shortLabels,'TickLabelInterpreter', 'none', 'XTickLabelRotation', 90, 'FontSize', 8);
    ylim([0 110]);
end
function createGroundTruthTab(parentTab, allRuns, allDetectors)
    % Filter to runs with ground truth
    runsWithGT = allRuns(cellfun(@(r) r.hasGroundTruth, allRuns));
    numRuns = length(runsWithGT);
    numDetectors = length(allDetectors);
    
    if numRuns == 0
        annotation(parentTab, 'textbox', [0.3, 0.4, 0.4, 0.2], ...
            'String', 'No runs with ground truth data found.', ...
            'FontSize', 14, 'HorizontalAlignment', 'center', 'EdgeColor', 'none');
        return;
    end
    
    % Extract errors
    errors = nan(numDetectors, numRuns);
    absErrors = nan(numDetectors, numRuns);
    
    for i = 1:numRuns
        for j = 1:numDetectors
            detName = allDetectors{j};
            if isKey(runsWithGT{i}.detections, detName)
                detection = runsWithGT{i}.detections(detName);
                if detection.detected
                    errors(j, i) = detection.error;
                    absErrors(j, i) = detection.absError;
                end
            end
        end
    end
    
    % Calculate statistics
    meanError = mean(errors, 2, 'omitnan');
    stdError = std(errors, 0, 2, 'omitnan');
    meanAbsError = mean(absErrors, 2, 'omitnan');
    stdAbsError = std(absErrors, 0, 2, 'omitnan');
    
    % Create axes - only 2 plots now
    ax2 = axes('Parent', parentTab, 'Position', [0.08, 0.63, 0.86, 0.32]);
    ax4 = axes('Parent', parentTab, 'Position', [0.08, 0.18, 0.86, 0.32]);
    
    % Plot 1: Absolute error (top)
    axes(ax2);
    errorbar(1:numDetectors, meanAbsError, stdAbsError, 'o-', 'LineStyle', 'none', 'LineWidth', 2, 'MarkerSize', 8, 'Color', [0.8 0.2 0.2]);
    grid on;
    ylabel('Mean Absolute Error (mmHg)', 'FontSize', 12);
    title('Mean Absolute Error from Ground Truth ± Std Dev', 'FontSize', 14);
    shortLabels = shortenDetectorNames(allDetectors);
    set(gca, 'XTick', 1:numDetectors, 'XTickLabel', shortLabels, 'TickLabelInterpreter', 'none', 'XTickLabelRotation', 90, 'FontSize', 8);
    
    % Plot 2: Ranking by absolute error (bottom)
    axes(ax4);
    [sortedMAE, sortIdx] = sort(meanAbsError, 'ascend');
    sortedDetectors = allDetectors(sortIdx);
    sortedStdAbsError = stdAbsError(sortIdx);  % Sort the std deviations to match
    
    bar(sortedMAE, 'FaceColor', [0.3 0.5 0.8]);
    hold on;
    errorbar(1:numDetectors, sortedMAE, sortedStdAbsError, 'k.', 'LineWidth', 1.5, 'CapSize', 8);
    hold off;
    
    grid on;
    xlabel('Detector (Ranked)', 'FontSize', 12);
    ylabel('Mean Absolute Error (mmHg)', 'FontSize', 12);
    title('Detector Ranking by Accuracy', 'FontSize', 14);
    shortLabels = shortenDetectorNames(sortedDetectors);
    set(gca, 'XTick', 1:numDetectors, 'XTickLabel', shortLabels, 'TickLabelInterpreter', 'none', 'XTickLabelRotation', 90, 'FontSize', 8);
    
    % Print summary
    fprintf('\n=== GROUND TRUTH ANALYSIS SUMMARY ===\n');
    fprintf('Analyzed %d runs with ground truth data\n\n', numRuns);
    fprintf('Detector Rankings by Mean Absolute Error:\n');
    for i = 1:min(numDetectors, 10)
        idx = sortIdx(i);
        fprintf('%2d. %s: MAE = %.2f ± %.2f mmHg, Bias = %.2f mmHg\n', ...
            i, allDetectors{idx}, meanAbsError(idx), stdAbsError(idx), meanError(idx));
    end
    fprintf('\n');
end

function createBLAnalysisTab(parentTab, allRuns, allDetectors)
    blDetectors = allDetectors(startsWith(allDetectors, 'BL_'));
    for i = 1:length(blDetectors)
        fprintf('%d: %s\n', i, blDetectors{i});
    end
    
    if isempty(blDetectors)
        annotation(parentTab, 'textbox', [0.3, 0.4, 0.4, 0.2], ...
            'String', 'No BL detectors found in the data.', ...
            'FontSize', 14, 'HorizontalAlignment', 'center', 'EdgeColor', 'none');
        return;
    end
    
    numBL = length(blDetectors);
    params = zeros(numBL, 3);
    validDetectors = true(numBL, 1);
    
    for i = 1:numBL
        % Updated regex to handle underscore-separated decimals and optional _C suffix
        % Matches: BL_W20_T2_0_D10 or BL_W60_T2_50_D5_C2
        tokens = regexp(blDetectors{i}, 'BL_W(\d+)_T(\d+)_(\d+)_D(\d+)', 'tokens');
        if ~isempty(tokens) && ~isempty(tokens{1})
            W = str2double(tokens{1}{1});
            T_integer = str2double(tokens{1}{2});
            T_decimal = str2double(tokens{1}{3});
            D = str2double(tokens{1}{4});
            
            % Convert T from underscore format to decimal (e.g., 2_5 -> 2.5, 3_0 -> 3.0)
            T = T_integer + T_decimal / 10;
            
            params(i, :) = [W, T, D];
            fprintf('Parsed %s -> W=%d, T=%.1f, D=%d\n', blDetectors{i}, W, T, D);
        else
            validDetectors(i) = false;
            fprintf('Warning: Could not parse detector name: %s\n', blDetectors{i});
        end
    end
    
    % Filter to only valid detectors
    blDetectors = blDetectors(validDetectors);
    params = params(validDetectors, :);
    numBL = length(blDetectors);
    
    if numBL == 0
        annotation(parentTab, 'textbox', [0.3, 0.4, 0.4, 0.2], ...
            'String', 'No BL detectors with valid naming format found.', ...
            'FontSize', 14, 'HorizontalAlignment', 'center', 'EdgeColor', 'none');
        return;
    end
    
    fprintf('\nSuccessfully parsed %d BL detectors\n', numBL);
    
    numRuns = length(allRuns);
    detPressures = nan(numBL, numRuns);
    for i = 1:numRuns
        for j = 1:numBL
            detName = blDetectors{j};
            if isKey(allRuns{i}.detections, detName)
                detection = allRuns{i}.detections(detName);
                if detection.detected
                    detPressures(j, i) = detection.pressure;
                end
            end
        end
    end
    
    ax1 = axes('Parent', parentTab, 'Position', [0.08, 0.63, 0.38, 0.32]);
    ax2 = axes('Parent', parentTab, 'Position', [0.56, 0.63, 0.38, 0.32]);
    ax3 = axes('Parent', parentTab, 'Position', [0.08, 0.18, 0.38, 0.32]);
    
    % Window Size effect
    axes(ax1);
    uniqueW = unique(params(:, 1));
    meanByW = zeros(length(uniqueW), 1);
    stdByW = zeros(length(uniqueW), 1);
    for i = 1:length(uniqueW)
        idx = params(:, 1) == uniqueW(i);
        meanByW(i) = mean(detPressures(idx, :), 'all', 'omitnan');
        stdByW(i) = std(detPressures(idx, :), 0, 'all', 'omitnan');
    end
    errorbar(uniqueW, meanByW, stdByW, 'o-', 'LineStyle', 'none', 'LineWidth', 2, 'MarkerSize', 8);
    grid on; xlabel('Window Size (W)', 'FontSize', 12);
    ylabel('Mean Detected Pressure (mmHg)', 'FontSize', 12);
    title('Effect of Window Size', 'FontSize', 14);
    
    % Threshold effect
    axes(ax2);
    uniqueT = unique(params(:, 2));
    meanByT = zeros(length(uniqueT), 1);
    stdByT = zeros(length(uniqueT), 1);
    for i = 1:length(uniqueT)
        idx = params(:, 2) == uniqueT(i);
        meanByT(i) = mean(detPressures(idx, :), 'all', 'omitnan');
        stdByT(i) = std(detPressures(idx, :), 0, 'all', 'omitnan');
    end
    errorbar(uniqueT, meanByT, stdByT, 'o-', 'LineStyle', 'none', 'LineWidth', 2, 'MarkerSize', 8);
    grid on; xlabel('Threshold Multiplier (T)', 'FontSize', 12);
    ylabel('Mean Detected Pressure (mmHg)', 'FontSize', 12);
    title('Effect of Threshold', 'FontSize', 14);
    
    % Minimum Deviation effect
    axes(ax3);
    uniqueD = unique(params(:, 3));
    meanByD = zeros(length(uniqueD), 1);
    stdByD = zeros(length(uniqueD), 1);
    for i = 1:length(uniqueD)
        idx = params(:, 3) == uniqueD(i);
        meanByD(i) = mean(detPressures(idx, :), 'all', 'omitnan');
        stdByD(i) = std(detPressures(idx, :), 0, 'all', 'omitnan');
    end
    errorbar(uniqueD, meanByD, stdByD, 'o-', 'LineStyle', 'none', 'LineWidth', 2, 'MarkerSize', 8);
    grid on; xlabel('Minimum Deviation (D)', 'FontSize', 12);
    ylabel('Mean Detected Pressure (mmHg)', 'FontSize', 12);
    title('Effect of Minimum Deviation', 'FontSize', 14);
end

function createDRVAnalysisTab(parentTab, allRuns, allDetectors)
    drvDetectors = allDetectors(startsWith(allDetectors, 'DRV_'));
    if isempty(drvDetectors)
        annotation(parentTab, 'textbox', [0.3, 0.4, 0.4, 0.2], ...
            'String', 'No DRV detectors found in the data.', ...
            'FontSize', 14, 'HorizontalAlignment', 'center', 'EdgeColor', 'none');
        return;
    end
    
    numDRV = length(drvDetectors);
    params = zeros(numDRV, 2);
    for i = 1:numDRV
        tokens = regexp(drvDetectors{i}, 'DRV_W(\d+)_T([\d.]+)', 'tokens');
        if ~isempty(tokens)
            params(i, :) = [str2double(tokens{1}{1}), str2double(tokens{1}{2})];
        end
    end
    
    numRuns = length(allRuns);
    detPressures = nan(numDRV, numRuns);
    for i = 1:numRuns
        for j = 1:numDRV
            detName = drvDetectors{j};
            if isKey(allRuns{i}.detections, detName)
                detection = allRuns{i}.detections(detName);
                detPressures(j, i) = detection.pressure;
            end
        end
    end
    
    ax1 = axes('Parent', parentTab, 'Position', [0.1, 0.3, 0.35, 0.5]);
    ax2 = axes('Parent', parentTab, 'Position', [0.55, 0.3, 0.35, 0.5]);
    
    axes(ax1);
    uniqueW = unique(params(:, 1));
    meanByW = zeros(length(uniqueW), 1);
    stdByW = zeros(length(uniqueW), 1);
    for i = 1:length(uniqueW)
        idx = params(:, 1) == uniqueW(i);
        meanByW(i) = mean(detPressures(idx, :), 'all', 'omitnan');
        stdByW(i) = std(detPressures(idx, :), 0, 'all', 'omitnan');
    end
    errorbar(uniqueW, meanByW, stdByW, 'o-', 'LineStyle', 'none', 'LineWidth', 2, 'MarkerSize', 8);
    grid on; xlabel('Window Size (W)', 'FontSize', 12);
    ylabel('Mean Detected Pressure (mmHg)', 'FontSize', 12);
    title('Effect of Window Size', 'FontSize', 14);
    
    axes(ax2);
    uniqueT = unique(params(:, 2));
    meanByT = zeros(length(uniqueT), 1);
    stdByT = zeros(length(uniqueT), 1);
    for i = 1:length(uniqueT)
        idx = params(:, 2) == uniqueT(i);
        meanByT(i) = mean(detPressures(idx, :), 'all', 'omitnan');
        stdByT(i) = std(detPressures(idx, :), 0, 'all', 'omitnan');
    end
    errorbar(uniqueT, meanByT, stdByT, 'o-', 'LineStyle', 'none', 'LineWidth', 2, 'MarkerSize', 8);
    grid on; xlabel('Threshold (T)', 'FontSize', 12);
    ylabel('Mean Detected Pressure (mmHg)', 'FontSize', 12);
    title('Effect of Threshold', 'FontSize', 14);
end

function shortLabels = shortenDetectorNames(detectors)
    shortLabels = cell(size(detectors));
    for i = 1:length(detectors)
        label = detectors{i};
        % Remove common prefixes
        label = strrep(label, 'BL_', 'BL');
        label = strrep(label, 'DRV_', 'DRV');
        % Replace underscores with spaces
        label = strrep(label, '_', ',');
        % Replace periods with underscores (if any)
        shortLabels{i} = label;
    end
end

function createSBPReferenceTab(parentTab, allRuns, allDetectors)
    % Filter to runs with SBP reference
    runsWithSBP = allRuns(cellfun(@(r) r.hasSBPReference, allRuns));
    numRuns = length(runsWithSBP);
    numDetectors = length(allDetectors);
    
    if numRuns == 0
        annotation(parentTab, 'textbox', [0.3, 0.4, 0.4, 0.2], ...
            'String', 'No runs with SBP reference in filename found.', ...
            'FontSize', 14, 'HorizontalAlignment', 'center', 'EdgeColor', 'none');
        return;
    end
    
    % Extract errors
    errors = nan(numDetectors, numRuns);
    absErrors = nan(numDetectors, numRuns);
    
    for i = 1:numRuns
        for j = 1:numDetectors
            detName = allDetectors{j};
            if isKey(runsWithSBP{i}.detections, detName)
                detection = runsWithSBP{i}.detections(detName);
                if detection.detected
                    errors(j, i) = detection.sbpError;
                    absErrors(j, i) = detection.sbpAbsError;
                end
            end
        end
    end
    
    % Calculate statistics
    meanError = mean(errors, 2, 'omitnan');
    stdError = std(errors, 0, 2, 'omitnan');
    meanAbsError = mean(absErrors, 2, 'omitnan');
    stdAbsError = std(absErrors, 0, 2, 'omitnan');
    
    % Create axes - only 2 plots now
    ax2 = axes('Parent', parentTab, 'Position', [0.08, 0.63, 0.86, 0.32]);
    ax4 = axes('Parent', parentTab, 'Position', [0.08, 0.18, 0.86, 0.32]);
    
    % Plot 1: Absolute error (top)
    axes(ax2);
    errorbar(1:numDetectors, meanAbsError, stdAbsError, 'o-', 'LineStyle', 'none', 'LineWidth', 2, 'MarkerSize', 8, 'Color', [0.8 0.2 0.2]);
    grid on;
    ylabel('Mean Absolute Error (mmHg)', 'FontSize', 12);
    title('Mean Absolute Error from SBP Reference ± Std Dev', 'FontSize', 14);
    shortLabels = shortenDetectorNames(allDetectors);
    set(gca, 'XTick', 1:numDetectors, 'XTickLabel', shortLabels, 'TickLabelInterpreter', 'none', 'XTickLabelRotation', 90, 'FontSize', 8);
    
    % Plot 2: Ranking by absolute error (bottom)
    axes(ax4);
    [sortedMAE, sortIdx] = sort(meanAbsError, 'ascend');
    sortedDetectors = allDetectors(sortIdx);
    sortedStdAbsError = stdAbsError(sortIdx);  % Sort the std deviations to match
    
    bar(sortedMAE, 'FaceColor', [0.3 0.5 0.8]);
    hold on;
    errorbar(1:numDetectors, sortedMAE, sortedStdAbsError, 'k.', 'LineWidth', 1.5, 'CapSize', 8);
    hold off;
    
    grid on;
    xlabel('Detector (Ranked)', 'FontSize', 12);
    ylabel('Mean Absolute Error (mmHg)', 'FontSize', 12);
    title('Detector Ranking by Accuracy (vs SBP Reference)', 'FontSize', 14);
    shortLabels = shortenDetectorNames(sortedDetectors);
    set(gca, 'XTick', 1:numDetectors, 'XTickLabel', shortLabels, 'TickLabelInterpreter', 'none', 'XTickLabelRotation', 90, 'FontSize', 8);
    
    % Print summary
    fprintf('\n=== SBP REFERENCE ANALYSIS SUMMARY ===\n');
    fprintf('Analyzed %d runs with SBP reference in filename\n\n', numRuns);
    fprintf('Detector Rankings by Mean Absolute Error:\n');
    for i = 1:min(numDetectors, 10)
        idx = sortIdx(i);
        fprintf('%2d. %s: MAE = %.2f ± %.2f mmHg, Bias = %.2f mmHg\n', ...
            i, allDetectors{idx}, meanAbsError(idx), stdAbsError(idx), meanError(idx));
    end
    fprintf('\n');
end
function createBLAnalysisTab_v2(parentTab, allRuns, allDetectors)
    % Enhanced BL Analysis with Error Statistics
    
    blDetectors = allDetectors(startsWith(allDetectors, 'BL_'));
    
    if isempty(blDetectors)
        annotation(parentTab, 'textbox', [0.3, 0.4, 0.4, 0.2], ...
            'String', 'No BL detectors found in the data.', ...
            'FontSize', 14, 'HorizontalAlignment', 'center', 'EdgeColor', 'none');
        return;
    end
    
    % Filter to runs with ground truth
    runsWithGT = allRuns(cellfun(@(r) r.hasGroundTruth, allRuns));
    numRunsGT = length(runsWithGT);
    
    if numRunsGT == 0
        annotation(parentTab, 'textbox', [0.3, 0.4, 0.4, 0.2], ...
            'String', 'No runs with ground truth data found for error analysis.', ...
            'FontSize', 14, 'HorizontalAlignment', 'center', 'EdgeColor', 'none');
        return;
    end
    
    numBL = length(blDetectors);
    params = zeros(numBL, 3);
    validDetectors = true(numBL, 1);
    
    % Parse detector parameters
    for i = 1:numBL
        tokens = regexp(blDetectors{i}, 'BL_W(\d+)_T(\d+)_(\d+)_D(\d+)', 'tokens');
        if ~isempty(tokens) && ~isempty(tokens{1})
            W = str2double(tokens{1}{1});
            T_integer = str2double(tokens{1}{2});
            T_decimal = str2double(tokens{1}{3});
            D = str2double(tokens{1}{4});
            T = T_integer + T_decimal / 10;
            params(i, :) = [W, T, D];
        else
            validDetectors(i) = false;
        end
    end
    
    % Filter to only valid detectors
    blDetectors = blDetectors(validDetectors);
    params = params(validDetectors, :);
    numBL = length(blDetectors);
    
    if numBL == 0
        annotation(parentTab, 'textbox', [0.3, 0.4, 0.4, 0.2], ...
            'String', 'No BL detectors with valid naming format found.', ...
            'FontSize', 14, 'HorizontalAlignment', 'center', 'EdgeColor', 'none');
        return;
    end
    
    fprintf('\nSuccessfully parsed %d BL detectors for error analysis\n', numBL);
    
    % Extract errors from ground truth runs
    errors = nan(numBL, numRunsGT);
    
    for i = 1:numRunsGT
        for j = 1:numBL
            detName = blDetectors{j};
            if isKey(runsWithGT{i}.detections, detName)
                detection = runsWithGT{i}.detections(detName);
                if detection.detected
                    errors(j, i) = detection.error;
                end
            end
        end
    end
    
    % Create axes - 3 plots stacked vertically
    ax1 = axes('Parent', parentTab, 'Position', [0.08, 0.68, 0.86, 0.26]);  % W - Mean Error
    ax2 = axes('Parent', parentTab, 'Position', [0.08, 0.38, 0.86, 0.26]);  % T - Mean Error
    ax3 = axes('Parent', parentTab, 'Position', [0.08, 0.08, 0.86, 0.26]);  % D - Mean Error
    
    % --- Window Size Analysis ---
    uniqueW = sort(unique(params(:, 1)));
    meanErrByW = zeros(length(uniqueW), 1);
    stdErrByW = zeros(length(uniqueW), 1);
    
    for i = 1:length(uniqueW)
        idx = params(:, 1) == uniqueW(i);
        meanErrByW(i) = mean(errors(idx, :), 'all', 'omitnan');
        stdErrByW(i) = std(errors(idx, :), 0, 'all', 'omitnan');
    end
    
    % Plot Window Size - Mean Error
    axes(ax1);
    errorbar(uniqueW, meanErrByW, stdErrByW, 'o-', 'LineWidth', 2, 'MarkerSize', 8);
    grid on;
    xlabel('Window Size (W)', 'FontSize', 11);
    ylabel('Mean Error (mmHg)', 'FontSize', 11);
    title('Window Size: Mean Error ± SD', 'FontSize', 12);
    yline(0, 'k--', 'LineWidth', 1);
    
    % --- Threshold Analysis ---
    uniqueT = sort(unique(params(:, 2)));
    meanErrByT = zeros(length(uniqueT), 1);
    stdErrByT = zeros(length(uniqueT), 1);
    
    for i = 1:length(uniqueT)
        idx = params(:, 2) == uniqueT(i);
        meanErrByT(i) = mean(errors(idx, :), 'all', 'omitnan');
        stdErrByT(i) = std(errors(idx, :), 0, 'all', 'omitnan');
    end
    
    % Plot Threshold - Mean Error
    axes(ax2);
    errorbar(uniqueT, meanErrByT, stdErrByT, 'o-', 'LineWidth', 2, 'MarkerSize', 8);
    grid on;
    xlabel('Threshold Multiplier (T)', 'FontSize', 11);
    ylabel('Mean Error (mmHg)', 'FontSize', 11);
    title('Threshold: Mean Error ± SD', 'FontSize', 12);
    yline(0, 'k--', 'LineWidth', 1);
    
    % --- Minimum Deviation Analysis ---
    uniqueD = sort(unique(params(:, 3)));
    meanErrByD = zeros(length(uniqueD), 1);
    stdErrByD = zeros(length(uniqueD), 1);
    
    for i = 1:length(uniqueD)
        idx = params(:, 3) == uniqueD(i);
        meanErrByD(i) = mean(errors(idx, :), 'all', 'omitnan');
        stdErrByD(i) = std(errors(idx, :), 0, 'all', 'omitnan');
    end
    
    % Plot Minimum Deviation - Mean Error
    axes(ax3);
    errorbar(uniqueD, meanErrByD, stdErrByD, 'o-', 'LineWidth', 2, 'MarkerSize', 8);
    grid on;
    xlabel('Minimum Deviation (D)', 'FontSize', 11);
    ylabel('Mean Error (mmHg)', 'FontSize', 11);
    title('Min Deviation: Mean Error ± SD', 'FontSize', 12);
    yline(0, 'k--', 'LineWidth', 1);
    
    % Print summary statistics
    fprintf('\n=== BL PARAMETER ERROR ANALYSIS ===\n');
    fprintf('Analyzed %d BL detectors across %d runs with ground truth\n\n', numBL, numRunsGT);
    
    fprintf('Window Size Effects:\n');
    for i = 1:length(uniqueW)
        fprintf('  W=%d: Mean Error = %.2f ± %.2f mmHg\n', ...
            uniqueW(i), meanErrByW(i), stdErrByW(i));
    end
    
    fprintf('\nThreshold Effects:\n');
    for i = 1:length(uniqueT)
        fprintf('  T=%.1f: Mean Error = %.2f ± %.2f mmHg\n', ...
            uniqueT(i), meanErrByT(i), stdErrByT(i));
    end
    
    fprintf('\nMinimum Deviation Effects:\n');
    for i = 1:length(uniqueD)
        fprintf('  D=%d: Mean Error = %.2f ± %.2f mmHg\n', ...
            uniqueD(i), meanErrByD(i), stdErrByD(i));
    end
    fprintf('\n');
end
