% Blood Pressure Detection CSV Comparison Tool
% This script compares multiple CSV files with different detection algorithms
% for systolic blood pressure measurement
% Updated to parse new CSV format with SUMMARY section
% Added per-person analysis for specific names
clear; clc; close all;

fprintf('=== STARTING BLOOD PRESSURE ANALYSIS ===\n');

%% Configuration
selectedFiles = {};

while true
    [files, path] = uigetfile('*.csv', ...
                              'Select CSV files (Cancel to finish)', ...
                              'MultiSelect', 'on');
    if isequal(files, 0)
        break;
    end
    if ~iscell(files)
        files = {files};
    end
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
    filename = files{fileIdx};
    [~, shortName, ext] = fileparts(filename);
    fprintf(' Loading: %s%s\n', shortName, ext);

    fid = fopen(filename, 'r');
    if fid == -1
        fprintf('  ERROR: Could not open file\n');
        continue;
    end
    fileContent = fread(fid, '*char')';
    fclose(fid);

    summaryStart = strfind(fileContent, '#SUMMARY_START');
    summaryEnd   = strfind(fileContent, '#SUMMARY_END');

    if isempty(summaryStart) || isempty(summaryEnd)
        fprintf('  WARNING: No SUMMARY section found, skipping file\n');
        continue;
    end

    mainDataText = fileContent(1:summaryStart-1);
    tempFile = tempname;
    fid = fopen(tempFile, 'w');
    fprintf(fid, '%s', mainDataText);
    fclose(fid);
    opts = detectImportOptions(tempFile);
    opts.VariableNamesLine = 1;
    data = readtable(tempFile, opts);
    delete(tempFile);
    fprintf('  Initial data has %d rows\n', height(data));

    summaryText  = fileContent(summaryStart:summaryEnd);
    summaryLines = strsplit(summaryText, '\n');
    summaryData  = {};
    for i = 1:length(summaryLines)
        line = strtrim(summaryLines{i});
        if isempty(line) || startsWith(line, '#'); continue; end
        if startsWith(line, 'Detector,');          continue; end
        parts = strsplit(line, ',');
        if length(parts) >= 4
            summaryData{end+1} = struct(...
                'Detector',   strtrim(parts{1}), ...
                'Timestamp',  str2double(parts{2}), ...
                'Pressure',   str2double(parts{3}), ...
                'Confidence', str2double(parts{4}));
        end
    end
    fprintf('  Found %d summary entries\n', length(summaryData));

    run = {data(data.Pressure >= 30, :)};
    fprintf('  Found %d run(s) in this file\n', length(run));

    for runIdx = 1:length(run)
        runData = run{runIdx};
        runInfo = struct();

        [fullPath, shortName, ext] = fileparts(filename);
        [~, folderName] = fileparts(fullPath);
        displayBase = fullfile(folderName, [shortName, ext]);

        if length(run) == 1
            runInfo.filename    = filename;
            runInfo.displayName = displayBase;
        else
            runInfo.filename    = filename;
            runInfo.displayName = sprintf('%s (Run %d)', displayBase, runIdx);
        end
        runInfo.fileIdx = fileIdx;
        runInfo.runIdx  = runIdx;
        runInfo.data    = runData;

        runInfo.hasRawPPG           = false;
        runInfo.hasAusPulse         = false;
        runInfo.pulseHeardIndices   = [];
        runInfo.hasGroundTruth      = false;
        runInfo.groundTruthPressure = NaN;

        % MAP columns (legacy)
        mapCols = {'Osc_Amp','Osc_SBP','MAP','Osc_DBP'};
        if all(ismember(mapCols, runData.Properties.VariableNames))
            runInfo.hasMAP  = true;
            runInfo.oscAmp  = runData.Osc_Amp * 50;
            runInfo.oscSBP  = runData.Osc_SBP;
            runInfo.MAP     = runData.MAP;
            runInfo.oscDBP  = runData.Osc_DBP;
            fprintf('  *** FOUND MAP DATA (legacy columns) ***\n');
        else
            runInfo.hasMAP = false;
        end

        % PPG columns
        if ismember('rawPPGSignal', runData.Properties.VariableNames)
            rawPPG = runData.rawPPGSignal;
            if ismember("PPGSignal", runData.Properties.VariableNames)
                runInfo.ppg = runData.PPGSignal;
            else
                runInfo.ppg = rawPPG;
            end
            rawPPG = rawPPG - mean(rawPPG,'omitnan') + mean(runInfo.ppg,'omitnan');
            runInfo.rawPPG    = rawPPG;
            runInfo.hasRawPPG = true;
        elseif ismember('PPG', runData.Properties.VariableNames)
            rawPPG = runData.PPG;
            if ismember("PPGSignal", runData.Properties.VariableNames)
                runInfo.ppg = runData.PPGSignal;
            else
                runInfo.ppg = rawPPG;
            end
            rawPPG = rawPPG - mean(rawPPG,'omitnan') + mean(runInfo.ppg,'omitnan');
            runInfo.rawPPG    = rawPPG;
            runInfo.hasRawPPG = true;
        end

        if ismember("Time", runData.Properties.VariableNames)
            runInfo.time = runData.Time / 1000;
        elseif ismember("Timestamp", runData.Properties.VariableNames)
            runInfo.time = runData.Timestamp / 1000;
        end

        if ismember("PPGSignal", runData.Properties.VariableNames)
            runInfo.ppg = runData.PPGSignal;
        else
            if isfield(runInfo, "rawPPG")
                runInfo.ppg = runInfo.rawPPG;
            else
                runInfo.ppg = 0;
            end
        end
        runInfo.pressure = runData.Pressure;

        % AUS_PULSE_HEARD / ground truth
        if ismember('AUS_PULSE_HEARD', runData.Properties.VariableNames)
            runInfo.ausPulseHeard     = runData.AUS_PULSE_HEARD;
            runInfo.pulseHeardIndices = find(runData.AUS_PULSE_HEARD == 1);
            runInfo.hasAusPulse       = true;
            fprintf('  *** FOUND AUS_PULSE_HEARD with %d pulses ***\n', length(runInfo.pulseHeardIndices));
            if ~isempty(runInfo.pulseHeardIndices)
                firstPulseIdx               = runInfo.pulseHeardIndices(1);
                runInfo.groundTruthPressure = runInfo.pressure(firstPulseIdx);
                runInfo.groundTruthTime     = runInfo.time(firstPulseIdx);
                runInfo.hasGroundTruth      = true;
                fprintf('  *** GROUND TRUTH: %.1f mmHg at %.2f s ***\n', ...
                    runInfo.groundTruthPressure, runInfo.groundTruthTime);
            end
        else
            fprintf('  AUS_PULSE_HEARD not found\n');
        end

        % SBP reference from filename
        runInfo.hasSBPReference = false;
        runInfo.sbpReference    = NaN;
        [~, fname, ~] = fileparts(filename);
        tokens = regexp(fname, '^SBP(\d+)_', 'tokens');
        if ~isempty(tokens)
            runInfo.hasSBPReference = true;
            runInfo.sbpReference    = str2double(tokens{1}{1});
            fprintf('  *** FOUND SBP REFERENCE: %d mmHg from filename ***\n', runInfo.sbpReference);
        end

        % Person name
        runInfo.personName = '';
        targetNames = {'Harleen','Brendan','Taegon','Alex','Alexander','Kali','Nina'};
        for nameIdx = 1:length(targetNames)
            if contains(lower(fname), lower(targetNames{nameIdx}))
                runInfo.personName = targetNames{nameIdx};
                fprintf('  *** FOUND PERSON NAME: %s ***\n', runInfo.personName);
                break;
            end
        end

        % Parse SUMMARY detectors
        detectorMap    = containers.Map();
        postProcessors = {'Ensemble','OscSys','OscMAP','OscDia','EstDia','BPM'};

        for i = 1:length(summaryData)
            entry   = summaryData{i};
            detName = entry.Detector;
            if entry.Timestamp == 0 || entry.Pressure < 0
                if ~strcmp(detName, 'BPM'); continue; end
            end
            if ismember(detName, postProcessors)
                detectorMap(detName) = entry;
                continue;
            end
            if ~isKey(detectorMap, detName) || entry.Confidence > detectorMap(detName).Confidence
                detectorMap(detName) = entry;
            end
        end

        % BPM
        runInfo.hasBPM = false;
        runInfo.bpm    = NaN;
        if isKey(detectorMap, 'BPM')
            bpmEntry       = detectorMap('BPM');
            runInfo.bpm    = bpmEntry.Pressure;
            runInfo.hasBPM = true;
            fprintf('  *** FOUND BPM: %.1f ***\n', runInfo.bpm);
        end

        detectorNames      = keys(detectorMap);
        algorithmDetectors = detectorNames(~ismember(detectorNames, postProcessors));
        postDetectors      = detectorNames( ismember(detectorNames, postProcessors));

        runInfo.detectors  = algorithmDetectors;
        runInfo.post       = postDetectors;

        runInfo.postValues = struct();
        for i = 1:length(postDetectors)
            detName = postDetectors{i};
            entry   = detectorMap(detName);
            runInfo.postValues.(detName) = entry.Pressure;
        end

        fprintf('  Found %d algorithm detectors, %d post-processors\n', ...
            length(algorithmDetectors), length(postDetectors));

        runInfo.detections = containers.Map();
        for i = 1:length(algorithmDetectors)
            detectorName = algorithmDetectors{i};
            entry        = detectorMap(detectorName);

            detection            = struct();
            detection.detected   = true;
            detection.pressure   = entry.Pressure;
            detection.confidence = entry.Confidence;
            detection.timestamp  = entry.Timestamp;
            detection.time       = entry.Timestamp / 1000;
            detection.value      = entry.Pressure;

            if runInfo.hasGroundTruth
                detection.error    = detection.pressure - runInfo.groundTruthPressure;
                detection.absError = abs(detection.error);
            else
                detection.error    = NaN;
                detection.absError = NaN;
            end
            if runInfo.hasSBPReference
                detection.sbpError    = detection.pressure - runInfo.sbpReference;
                detection.sbpAbsError = abs(detection.sbpError);
            else
                detection.sbpError    = NaN;
                detection.sbpAbsError = NaN;
            end

            runInfo.detections(detectorName) = detection;
        end

        if ismember('BaselineBeat', runData.Properties.VariableNames)
            runInfo.baselineBeat    = runData.BaselineBeat;
            runInfo.hasBaselineBeat = true;
        else
            runInfo.baselineBeat    = [];
            runInfo.hasBaselineBeat = false;
        end

        allRuns{runCounter} = runInfo;
        runCounter = runCounter + 1;
    end
end

% Collect unique detectors
allDetectors = {};
for i = 1:length(allRuns)
    allDetectors = union(allDetectors, allRuns{i}.detectors);
end

fprintf('\nFound %d total run(s) across all files.\n', length(allRuns));
fprintf('Found %d unique detectors across all runs.\n', length(allDetectors));

runsWithGroundTruth = sum(cellfun(@(r) r.hasGroundTruth, allRuns));
fprintf('Found %d run(s) with ground truth data.\n\n', runsWithGroundTruth);

runsWithSBPRef = sum(cellfun(@(r) r.hasSBPReference, allRuns));
fprintf('Found %d run(s) with SBP reference in filename.\n\n', runsWithSBPRef);

runsWithBPM = sum(cellfun(@(r) isfield(r,'hasBPM') && r.hasBPM, allRuns));
fprintf('Found %d run(s) with BPM (heart rate) data.\n\n', runsWithBPM);

%% Create Main Tabbed Figure
screenSize = get(0,'ScreenSize');
figWidth   = 1600; figHeight = 900;
figLeft    = max(1, (screenSize(3) - figWidth)  / 2);
figBottom  = max(1, (screenSize(4) - figHeight) / 2);

mainFig = figure('Name','Blood Pressure Analysis', ...
    'Position',[figLeft, figBottom, figWidth, figHeight], ...
    'NumberTitle','off');

tabGroup = uitabgroup(mainFig);

% Detector Comparison
compTab = uitab(tabGroup, 'Title', 'Detector Comparison');
createComparisonTab(compTab, allRuns, allDetectors);

% Baseline (BL) Analysis
blTab = uitab(tabGroup, 'Title', 'Baseline Analysis');
createBLAnalysisTab(blTab, allRuns, allDetectors);

if runsWithGroundTruth > 0
    blTab_v2 = uitab(tabGroup, 'Title', 'Baseline Analysis Error from Ground Truth');
    createBLAnalysisTab_v2(blTab_v2, allRuns, allDetectors);
end

% Derivative (DRV) Analysis
drvTab = uitab(tabGroup, 'Title', 'Derivative Analysis');
createDRVAnalysisTab(drvTab, allRuns, allDetectors);

% Envelope (ENV) Analysis
envTab = uitab(tabGroup, 'Title', 'Envelope Analysis');
createENVAnalysisTab(envTab, allRuns, allDetectors);

% Per-person tabs
personNames = {'Harleen','Brendan','Taegon','Alex','Alexander','Kali','Nina'};
for i = 1:length(personNames)
    personName = personNames{i};
    if strcmpi(personName, 'Alex')
        personRuns  = filterRunsByName(allRuns, {'Alex','Alexander'});
        displayName = 'Alex/Alexander';
    elseif strcmpi(personName, 'Alexander')
        continue;
    else
        personRuns  = filterRunsByName(allRuns, {personName});
        displayName = personName;
    end

    if ~isempty(personRuns)
        personTab = uitab(tabGroup, 'Title', sprintf('%s - BL Analysis', displayName));
        createBLAnalysisTab(personTab, personRuns, allDetectors);

        personRunsGT = personRuns(cellfun(@(r) r.hasGroundTruth, personRuns));
        if ~isempty(personRunsGT)
            personTabErr = uitab(tabGroup, 'Title', sprintf('%s - BL Error', displayName));
            createBLAnalysisTab_v2(personTabErr, personRuns, allDetectors);
        end
        fprintf('Created analysis tabs for %s (%d runs, %d with GT)\n', ...
            displayName, length(personRuns), length(personRunsGT));
    else
        fprintf('No runs found for %s\n', personName);
    end
end

% Heart Rate Analysis
runsWithBPMList = allRuns(cellfun(@(r) isfield(r,'hasBPM') && r.hasBPM, allRuns));
if ~isempty(runsWithBPMList) && runsWithGroundTruth > 0
    hrTab = uitab(tabGroup, 'Title', 'Heart Rate Analysis');
    createHeartRateAnalysisTab(hrTab, allRuns, allDetectors);
end

% Ground Truth paginated ranking tabs
if runsWithGroundTruth > 0
    createGroundTruthTabGroup(tabGroup, allRuns, allDetectors);
end

% SBP Reference paginated ranking tabs
if runsWithSBPRef > 0
    createSBPReferenceTabGroup(tabGroup, allRuns, allDetectors);
end

% Individual run tabs
for i = 1:length(allRuns)
    [~, baseName, ~] = fileparts(allRuns{i}.filename);
    tabName = sprintf('%s-R%d', baseName, allRuns{i}.runIdx);
    tab = uitab(tabGroup, 'Title', tabName);
    createFileTab(tab, allRuns{i});
end

fprintf('\nAnalysis complete! Use the tabs to navigate between views.\n');

%% ========================================================================
%% Helper Functions
%% ========================================================================

function filteredRuns = filterRunsByName(allRuns, namePatterns)
    filteredRuns = {};
    for i = 1:length(allRuns)
        if isfield(allRuns{i},'personName') && ~isempty(allRuns{i}.personName)
            for j = 1:length(namePatterns)
                if strcmpi(allRuns{i}.personName, namePatterns{j})
                    filteredRuns{end+1} = allRuns{i};
                    break;
                end
            end
        end
    end
end

% -------------------------------------------------------------------------
function createFileTab(parentTab, runInfo)
    ax = axes('Parent', parentTab, 'Position', [0.08, 0.15, 0.78, 0.75]);

    if ~isfield(runInfo,'hasAusPulse')
        runInfo.hasAusPulse       = false;
        runInfo.pulseHeardIndices = [];
    end

    yyaxis left; hold on; grid on;
    if runInfo.hasRawPPG
        h3 = plot(runInfo.time, runInfo.rawPPG, 'Color',[1 0.6 0.6], 'LineWidth',0.8);
    end
    h2 = plot(runInfo.time, runInfo.ppg, 'r-', 'LineWidth',1.0);
    ylabel('PPG and Raw PPG','FontSize',12);
    ax.YColor = 'r';

    yyaxis right; hold on;
    h1 = plot(runInfo.time, runInfo.pressure, 'b-', 'LineWidth',1.5);
    ylabel('Pressure (mmHg)','FontSize',12);
    ax.YColor = 'b';

    detectorPressures = zeros(length(runInfo.detectors),1);
    for j = 1:length(runInfo.detectors)
        detName = runInfo.detectors{j};
        if isKey(runInfo.detections, detName)
            det = runInfo.detections(detName);
            if det.detected
                detectorPressures(j) = det.pressure;
            else
                detectorPressures(j) = inf;
            end
        else
            detectorPressures(j) = inf;
        end
    end
    [~, sortIdx]    = sort(detectorPressures);
    sortedDetectors = runInfo.detectors(sortIdx);

    colors = lines(length(sortedDetectors));
    if runInfo.hasRawPPG
        legendHandles = [h1, h2, h3];
        legendLabels  = {'Pressure','PPG (Filtered)','PPG (Raw)'};
    else
        legendHandles = [h1, h2];
        legendLabels  = {'Pressure','PPG'};
    end

    hasGroundTruth = runInfo.hasGroundTruth;

    for j = 1:length(sortedDetectors)
        detName  = sortedDetectors{j};
        safeName = strrep(detName, '_', '\_');
        if isKey(runInfo.detections, detName)
            det = runInfo.detections(detName);
            if det.detected && ~isnan(det.time)
                xline(det.time, '--', 'Color', colors(j,:), 'LineWidth', 1.5);
                h = plot(det.time, det.pressure, 'o', 'Color', colors(j,:), ...
                    'MarkerSize',10,'LineWidth',2);
                legendHandles(end+1) = h;
                % Use plain "Err=" to avoid invalid \D escape in sprintf
                if hasGroundTruth
                    legendLabels{end+1} = sprintf('%s: %.1f mmHg (Err=%.1f, C=%.2f)', ...
                        safeName, det.pressure, det.error, det.confidence);
                else
                    legendLabels{end+1} = sprintf('%s: %.1f mmHg (C=%.2f)', ...
                        safeName, det.pressure, det.confidence);
                end
            else
                h = plot(NaN, NaN, 'o', 'Color', colors(j,:), 'MarkerSize',10,'LineWidth',2);
                legendHandles(end+1) = h;
                legendLabels{end+1}  = sprintf('%s: No detection', safeName);
            end
        end
    end

    xlabelStr = 'Time (s)';
    if isfield(runInfo,'post') && iscell(runInfo.post) && ~isempty(runInfo.post) && ...
       isfield(runInfo,'postValues') && ~isempty(fieldnames(runInfo.postValues))
        parts = {};
        for k = 1:numel(runInfo.post)
            colName = runInfo.post{k};
            if isfield(runInfo.postValues, colName)
                parts{end+1} = sprintf('%s: %.1f', colName, runInfo.postValues.(colName));
            end
        end
        if ~isempty(parts)
            xlabelStr = sprintf('%s\n%s', xlabelStr, strjoin(parts,' | '));
        end
    end
    xlabel(xlabelStr,'FontSize',12,'Interpreter','none');

    titleStr = runInfo.displayName;
    if hasGroundTruth
        titleStr = sprintf('%s - Ground Truth: %.1f mmHg', titleStr, runInfo.groundTruthPressure);
    end
    title(titleStr,'FontSize',14,'Interpreter','none');

    lgd = legend(legendHandles, legendLabels, 'Location','eastoutside','FontSize',9);
    lgd.NumColumns = 1;
    lgd.ItemHitFcn = @(src,evt) toggleVisibility(evt);
end

% -------------------------------------------------------------------------
function toggleVisibility(evt)
    obj = evt.Peer;
    if strcmp(obj.Visible,'on'); obj.Visible = 'off';
    else;                        obj.Visible = 'on';
    end
end

% -------------------------------------------------------------------------
function createComparisonTab(parentTab, allRuns, allDetectors)
    numRuns      = length(allRuns);
    numDetectors = length(allDetectors);

    detectionPressures = nan(numDetectors, numRuns);
    for i = 1:numRuns
        for j = 1:numDetectors
            detName = allDetectors{j};
            if isKey(allRuns{i}.detections, detName)
                det = allRuns{i}.detections(detName);
                if det.detected
                    detectionPressures(j,i) = det.pressure;
                end
            end
        end
    end

    ax3 = axes('Parent', parentTab, 'Position', [0.08, 0.63, 0.86, 0.32]);
    ax4 = axes('Parent', parentTab, 'Position', [0.08, 0.18, 0.86, 0.32]);

    axes(ax3);
    meanPressures = mean(detectionPressures, 2, 'omitnan');
    stdPressures  = std( detectionPressures, 0, 2, 'omitnan');
    fprintf('\nMean pressures:\n');
    for j = 1:numDetectors
        fprintf('%s: %.2f +/- %.2f\n', allDetectors{j}, meanPressures(j), stdPressures(j));
    end
    errorbar(1:numDetectors, meanPressures, stdPressures, 'o-', ...
        'LineStyle','none','LineWidth',1.5,'MarkerSize',8);
    grid on;
    title('Mean Detected Pressure +/- Std Dev','FontSize',14);
    ylabel('Pressure (mmHg)','FontSize',12);
    shortLabels = shortenDetectorNames(allDetectors);
    set(gca,'XTick',1:numDetectors,'XTickLabel',shortLabels, ...
        'TickLabelInterpreter','none','XTickLabelRotation',90,'FontSize',8);
    minVal = min(meanPressures - stdPressures,[],'omitnan');
    maxVal = max(meanPressures + stdPressures,[],'omitnan');
    if ~isnan(minVal) && ~isnan(maxVal) && maxVal > minVal
        yRange = maxVal - minVal;
        ylim([max(0,minVal-0.1*yRange), maxVal+0.1*yRange]);
    end

    axes(ax4);
    successRate = sum(~isnan(detectionPressures),2) / numRuns * 100;
    bar(successRate);
    grid on;
    title('Detection Success Rate','FontSize',14);
    ylabel('Success Rate (%)','FontSize',12);
    set(gca,'XTick',1:numDetectors,'XTickLabel',shortLabels, ...
        'TickLabelInterpreter','none','XTickLabelRotation',90,'FontSize',8);
    ylim([0 110]);
end

% -------------------------------------------------------------------------
%  Core ranking engine — builds all/ENV/DRV pages for one reference type
% -------------------------------------------------------------------------
function createRankingTabGroup(tabGroup, refRuns, allDetectors, ...
        errorField, absErrorField, groupLabel, tabPrefix)

    numDetectors = length(allDetectors);
    numRuns      = length(refRuns);
    if numDetectors == 0 || numRuns == 0; return; end

    errors    = nan(numDetectors, numRuns);
    absErrors = nan(numDetectors, numRuns);
    for i = 1:numRuns
        for j = 1:numDetectors
            detName = allDetectors{j};
            if isKey(refRuns{i}.detections, detName)
                det = refRuns{i}.detections(detName);
                if det.detected && isfield(det, errorField) && ~isnan(det.(errorField))
                    errors(j,i)    = det.(errorField);
                    absErrors(j,i) = det.(absErrorField);
                end
            end
        end
    end

    meanAbsError = mean(absErrors, 2, 'omitnan');
    stdAbsError  = std( absErrors, 0, 2, 'omitnan');
    meanError    = mean(errors,    2, 'omitnan');

    [sortedMAE, sortIdx] = sort(meanAbsError, 'ascend');
    sortedDetectors      = allDetectors(sortIdx);
    sortedStd            = stdAbsError(sortIdx);

    detectorsPerPage = 75;

    % --- All detectors ---
    numPages = ceil(numDetectors / detectorsPerPage);
    fprintf('\n=== %s ANALYSIS SUMMARY ===\n', upper(groupLabel));
    fprintf('Runs: %d | Detectors: %d | Pages: %d\n\n', numRuns, numDetectors, numPages);

    for pageNum = 1:numPages
        s = (pageNum-1)*detectorsPerPage + 1;
        e = min(pageNum*detectorsPerPage, numDetectors);
        if numPages == 1
            tabName = sprintf('%s Analysis', groupLabel);
        else
            tabName = sprintf('%s Analysis (Rank %d-%d)', tabPrefix, s, e);
        end
        aTab = uitab(tabGroup, 'Title', tabName);
        buildRankingPage(aTab, sortedDetectors(s:e), sortedMAE(s:e), sortedStd(s:e), ...
            groupLabel, s, e);
    end

    % --- ENV-only ---
    envMask      = startsWith(sortedDetectors, 'ENV_');
    envDetectors = sortedDetectors(envMask);
    envMAE       = sortedMAE(envMask);
    envStd       = sortedStd(envMask);
    if ~isempty(envDetectors)
        numENVPages = ceil(length(envDetectors) / detectorsPerPage);
        for pageNum = 1:numENVPages
            s = (pageNum-1)*detectorsPerPage + 1;
            e = min(pageNum*detectorsPerPage, length(envDetectors));
            if numENVPages == 1
                tabName = sprintf('%s ENV Detectors', tabPrefix);
            else
                tabName = sprintf('%s ENV (Rank %d-%d)', tabPrefix, s, e);
            end
            aTab = uitab(tabGroup, 'Title', tabName);
            buildRankingPage(aTab, envDetectors(s:e), envMAE(s:e), envStd(s:e), ...
                [groupLabel ' — ENV only'], s, e);
        end
    end

    % --- DRV-only ---
    drvMask      = startsWith(sortedDetectors, 'DRV_');
    drvDetectors = sortedDetectors(drvMask);
    drvMAE       = sortedMAE(drvMask);
    drvStd       = sortedStd(drvMask);
    if ~isempty(drvDetectors)
        numDRVPages = ceil(length(drvDetectors) / detectorsPerPage);
        for pageNum = 1:numDRVPages
            s = (pageNum-1)*detectorsPerPage + 1;
            e = min(pageNum*detectorsPerPage, length(drvDetectors));
            if numDRVPages == 1
                tabName = sprintf('%s DRV Detectors', tabPrefix);
            else
                tabName = sprintf('%s DRV (Rank %d-%d)', tabPrefix, s, e);
            end
            aTab = uitab(tabGroup, 'Title', tabName);
            buildRankingPage(aTab, drvDetectors(s:e), drvMAE(s:e), drvStd(s:e), ...
                [groupLabel ' — DRV only'], s, e);
        end
    end

    % Console top-20
    fprintf('Top 20 Rankings by MAE:\n');
    for i = 1:min(20, numDetectors)
        idx = sortIdx(i);
        fprintf('%2d. %s: MAE=%.2f+/-%.2f mmHg, Bias=%.2f mmHg\n', ...
            i, allDetectors{idx}, meanAbsError(idx), stdAbsError(idx), meanError(idx));
    end
    fprintf('\n');
end

% -------------------------------------------------------------------------
function buildRankingPage(parentTab, pageDetectors, pageMAE, pageStd, groupLabel, startIdx, endIdx)
    numInPage   = length(pageDetectors);
    shortLabels = shortenDetectorNames(pageDetectors);

    % Top: errorbar
    ax1 = axes('Parent', parentTab, 'Position', [0.08, 0.63, 0.86, 0.32]);
    axes(ax1);
    errorbar(1:numInPage, pageMAE, pageStd, 'o-', 'LineStyle','none', ...
        'LineWidth',2,'MarkerSize',8,'Color',[0.8 0.2 0.2]);
    grid on;
    ylabel('Mean Absolute Error (mmHg)','FontSize',12);
    title(sprintf('MAE from %s +/- Std Dev (Rank %d-%d)', groupLabel, startIdx, endIdx), ...
        'FontSize',13);
    set(gca,'XTick',1:numInPage,'XTickLabel',shortLabels, ...
        'TickLabelInterpreter','none','XTickLabelRotation',90,'FontSize',8);

    % Bottom: bar + error bars
    ax2 = axes('Parent', parentTab, 'Position', [0.08, 0.18, 0.86, 0.32]);
    axes(ax2);
    bar(pageMAE, 'FaceColor',[0.3 0.5 0.8]);
    hold on;
    errorbar(1:numInPage, pageMAE, pageStd, 'k.','LineWidth',1.5,'CapSize',8);
    hold off;
    grid on;
    xlabel('Detector (Ranked by MAE)','FontSize',12);
    ylabel('Mean Absolute Error (mmHg)','FontSize',12);
    title(sprintf('Detector Ranking — %s (Rank %d-%d)', groupLabel, startIdx, endIdx), ...
        'FontSize',13);
    set(gca,'XTick',1:numInPage,'XTickLabel',shortLabels, ...
        'TickLabelInterpreter','none','XTickLabelRotation',90,'FontSize',8);
end

% -------------------------------------------------------------------------
function createGroundTruthTabGroup(tabGroup, allRuns, allDetectors)
    runsWithGT = allRuns(cellfun(@(r) r.hasGroundTruth, allRuns));
    if isempty(runsWithGT); return; end
    createRankingTabGroup(tabGroup, runsWithGT, allDetectors, ...
        'error', 'absError', 'Ground Truth', 'GT');
end

function createSBPReferenceTabGroup(tabGroup, allRuns, allDetectors)
    runsWithSBP = allRuns(cellfun(@(r) r.hasSBPReference, allRuns));
    if isempty(runsWithSBP); return; end
    createRankingTabGroup(tabGroup, runsWithSBP, allDetectors, ...
        'sbpError', 'sbpAbsError', 'SBP Reference', 'SBP');
end

% -------------------------------------------------------------------------
function createBLAnalysisTab(parentTab, allRuns, allDetectors)
    blDetectors = allDetectors(startsWith(allDetectors,'BL_'));
    if isempty(blDetectors)
        annotation(parentTab,'textbox',[0.3,0.4,0.4,0.2], ...
            'String','No BL detectors found.','FontSize',14, ...
            'HorizontalAlignment','center','EdgeColor','none');
        return;
    end

    numBL  = length(blDetectors);
    params = zeros(numBL,3);
    valid  = true(numBL,1);
    for i = 1:numBL
        tok = regexp(blDetectors{i},'BL_W(\d+)_T(\d+)\.(\d+)_D(\d+)','tokens');
        if ~isempty(tok) && ~isempty(tok{1})
            W = str2double(tok{1}{1});
            T = str2double(tok{1}{2}) + str2double(tok{1}{3})/10;
            D = str2double(tok{1}{4});
            params(i,:) = [W,T,D];
            fprintf('Parsed %s -> W=%d, T=%.1f, D=%d\n', blDetectors{i}, W, T, D);
        else
            valid(i) = false;
            fprintf('Warning: Could not parse detector: %s\n', blDetectors{i});
        end
    end
    blDetectors = blDetectors(valid); params = params(valid,:); numBL = length(blDetectors);
    if numBL == 0
        annotation(parentTab,'textbox',[0.3,0.4,0.4,0.2], ...
            'String','No BL detectors with valid format found.','FontSize',14, ...
            'HorizontalAlignment','center','EdgeColor','none');
        return;
    end
    fprintf('\nSuccessfully parsed %d BL detectors\n', numBL);

    numRuns      = length(allRuns);
    detPressures = nan(numBL, numRuns);
    for i = 1:numRuns
        for j = 1:numBL
            detName = blDetectors{j};
            if isKey(allRuns{i}.detections, detName)
                det = allRuns{i}.detections(detName);
                if det.detected; detPressures(j,i) = det.pressure; end
            end
        end
    end

    ax1 = axes('Parent',parentTab,'Position',[0.08,0.63,0.38,0.32]);
    ax2 = axes('Parent',parentTab,'Position',[0.56,0.63,0.38,0.32]);
    ax3 = axes('Parent',parentTab,'Position',[0.08,0.18,0.38,0.32]);
    plotParamEffect(ax1, params(:,1), detPressures, 'Window Size (W)',       'Effect of Window Size');
    plotParamEffect(ax2, params(:,2), detPressures, 'Threshold Multiplier (T)', 'Effect of Threshold');
    plotParamEffect(ax3, params(:,3), detPressures, 'Minimum Deviation (D)', 'Effect of Min Deviation');
end

% -------------------------------------------------------------------------
function createDRVAnalysisTab(parentTab, allRuns, allDetectors)
    drvDetectors = allDetectors(startsWith(allDetectors,'DRV_'));
    if isempty(drvDetectors)
        annotation(parentTab,'textbox',[0.3,0.4,0.4,0.2], ...
            'String','No DRV detectors found.','FontSize',14, ...
            'HorizontalAlignment','center','EdgeColor','none');
        return;
    end

    numDRV = length(drvDetectors);
    params = zeros(numDRV,2);
    for i = 1:numDRV
        tok = regexp(drvDetectors{i},'DRV_W(\d+)_T([\d.]+)','tokens');
        if ~isempty(tok)
            params(i,:) = [str2double(tok{1}{1}), str2double(tok{1}{2})];
        end
    end

    numRuns      = length(allRuns);
    detPressures = nan(numDRV, numRuns);
    for i = 1:numRuns
        for j = 1:numDRV
            detName = drvDetectors{j};
            if isKey(allRuns{i}.detections, detName)
                det = allRuns{i}.detections(detName);
                if det.detected; detPressures(j,i) = det.pressure; end
            end
        end
    end

    ax1 = axes('Parent',parentTab,'Position',[0.1,0.3,0.35,0.5]);
    ax2 = axes('Parent',parentTab,'Position',[0.55,0.3,0.35,0.5]);
    plotParamEffect(ax1, params(:,1), detPressures, 'Window Size (W)', 'Effect of Window Size');
    plotParamEffect(ax2, params(:,2), detPressures, 'Threshold (T)',    'Effect of Threshold');
end

% -------------------------------------------------------------------------
function createENVAnalysisTab(parentTab, allRuns, allDetectors)
    envDetectors = allDetectors(startsWith(allDetectors,'ENV_'));
    if isempty(envDetectors)
        annotation(parentTab,'textbox',[0.3,0.4,0.4,0.2], ...
            'String','No ENV detectors found.','FontSize',14, ...
            'HorizontalAlignment','center','EdgeColor','none');
        return;
    end

    numENV = length(envDetectors);
    params = nan(numENV, 2);   % [W, T]
    valid  = true(numENV, 1);
    for i = 1:numENV
        tok = regexp(envDetectors{i},'ENV_W(\d+)_T([\d.]+)','tokens');
        if ~isempty(tok)
            params(i,:) = [str2double(tok{1}{1}), str2double(tok{1}{2})];
        else
            tok2 = regexp(envDetectors{i},'ENV_W(\d+)','tokens');
            if ~isempty(tok2)
                params(i,1) = str2double(tok2{1}{1});
            else
                valid(i) = false;
                fprintf('Warning: Could not parse ENV detector: %s\n', envDetectors{i});
            end
        end
    end
    envDetectors = envDetectors(valid); params = params(valid,:); numENV = length(envDetectors);

    if numENV == 0
        annotation(parentTab,'textbox',[0.3,0.4,0.4,0.2], ...
            'String','No ENV detectors with valid format found.','FontSize',14, ...
            'HorizontalAlignment','center','EdgeColor','none');
        return;
    end

    numRuns      = length(allRuns);
    detPressures = nan(numENV, numRuns);
    for i = 1:numRuns
        for j = 1:numENV
            detName = envDetectors{j};
            if isKey(allRuns{i}.detections, detName)
                det = allRuns{i}.detections(detName);
                if det.detected; detPressures(j,i) = det.pressure; end
            end
        end
    end

    hasT = any(~isnan(params(:,2)));
    if hasT
        ax1 = axes('Parent',parentTab,'Position',[0.1,0.3,0.35,0.5]);
        ax2 = axes('Parent',parentTab,'Position',[0.55,0.3,0.35,0.5]);
        plotParamEffect(ax1, params(:,1), detPressures, 'Window Size (W)', 'Effect of Window Size');
        plotParamEffect(ax2, params(:,2), detPressures, 'Threshold (T)',    'Effect of Threshold');
    else
        ax1 = axes('Parent',parentTab,'Position',[0.15,0.3,0.7,0.5]);
        plotParamEffect(ax1, params(:,1), detPressures, 'Window Size (W)', 'Effect of Window Size');
    end
end

% -------------------------------------------------------------------------
function plotParamEffect(ax, paramVec, detPressures, xlabelStr, titleStr)
    axes(ax);
    uniqueVals = unique(paramVec(~isnan(paramVec)));
    meanVals   = zeros(length(uniqueVals),1);
    stdVals    = zeros(length(uniqueVals),1);
    for i = 1:length(uniqueVals)
        idx         = paramVec == uniqueVals(i);
        meanVals(i) = mean(detPressures(idx,:),'all','omitnan');
        stdVals(i)  = std( detPressures(idx,:),0,  'all','omitnan');
    end
    errorbar(uniqueVals, meanVals, stdVals, 'o-','LineStyle','none','LineWidth',2,'MarkerSize',8);
    grid on;
    xlabel(xlabelStr,'FontSize',12);
    ylabel('Mean Detected Pressure (mmHg)','FontSize',12);
    title(titleStr,'FontSize',14);
end

% -------------------------------------------------------------------------
function createBLAnalysisTab_v2(parentTab, allRuns, allDetectors)
    blDetectors = allDetectors(startsWith(allDetectors,'BL_'));
    if isempty(blDetectors)
        annotation(parentTab,'textbox',[0.3,0.4,0.4,0.2], ...
            'String','No BL detectors found.','FontSize',14, ...
            'HorizontalAlignment','center','EdgeColor','none');
        return;
    end

    runsWithGT = allRuns(cellfun(@(r) r.hasGroundTruth, allRuns));
    if isempty(runsWithGT)
        annotation(parentTab,'textbox',[0.3,0.4,0.4,0.2], ...
            'String','No runs with ground truth found.','FontSize',14, ...
            'HorizontalAlignment','center','EdgeColor','none');
        return;
    end
    numRunsGT = length(runsWithGT);

    numBL  = length(blDetectors);
    params = zeros(numBL,3);
    valid  = true(numBL,1);
    for i = 1:numBL
        tok = regexp(blDetectors{i},'BL_W(\d+)_T(\d+)\.(\d+)_D(\d+)','tokens');
        if ~isempty(tok) && ~isempty(tok{1})
            W = str2double(tok{1}{1});
            T = str2double(tok{1}{2}) + str2double(tok{1}{3})/10;
            D = str2double(tok{1}{4});
            params(i,:) = [W,T,D];
        else
            valid(i) = false;
        end
    end
    blDetectors = blDetectors(valid); params = params(valid,:); numBL = length(blDetectors);
    if numBL == 0
        annotation(parentTab,'textbox',[0.3,0.4,0.4,0.2], ...
            'String','No BL detectors with valid format found.','FontSize',14, ...
            'HorizontalAlignment','center','EdgeColor','none');
        return;
    end
    fprintf('\nSuccessfully parsed %d BL detectors for error analysis\n', numBL);

    errors = nan(numBL, numRunsGT);
    for i = 1:numRunsGT
        for j = 1:numBL
            detName = blDetectors{j};
            if isKey(runsWithGT{i}.detections, detName)
                det = runsWithGT{i}.detections(detName);
                if det.detected; errors(j,i) = det.error; end
            end
        end
    end

    ax1 = axes('Parent',parentTab,'Position',[0.08,0.68,0.86,0.26]);
    ax2 = axes('Parent',parentTab,'Position',[0.08,0.38,0.86,0.26]);
    ax3 = axes('Parent',parentTab,'Position',[0.08,0.08,0.86,0.26]);
    plotErrorEffect(ax1, params(:,1), errors, 'Window Size (W)',         'Window Size: Mean Error +/- SD');
    plotErrorEffect(ax2, params(:,2), errors, 'Threshold Multiplier (T)','Threshold: Mean Error +/- SD');
    plotErrorEffect(ax3, params(:,3), errors, 'Minimum Deviation (D)',   'Min Deviation: Mean Error +/- SD');

    fprintf('\n=== BL PARAMETER ERROR ANALYSIS ===\n');
    fprintf('Analyzed %d BL detectors across %d runs with ground truth\n', numBL, numRunsGT);
end

% -------------------------------------------------------------------------
function plotErrorEffect(ax, paramVec, errors, xlabelStr, titleStr)
    axes(ax);
    uniqueVals = sort(unique(paramVec));
    meanVals   = zeros(length(uniqueVals),1);
    stdVals    = zeros(length(uniqueVals),1);
    for i = 1:length(uniqueVals)
        idx         = paramVec == uniqueVals(i);
        meanVals(i) = mean(errors(idx,:),'all','omitnan');
        stdVals(i)  = std( errors(idx,:),0,  'all','omitnan');
    end
    errorbar(uniqueVals, meanVals, stdVals, 'o-','LineWidth',2,'MarkerSize',8);
    grid on;
    xlabel(xlabelStr,'FontSize',11);
    ylabel('Mean Error (mmHg)','FontSize',11);
    title(titleStr,'FontSize',12);
    yline(0,'k--','LineWidth',1);
end

% -------------------------------------------------------------------------
function createHeartRateAnalysisTab(parentTab, allRuns, allDetectors)
    blDetectors = allDetectors(startsWith(allDetectors,'BL_'));
    if isempty(blDetectors)
        annotation(parentTab,'textbox',[0.3,0.4,0.4,0.2], ...
            'String','No BL detectors found.','FontSize',14, ...
            'HorizontalAlignment','center','EdgeColor','none');
        return;
    end

    runsWithBoth = allRuns(cellfun(@(r) r.hasGroundTruth && isfield(r,'hasBPM') && r.hasBPM, allRuns));
    if isempty(runsWithBoth)
        annotation(parentTab,'textbox',[0.3,0.4,0.4,0.2], ...
            'String','No runs with both ground truth and BPM data.','FontSize',14, ...
            'HorizontalAlignment','center','EdgeColor','none');
        return;
    end

    numBL  = length(blDetectors);
    params = zeros(numBL,3);
    valid  = true(numBL,1);
    for i = 1:numBL
        tok = regexp(blDetectors{i},'BL_W(\d+)_T(\d+)\.(\d+)_D(\d+)','tokens');
        if ~isempty(tok) && ~isempty(tok{1})
            W = str2double(tok{1}{1});
            T = str2double(tok{1}{2}) + str2double(tok{1}{3})/10;
            D = str2double(tok{1}{4});
            params(i,:) = [W,T,D];
        else
            valid(i) = false;
        end
    end
    blDetectors = blDetectors(valid); params = params(valid,:); numBL = length(blDetectors);
    if numBL == 0; return; end

    numRuns = length(runsWithBoth);
    errors  = nan(numBL, numRuns);
    bpms    = zeros(1, numRuns);
    for i = 1:numRuns
        bpms(i) = runsWithBoth{i}.bpm;
        for j = 1:numBL
            detName = blDetectors{j};
            if isKey(runsWithBoth{i}.detections, detName)
                det = runsWithBoth{i}.detections(detName);
                if det.detected; errors(j,i) = det.error; end
            end
        end
    end

    minBPM    = floor(min(bpms)/5)*5;
    maxBPM    = ceil(max(bpms)/5)*5;
    bpmBins   = minBPM:5:maxBPM;
    bpmLabels = arrayfun(@(a,b) sprintf('%d-%d',a,b-1), bpmBins(1:end-1), bpmBins(2:end), ...
        'UniformOutput',false);
    bpmBinIdx = discretize(bpms, bpmBins);

    ax1 = axes('Parent',parentTab,'Position',[0.08,0.68,0.86,0.26]);
    ax2 = axes('Parent',parentTab,'Position',[0.08,0.38,0.86,0.26]);
    ax3 = axes('Parent',parentTab,'Position',[0.08,0.08,0.86,0.26]);

    plotHRHeatmap(ax1, params(:,1), sort(unique(params(:,1))), errors, bpmBinIdx, bpmLabels, ...
        'Window Size (W)',    'MAE: Window Size vs Heart Rate');
    plotHRHeatmap(ax2, params(:,2), sort(unique(params(:,2))), errors, bpmBinIdx, bpmLabels, ...
        'Threshold (T)',      'MAE: Threshold vs Heart Rate');
    plotHRHeatmap(ax3, params(:,3), sort(unique(params(:,3))), errors, bpmBinIdx, bpmLabels, ...
        'Min Deviation (D)',  'MAE: Min Deviation vs Heart Rate');

    fprintf('\n=== HEART RATE ANALYSIS ===\n');
    fprintf('Runs with BPM+GT: %d | BPM range: %.0f-%.0f\n', numRuns, min(bpms), max(bpms));
end

% -------------------------------------------------------------------------
function plotHRHeatmap(ax, paramVec, uniqueVals, errors, bpmBinIdx, bpmLabels, ylabelStr, titleStr)
    axes(ax);
    nRows = length(uniqueVals);
    nCols = length(bpmLabels);
    hmap  = nan(nRows, nCols);
    for i = 1:nRows
        detIdx = find(paramVec == uniqueVals(i));
        for j = 1:nCols
            runIdx = find(bpmBinIdx == j);
            if ~isempty(runIdx) && ~isempty(detIdx)
                sub = errors(detIdx, runIdx);
                hmap(i,j) = mean(abs(sub(:)),'omitnan');
            end
        end
    end
    imagesc(hmap); colormap(ax, createErrorColormap()); caxis([0 20]); colorbar;
    set(gca,'XTick',1:nCols,'XTickLabel',bpmLabels,'XTickLabelRotation',45);
    set(gca,'YTick',1:nRows,'YTickLabel',uniqueVals);
    xlabel('Heart Rate (BPM)','FontSize',11);
    ylabel(ylabelStr,'FontSize',11);
    title(titleStr,'FontSize',12);
    for i = 1:nRows
        for j = 1:nCols
            if ~isnan(hmap(i,j))
                text(j,i,sprintf('%.1f',hmap(i,j)),'HorizontalAlignment','center', ...
                    'FontSize',8,'Color',getTextColor(hmap(i,j)));
            end
        end
    end
end

% -------------------------------------------------------------------------
function shortLabels = shortenDetectorNames(detectors)
    shortLabels = cell(size(detectors));
    for i = 1:length(detectors)
        label = detectors{i};
        label = strrep(label,'BL_','BL');
        label = strrep(label,'DRV_','DRV');
        label = strrep(label,'ENV_','ENV');
        label = strrep(label,'_',',');
        shortLabels{i} = label;
    end
end

function cmap = createErrorColormap()
    n = 256; cmap = zeros(n,3);
    for i = 1:n
        err = (i-1)*20/(n-1);
        if err < 5;      cmap(i,:) = [0, 0.8, 0];
        elseif err < 10; t = (err-5)/5;   cmap(i,:) = [t, 0.8, 0];
        elseif err < 15; t = (err-10)/5;  cmap(i,:) = [1, 0.8*(1-t)+0.5*t, 0];
        else;            t = min((err-15)/5,1); cmap(i,:) = [1, 0.5*(1-t), 0];
        end
    end
end

function color = getTextColor(err)
    if isnan(err) || err < 10; color = 'k'; else; color = 'w'; end
end