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
            runInfo.pulseHeardIndices = find(runData.AUS_PULSE_HEARD == 0);
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
        runInfo.hasOmron        = false;
        runInfo.omronSystolic   = NaN;
        runInfo.omronDiastolic  = NaN;
        [~, fname, ~] = fileparts(filename);
        tokens = regexp(fname, '(?i)SBP[_\s-]*([0-9]{2,3})', 'tokens');
        if ~isempty(tokens)
            runInfo.hasSBPReference = true;
            runInfo.sbpReference    = str2double(tokens{1}{1});
            fprintf('  *** FOUND SBP REFERENCE: %d mmHg from filename ***\n', runInfo.sbpReference);
        end

        % --- Compute Max Ground Truth if both GT and SBP exist ---
        runInfo.hasMaxGT = false;
        runInfo.maxGTPressure = NaN;
        if runInfo.hasGroundTruth && runInfo.hasSBPReference
            runInfo.hasMaxGT = true;
            runInfo.maxGTPressure = max(runInfo.groundTruthPressure, runInfo.sbpReference);
            fprintf('  *** MAX GROUND TRUTH: %.1f mmHg (max of GT=%.1f, SBP=%.1f) ***\n', ...
                runInfo.maxGTPressure, runInfo.groundTruthPressure, runInfo.sbpReference);
        end

        omronTok = regexp(fname, '_(\d{2,3})_(\d{2,3})[^0-9]*$', 'tokens');
        if ~isempty(omronTok)
            runInfo.omronSystolic  = str2double(omronTok{1}{1});
            runInfo.omronDiastolic = str2double(omronTok{1}{2});
            runInfo.hasOmron       = true;
            fprintf('  *** FOUND OMRON: %d/%d mmHg from filename ***\n', ...
                runInfo.omronSystolic, runInfo.omronDiastolic);
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
        postProcessors = {'Ensemble','OscSys','OscMAP','OscDia','EstDia','BPM', 'BestConf'};

        for i = 1:length(summaryData)
            entry   = summaryData{i};
            detName = entry.Detector;
            if entry.Timestamp == 0 || entry.Pressure < 0
                if ~strcmp(detName, 'BPM') && ~ismember(detName, postProcessors)
                    continue;
                end
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

            % Inside detection struct creation
            if runInfo.hasMaxGT
                detection.maxError    = detection.pressure - runInfo.maxGTPressure;
                detection.maxAbsError = abs(detection.maxError);
            else
                detection.maxError    = NaN;
                detection.maxAbsError = NaN;
            end

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
if runsWithSBPRef > 0
    blTab_sbp = uitab(tabGroup, 'Title', 'Baseline Analysis Error from SBP Reference');
    createBLAnalysisTab_v2(blTab_sbp, allRuns, allDetectors, 'sbp');
end

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
        personRunsSBP = personRuns(cellfun(@(r) r.hasSBPReference, personRuns));
        if ~isempty(personRunsSBP)
            personTabErrSBP = uitab(tabGroup, 'Title', sprintf('%s - BL Error (SBP)', displayName));
            createBLAnalysisTab_v2(personTabErrSBP, personRuns, allDetectors, 'sbp');
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

% Dot graph: SBP reference vs. Auscultatory GT
if runsWithGroundTruth > 0 && runsWithSBPRef > 0
    dotTab = uitab(tabGroup, 'Title', 'SBP Reference vs. AUS GT');
    createSBPvsAUSComparisonTab(dotTab, allRuns);
end

% Max Ground Truth (max of auscultatory GT and SBP reference)
if runsWithGroundTruth > 0 && runsWithSBPRef > 0
    maxGTRuns = {};
    for i = 1:length(allRuns)
        if allRuns{i}.hasMaxGT
            maxGTRuns{end+1} = allRuns{i};
        end
    end
    if ~isempty(maxGTRuns)
        createMaxGroundTruthTabGroup(tabGroup, maxGTRuns, allDetectors);
    end
end

% Combined overview across all available references
if runsWithGroundTruth > 0 || runsWithSBPRef > 0
    createAllGroundTruthsSummaryTab(tabGroup, allRuns, allDetectors);
    createErrorDistributionTab(tabGroup, allRuns, allDetectors);
end

% Subject-level run analysis tabs
createSubjectRunTabs(tabGroup, allRuns);

% Individual run tabs
for i = 1:length(allRuns)
    [~, baseName, ~] = fileparts(allRuns{i}.filename);
    tabName = sprintf('%s-R%d', baseName, allRuns{i}.runIdx);
    tab = uitab(tabGroup, 'Title', tabName);
    createFileTab(tab, allRuns{i});

    % *** ADD THIS BLOCK ***
    confTabName = sprintf('%s-R%d [Conf]', baseName, allRuns{i}.runIdx);
    confTab = uitab(tabGroup, 'Title', confTabName);
    createConfidenceHeatmapTab(confTab, allRuns{i}, allDetectors);  % <-- add allDetectors
    % *** END ADD ***
end

exportResultsCSV(allRuns);

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
function pct = computePct5(absErrVec)
% Returns % of valid (non-NaN) absolute errors <= 5 mmHg.
    valid = absErrVec(~isnan(absErrVec));
    if isempty(valid)
        pct = NaN;
    else
        pct = mean(valid <= 5) * 100;
    end
end

% -------------------------------------------------------------------------
function createFileTab(parentTab, runInfo)
    if ~isfield(runInfo,'hasRawPPG'); runInfo.hasRawPPG = false; end
    if ~isfield(runInfo,'hasAusPulse'); runInfo.hasAusPulse = false; end
    if ~isfield(runInfo,'pulseHeardIndices'); runInfo.pulseHeardIndices = []; end
    if ~isfield(runInfo,'detectors'); runInfo.detectors = {}; end
    if ~isfield(runInfo,'detections'); runInfo.detections = containers.Map; end
    if ~isfield(runInfo,'time'); runInfo.time = []; end
    if ~isfield(runInfo,'ppg'); runInfo.ppg = []; end
    if ~isfield(runInfo,'rawPPG'); runInfo.rawPPG = []; end
    if ~isfield(runInfo,'pressure'); runInfo.pressure = []; end

    ax = axes('Parent', parentTab);
    hold(ax,'on'); grid(ax,'on');

    if runInfo.hasRawPPG && ~isempty(runInfo.rawPPG)
        hRaw = plot(ax, runInfo.time, runInfo.rawPPG, 'Color',[1 0.6 0.6],'LineWidth',0.8);
    end
    hPPG = plot(ax, runInfo.time, runInfo.ppg, 'r-','LineWidth',1.0);

    yyaxis(ax,'right');
    hPres = plot(ax, runInfo.time, runInfo.pressure,'b-','LineWidth',1.5);
    ylabel(ax,'Pressure (mmHg)');

    if runInfo.hasAusPulse && ~isempty(runInfo.pulseHeardIndices)
        ausT = runInfo.time(runInfo.pulseHeardIndices);
        ausP = runInfo.pressure(runInfo.pulseHeardIndices);
        plot(ax, ausT, ausP,'bx','MarkerSize',10,'LineWidth',2.5);
    end

    xlabel(ax,'Time (s)');
    yyaxis(ax,'left'); ylabel(ax,'PPG');
    if isfield(runInfo,'displayName')
        title(ax, runInfo.displayName,'Interpreter','none');
    end

    legendHandles = [hPres, hPPG];
    legendEntries  = {'Pressure','PPG'};

    if exist('hRaw','var')
        legendHandles(end+1) = hRaw;
        legendEntries{end+1} = 'PPG (Raw)';
    end

    if runInfo.hasAusPulse && ~isempty(runInfo.pulseHeardIndices)
        hAus = findobj(ax, 'Marker','x', 'Color',[0 0 1]);
        if ~isempty(hAus)
            legendHandles(end+1) = hAus(1);
        else
            legendHandles(end+1) = plot(ax, NaN,NaN,'bx','MarkerSize',10,'LineWidth',2.5);
        end
        legendEntries{end+1} = 'AUS Pulse Heard';
    end

    colors = lines(length(runInfo.detectors));
    allXlines = findobj(ax, 'Type','ConstantLine');

    detProxyHandles = gobjects(0);
    detProxyLabels  = {};
    detHandlesMap   = struct();

    for j = 1:length(runInfo.detectors)
        detName = runInfo.detectors{j};
        if ~isKey(runInfo.detections, detName); continue; end
        det = runInfo.detections(detName);
        if ~det.detected || isnan(det.time); continue; end

        matchH = gobjects(0);
        for k = 1:length(allXlines)
            if abs(allXlines(k).Value - det.time) < 1e-6
                matchH = allXlines(k);
                break;
            end
        end

        hProxy = plot(ax, NaN, NaN, '--', 'Color', colors(j,:), 'LineWidth', 1.5);

        detProxyHandles(end+1) = hProxy;
        detProxyLabels{end+1}  = detName;

        safeName = matlab.lang.makeValidName(detName);
        if ~isempty(matchH)
            detHandlesMap.(safeName) = [matchH, hProxy];
        else
            detHandlesMap.(safeName) = hProxy;
        end
    end

    legendHandles = [legendHandles, detProxyHandles];
    legendEntries  = [legendEntries,  detProxyLabels];

    leg = legend(ax, legendHandles, legendEntries, ...
        'Location','eastoutside','FontSize',8,'Interpreter','none');

    setappdata(leg, 'detHandlesMap', detHandlesMap);
    leg.ItemHitFcn = @(src, evt) toggleLegendItem(src, evt);

    legend(ax, legendHandles, legendEntries, 'Location','eastoutside','FontSize',8);

    infoStr = '';
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
            infoStr = strjoin(parts,' | ');
        end
    end
    if runInfo.hasAusPulse && ~isempty(runInfo.pulseHeardIndices)
        if ~isempty(infoStr)
            infoStr = [infoStr ' | '];
        end
        infoStr = [infoStr 'AUS pulses detected'];
    end

    xLim = get(ax,'XLim');
    yLim = get(ax,'YLim');
    text(ax, xLim(1), yLim(1)-0.05*diff(yLim), infoStr, ...
        'VerticalAlignment','top','HorizontalAlignment','left','FontSize',9, ...
        'Interpreter','none');

    attachRunGraphInspector(ax, runInfo);

    text(ax, xLim(1) + 0.01*diff(xLim), yLim(2) - 0.04*diff(yLim), ...
        'Click graph to inspect Time / Pressure / PPG values', ...
        'VerticalAlignment','top','HorizontalAlignment','left','FontSize',8, ...
        'BackgroundColor',[1 1 1 0.6], 'Margin', 2, 'Interpreter','none');

    zoom(ax,'on'); pan(ax,'on');
end

% -------------------------------------------------------------------------
function attachRunGraphInspector(ax, runInfo)
    if ~isfield(runInfo,'time') || isempty(runInfo.time)
        return;
    end
    set(ax, 'ButtonDownFcn', @(src,evt)handleRunGraphClick(ax, runInfo));
    children = findobj(ax, '-property', 'ButtonDownFcn');
    for i = 1:length(children)
        try
            set(children(i), 'ButtonDownFcn', @(src,evt)handleRunGraphClick(ax, runInfo));
        catch
        end
    end
end

% -------------------------------------------------------------------------
function handleRunGraphClick(ax, runInfo)
    if ~isvalid(ax) || ~isfield(runInfo,'time') || isempty(runInfo.time)
        return;
    end
    cp = get(ax, 'CurrentPoint');
    clickTime = cp(1,1);
    [~, idx] = min(abs(runInfo.time - clickTime));
    idx = max(1, min(idx, length(runInfo.time)));
    tVal     = safeValueAt(runInfo.time, idx);
    pVal     = safeValueAt(runInfo.pressure, idx);
    ppgVal   = safeValueAt(runInfo.ppg, idx);
    rawPPGVal = NaN;
    if isfield(runInfo,'rawPPG') && ~isempty(runInfo.rawPPG)
        rawPPGVal = safeValueAt(runInfo.rawPPG, idx);
    end
    oldLine = getappdata(ax, 'runProbeLine');
    if ~isempty(oldLine) && isgraphics(oldLine)
        delete(oldLine);
    end
    oldText = getappdata(ax, 'runProbeText');
    if ~isempty(oldText) && isgraphics(oldText)
        delete(oldText);
    end
    yl = ylim(ax);
    probeLine = line(ax, [tVal tVal], yl, 'Color',[0 0 0], 'LineStyle',':', 'LineWidth',1.2);
    if isnan(rawPPGVal)
        msg = sprintf('t = %.3f s | Pressure = %.1f mmHg | PPG = %.3f', tVal, pVal, ppgVal);
    else
        msg = sprintf('t = %.3f s | Pressure = %.1f mmHg | PPG = %.3f | Raw PPG = %.3f', ...
            tVal, pVal, ppgVal, rawPPGVal);
    end
    xLim = xlim(ax);
    yLim = ylim(ax);
    textX = xLim(1) + 0.01 * diff(xLim);
    textY = yLim(2) - 0.09 * diff(yLim);
    probeText = text(ax, textX, textY, msg, ...
        'VerticalAlignment', 'top', ...
        'HorizontalAlignment', 'left', ...
        'FontSize', 9, ...
        'BackgroundColor', [1 1 1 0.75], ...
        'Margin', 3, ...
        'Interpreter', 'none');
    setappdata(ax, 'runProbeLine', probeLine);
    setappdata(ax, 'runProbeText', probeText);
end

% -------------------------------------------------------------------------
function value = safeValueAt(vec, idx)
    if isempty(vec)
        value = NaN;
        return;
    end
    idx = max(1, min(idx, numel(vec)));
    value = vec(idx);
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

    % Compute % within 5 mmHg per detector
    pct5 = nan(numDetectors, 1);
    for j = 1:numDetectors
        pct5(j) = computePct5(absErrors(j,:));
    end

    [sortedMAE, sortIdx] = sort(meanAbsError, 'ascend');
    sortedDetectors      = allDetectors(sortIdx);
    sortedStd            = stdAbsError(sortIdx);
    sortedPct5           = pct5(sortIdx);

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
            sortedPct5(s:e), groupLabel, s, e);
    end

    % --- ENV-only ---
    envMask      = startsWith(sortedDetectors, 'ENV_');
    envDetectors = sortedDetectors(envMask);
    envMAE       = sortedMAE(envMask);
    envStd       = sortedStd(envMask);
    envPct5      = sortedPct5(envMask);
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
                envPct5(s:e), [groupLabel ' — ENV only'], s, e);
        end
    end

    % --- DRV-only ---
    drvMask      = startsWith(sortedDetectors, 'DRV_');
    drvDetectors = sortedDetectors(drvMask);
    drvMAE       = sortedMAE(drvMask);
    drvStd       = sortedStd(drvMask);
    drvPct5      = sortedPct5(drvMask);
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
                drvPct5(s:e), [groupLabel ' — DRV only'], s, e);
        end
    end

    % Console top-20
    fprintf('Top 20 Rankings by MAE:\n');
    for i = 1:min(20, numDetectors)
        idx = sortIdx(i);
        fprintf('%2d. %s: MAE=%.2f+/-%.2f mmHg, Bias=%.2f mmHg, Within5mmHg=%.1f%%\n', ...
            i, allDetectors{idx}, meanAbsError(idx), stdAbsError(idx), meanError(idx), pct5(idx));
    end
    fprintf('\n');
end

% -------------------------------------------------------------------------
function buildRankingPage(parentTab, pageDetectors, pageMAE, pageStd, pagePct5, groupLabel, startIdx, endIdx)
    numInPage   = length(pageDetectors);
    shortLabels = shortenDetectorNames(pageDetectors);

    % Three panels: MAE errorbar | MAE bar | % within 5 mmHg bar
    ax1 = axes('Parent', parentTab, 'Position', [0.08, 0.70, 0.86, 0.25]);
    axes(ax1);
    errorbar(1:numInPage, pageMAE, pageStd, 'o-', 'LineStyle','none', ...
        'LineWidth',2,'MarkerSize',8,'Color',[0.8 0.2 0.2]);
    grid on;
    ylabel('MAE (mmHg)','FontSize',11);
    title(sprintf('MAE from %s +/- Std Dev (Rank %d-%d)', groupLabel, startIdx, endIdx), ...
        'FontSize',12);
    set(gca,'XTick',1:numInPage,'XTickLabel',shortLabels, ...
        'TickLabelInterpreter','none','XTickLabelRotation',90,'FontSize',8);

    ax2 = axes('Parent', parentTab, 'Position', [0.08, 0.39, 0.86, 0.25]);
    axes(ax2);
    bar(pageMAE, 'FaceColor',[0.3 0.5 0.8]);
    hold on;
    errorbar(1:numInPage, pageMAE, pageStd, 'k.','LineWidth',1.5,'CapSize',8);
    hold off;
    grid on;
    xlabel('Detector (Ranked by MAE)','FontSize',11);
    ylabel('MAE (mmHg)','FontSize',11);
    title(sprintf('Detector Ranking — %s (Rank %d-%d)', groupLabel, startIdx, endIdx), ...
        'FontSize',12);
    set(gca,'XTick',1:numInPage,'XTickLabel',shortLabels, ...
        'TickLabelInterpreter','none','XTickLabelRotation',90,'FontSize',8);

    % NEW: % within 5 mmHg panel
    ax3 = axes('Parent', parentTab, 'Position', [0.08, 0.07, 0.86, 0.25]);
    axes(ax3);
    validPct5 = pagePct5;
    validPct5(isnan(validPct5)) = 0;
    bh = bar(validPct5, 'FaceColor',[0.2 0.7 0.35]);
    hold on;
    % Colour bars with fewer than 5 data points grey to flag low-n estimates
    for k = 1:numInPage
        if isnan(pagePct5(k))
            bh.FaceColor = 'flat';
            bh.CData(k,:) = [0.7 0.7 0.7];
        end
    end
    yline(50, 'k:', 'LineWidth', 1);   % 50% reference
    yline(80, 'r--', 'LineWidth', 1);  % aspirational 80% reference
    hold off;
    grid on;
    xlabel('Detector (Ranked by MAE)','FontSize',11);
    ylabel('% Within 5 mmHg','FontSize',11);
    title(sprintf('%% Readings Within 5 mmHg of %s (Rank %d-%d)', groupLabel, startIdx, endIdx), ...
        'FontSize',12);
    set(gca,'XTick',1:numInPage,'XTickLabel',shortLabels, ...
        'TickLabelInterpreter','none','XTickLabelRotation',90,'FontSize',8);
    ylim([0 105]);
    legend(ax3, {'% within 5 mmHg','50% line','80% line'}, 'Location','northeast','FontSize',8);
end

% -------------------------------------------------------------------------
function createGroundTruthTabGroup(tabGroup, allRuns, allDetectors)
    runsWithGT = allRuns(cellfun(@(r) r.hasGroundTruth, allRuns));
    if isempty(runsWithGT); return; end
    createRankingTabGroup(tabGroup, runsWithGT, allDetectors, ...
        'error', 'absError', 'Ground Truth', 'GT');
    ppTab = uitab(tabGroup, 'Title', 'Auscultatory GT: firstPPG vs Ensemble vs OscSys');
    createPostProcessorReferenceTab(ppTab, runsWithGT, 'gt', 'Auscultatory GT');
end

function createSBPReferenceTabGroup(tabGroup, allRuns, allDetectors)
    runsWithSBP = allRuns(cellfun(@(r) r.hasSBPReference, allRuns));
    if isempty(runsWithSBP); return; end
    createRankingTabGroup(tabGroup, runsWithSBP, allDetectors, ...
        'sbpError', 'sbpAbsError', 'SBP Reference', 'SBP');
    ppTab = uitab(tabGroup, 'Title', 'SBP Ref: firstPPG vs Ensemble vs Omron');
    createPostProcessorReferenceTab(ppTab, runsWithSBP, 'sbp', 'SBP Reference');
end

% -------------------------------------------------------------------------
function createPostProcessorReferenceTab(parentTab, refRuns, refMode, refLabel)

    numRuns = length(refRuns);

    procKeys   = {'BestConf', 'Ensemble', 'Omron'};
    procLabels = {'firstPPG', 'Ensemble', 'Omron'};
    colors     = {[0.15 0.65 0.35], [0.2 0.5 0.9], [0.9 0.4 0.1]};
    nP         = length(procKeys);

    refVals = nan(1, numRuns);
    ppVals  = nan(nP, numRuns);
    ppValid = false(nP, numRuns);

    for i = 1:numRuns
        if strcmpi(refMode, 'gt')
            if ~refRuns{i}.hasGroundTruth; continue; end
            refVals(i) = refRuns{i}.groundTruthPressure;
        elseif strcmpi(refMode, 'maxgt')
            if ~refRuns{i}.hasMaxGT; continue; end
            refVals(i) = refRuns{i}.maxGTPressure;
        else
            if ~refRuns{i}.hasSBPReference; continue; end
            refVals(i) = refRuns{i}.sbpReference;
            
        end

        for p = 1:nP
            key = procKeys{p};
            if strcmp(key, 'Omron')
                if isfield(refRuns{i}, 'hasOmron') && refRuns{i}.hasOmron
                    ppVals(p, i)  = refRuns{i}.omronSystolic;
                    ppValid(p, i) = true;
                end
            else
                if isfield(refRuns{i}.postValues, key)
                    val = refRuns{i}.postValues.(key);
                    ppVals(p, i)  = val;
                    ppValid(p, i) = true;
                end
            end
        end
    end

    errors    = nan(nP, numRuns);
    absErrors = nan(nP, numRuns);
    for p = 1:nP
        mask              = ppValid(p,:);
        errors(p,mask)    = ppVals(p,mask) - refVals(mask);
        absErrors(p,mask) = abs(errors(p,mask));
    end

        % --- ADDED: 95% range and max/min errors ---
    prc2_5   = nan(nP, 1);
    prc97_5  = nan(nP, 1);
    maxPos   = nan(nP, 1);
    maxNeg   = nan(nP, 1);

    mae      = mean(absErrors, 2, 'omitnan');
    sdAE     = std( absErrors, 0, 2, 'omitnan');
    bias     = mean(errors,    2, 'omitnan');
    sdBias   = std( errors,    0, 2, 'omitnan');
    nValid   = sum(ppValid, 2);
    succRate = nValid / numRuns * 100;

    for p = 1:nP
        if nValid(p) > 0
            errVec = errors(p, ppValid(p,:));
            prc2_5(p)  = prctile(errVec, 2.5);
            prc97_5(p) = prctile(errVec, 97.5);
            maxPos(p)  = max(errVec);
            maxNeg(p)  = min(errVec);
        end
    end

    % Compute % within 5 mmHg per processor
    pct5 = nan(nP, 1);
    for p = 1:nP
        pct5(p) = computePct5(absErrors(p,:));
    end

    % Console summary
    % Console summary (updated)
    fprintf('\n=== POST-PROCESSOR vs %s ===\n', upper(refLabel));
    fprintf('Total runs: %d\n\n', numRuns);
    for p = 1:nP
        fprintf('%s:\n', procLabels{p});
        fprintf('  Valid        : %d / %d  (%.1f%%)\n', nValid(p), numRuns, succRate(p));
        fprintf('  MAE          : %.2f +/- %.2f mmHg\n', mae(p), sdAE(p));
        fprintf('  Bias         : %.2f +/- %.2f mmHg\n', bias(p), sdBias(p));
        fprintf('  Within 5mmHg : %.1f%%\n', pct5(p));
        if nValid(p) > 0
            fprintf('  95%% range    : [%.1f, %.1f] mmHg\n', prc2_5(p), prc97_5(p));
            fprintf('  Max overest  : +%.1f mmHg\n', maxPos(p));
            fprintf('  Max underest : %.1f mmHg\n', maxNeg(p));
        end
        fprintf('\n');
    end

    % --- ADDED: Specific agreement statistics for firstPPG vs GT (or SBP) ---
    if strcmpi(refMode, 'gt') || strcmpi(refMode, 'sbp')
        % Use BestConf (firstPPG) which is index 1 in procKeys
        pBest = 1;  
        if nValid(pBest) > 0
            validMask = ppValid(pBest, :);
            gtVals = refVals(validMask);
            ppValsBest = ppVals(pBest, validMask);
            
            diff = ppValsBest - gtVals;           % signed difference (firstPPG - GT)
            absDiff = abs(diff);
            
            % Agreement within 1 mmHg
            agreeWithin1 = sum(absDiff <= 10);
            pctAgree1 = 100 * agreeWithin1 / nValid(pBest);
            
            % GT higher than firstPPG (any amount)
            gtHigher = sum(diff < 0);
            pctGtHigher = 100 * gtHigher / nValid(pBest);
            
            % firstPPG higher than GT (any amount)
            ppHigher = sum(diff > 0);
            pctPpHigher = 100 * ppHigher / nValid(pBest);
            
            fprintf('FirstPPG vs %s agreement details:\n', refLabel);
            fprintf('  Within 1 mmHg       : %.1f%%  (%d/%d)\n', pctAgree1, agreeWithin1, nValid(pBest));
            fprintf('  %s higher : %.1f%%  (%d/%d)\n', refLabel, pctGtHigher, gtHigher, nValid(pBest));
            fprintf('  FirstPPG higher     : %.1f%%  (%d/%d)\n', pctPpHigher, ppHigher, nValid(pBest));
            fprintf('\n');
        else
            pctAgree1 = NaN; pctGtHigher = NaN; pctPpHigher = NaN;
        end
    end

    % Layout:
    %   Row 1 (top):    3 scatter plots side by side
    %   Row 2 (bottom): MAE bar | Bias bar | Success rate bar | % within 5 bar
    rowH    = 0.36;
    scatBot = 0.54;
    barBot  = 0.08;
    colW    = 0.27;
    gaps    = [0.04, 0.37, 0.70];

    % Row 1 — Scatter plots
    for p = 1:nP
        ax = axes('Parent', parentTab, ...           %#ok<LAXES>
            'Position', [gaps(p), scatBot, colW, rowH]);
        hold(ax, 'on'); grid(ax, 'on');
    
        validIdx = find(ppValid(p,:));
        if ~isempty(validIdx)
            refV = refVals(validIdx);
            ppV  = ppVals(p, validIdx);
            absDiffs = abs(ppV - refV);
    
            withinMask  = absDiffs <= 5;
            outsideMask = ~withinMask;
    
            % Points OUTSIDE 5 mmHg — small
            if any(outsideMask)
                scatter(ax, refV(outsideMask), ppV(outsideMask), 30, ...
                    colors{p}, 'o', 'MarkerFaceAlpha', 0.45, ...
                    'MarkerEdgeColor', colors{p}, 'MarkerFaceColor', 'none', 'LineWidth', 1.2);
            end
    
            % Points WITHIN 5 mmHg — large filled
            if any(withinMask)
                scatter(ax, refV(withinMask), ppV(withinMask), 90, ...
                    colors{p}, 'filled', 'MarkerFaceAlpha', 0.85, ...
                    'MarkerEdgeColor', colors{p} * 0.6, 'LineWidth', 1.0);
            end
        end
    
        allV = [refVals(validIdx), ppVals(p, validIdx)];
        % ... rest of the loop unchanged
        lo = min(allV); hi = max(allV);
        if isempty(lo) || isnan(lo); lo = 60; hi = 180; end
        plot(ax, [lo hi], [lo hi], 'k--', 'LineWidth', 1.2);

        xlabel(ax, sprintf('%s (mmHg)', refLabel), 'FontSize', 11);
        ylabel(ax, sprintf('%s (mmHg)', procLabels{p}), 'FontSize', 11);
        title(ax, sprintf('%s vs %s  (n=%d)', procLabels{p}, refLabel, nValid(p)), ...
            'FontSize', 12, 'FontWeight', 'bold');

        % Annotation now includes % within 5 mmHg
        annStr = sprintf('MAE = %.1f mmHg\nBias = %.1f mmHg\nSuccess = %.0f%%\nWithin 5 mmHg = %.1f%%', ...
            mae(p), bias(p), succRate(p), pct5(p));
        text(ax, 0.03, 0.97, annStr, 'Units', 'normalized', ...
            'VerticalAlignment', 'top', 'FontSize', 10, ...
            'BackgroundColor', [1 1 1 0.75], 'EdgeColor', [0.6 0.6 0.6]);
    end

    % Row 2 — Summary bars (4 panels: MAE | Bias | Success | % within 5)
    barW   = 0.19;
    barGap = 0.245;

    ax_mae = axes('Parent', parentTab, 'Position', [0.03, barBot, barW, rowH*0.7]); %#ok<LAXES>
    hold(ax_mae, 'on'); grid(ax_mae, 'on');
    bh = bar(ax_mae, mae, 0.55);
    errorbar(ax_mae, 1:nP, mae, sdAE, 'k.', 'LineWidth', 1.5, 'CapSize', 8);
    bh.FaceColor = 'flat';
    for p = 1:nP; bh.CData(p,:) = colors{p}; end
    set(ax_mae, 'XTick', 1:nP, 'XTickLabel', procLabels, ...
        'TickLabelInterpreter', 'none', 'FontSize', 10);
    ylabel(ax_mae, 'MAE (mmHg)', 'FontSize', 11);
    title(ax_mae, 'Mean Absolute Error', 'FontSize', 11);

    ax_bias = axes('Parent', parentTab, 'Position', [0.03+barGap, barBot, barW, rowH*0.7]); %#ok<LAXES>
    hold(ax_bias, 'on'); grid(ax_bias, 'on');
    bh2 = bar(ax_bias, bias, 0.55);
    errorbar(ax_bias, 1:nP, bias, sdBias, 'k.', 'LineWidth', 1.5, 'CapSize', 8);
    bh2.FaceColor = 'flat';
    for p = 1:nP; bh2.CData(p,:) = colors{p}; end
    yline(ax_bias, 0, 'k--', 'LineWidth', 1);
    set(ax_bias, 'XTick', 1:nP, 'XTickLabel', procLabels, ...
        'TickLabelInterpreter', 'none', 'FontSize', 10);
    ylabel(ax_bias, 'Mean Bias (mmHg)', 'FontSize', 11);
    title(ax_bias, sprintf('Bias vs %s', refLabel), 'FontSize', 11);

    ax_sr = axes('Parent', parentTab, 'Position', [0.03+2*barGap, barBot, barW, rowH*0.7]); %#ok<LAXES>
    hold(ax_sr, 'on'); grid(ax_sr, 'on');
    bh3 = bar(ax_sr, succRate, 0.55);
    bh3.FaceColor = 'flat';
    for p = 1:nP
        bh3.CData(p,:) = colors{p};
        text(ax_sr, p, succRate(p) + 1.5, sprintf('%.0f%%', succRate(p)), ...
            'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
    end
    set(ax_sr, 'XTick', 1:nP, 'XTickLabel', procLabels, ...
        'TickLabelInterpreter', 'none', 'FontSize', 10);
    ylabel(ax_sr, 'Success Rate (%)', 'FontSize', 11);
    title(ax_sr, 'Valid Reading Rate', 'FontSize', 11);
    ylim(ax_sr, [0 115]);

    % NEW: % within 5 mmHg bar
    ax_p5 = axes('Parent', parentTab, 'Position', [0.03+3*barGap, barBot, barW, rowH*0.7]); %#ok<LAXES>
    hold(ax_p5, 'on'); grid(ax_p5, 'on');
    bh4 = bar(ax_p5, pct5, 0.55);
    bh4.FaceColor = 'flat';
    for p = 1:nP
        bh4.CData(p,:) = colors{p};
        if ~isnan(pct5(p))
            text(ax_p5, p, pct5(p) + 1.5, sprintf('%.1f%%', pct5(p)), ...
                'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
        end
    end
    yline(ax_p5, 80, 'r--', 'LineWidth', 1.2);
    set(ax_p5, 'XTick', 1:nP, 'XTickLabel', procLabels, ...
        'TickLabelInterpreter', 'none', 'FontSize', 10);
    ylabel(ax_p5, '% Within 5 mmHg', 'FontSize', 11);
    title(ax_p5, '% Within 5 mmHg of Reference', 'FontSize', 11);
    ylim(ax_p5, [0 115]);

    annotation(parentTab, 'textbox', [0.01, 0.01, 0.98, 0.025], ...
        'String', ['Note: Omron systolic read from filename (last two numbers before extension).  ' ...
                   'firstPPG key in summary CSV = "BestConf".'], ...
        'FontSize', 8.5, 'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
        'Color', [0.4 0.4 0.4]);
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
    params = nan(numENV, 2);
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
function createBLAnalysisTab_v2(parentTab, allRuns, allDetectors, referenceType)
    if nargin < 4
        referenceType = 'gt';
    end

    blDetectors = allDetectors(startsWith(allDetectors,'BL_'));
    if isempty(blDetectors)
        annotation(parentTab,'textbox',[0.3,0.4,0.4,0.2], ...
            'String','No BL detectors found.','FontSize',14, ...
            'HorizontalAlignment','center','EdgeColor','none');
        return;
    end

    if strcmpi(referenceType, 'sbp')
        refRuns       = allRuns(cellfun(@(r) r.hasSBPReference, allRuns));
        referenceName = 'SBP reference';
        errorField    = 'sbpError';
        absErrorField = 'sbpAbsError';
    else
        refRuns       = allRuns(cellfun(@(r) r.hasGroundTruth, allRuns));
        referenceName = 'ground truth';
        errorField    = 'error';
        absErrorField = 'absError';
    end

    if isempty(refRuns)
        annotation(parentTab,'textbox',[0.3,0.4,0.4,0.2], ...
            'String',sprintf('No runs with %s found.', referenceName),'FontSize',14, ...
            'HorizontalAlignment','center','EdgeColor','none');
        return;
    end
    numRunsRef = length(refRuns);

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

    errors    = nan(numBL, numRunsRef);
    absErrors = nan(numBL, numRunsRef);
    for i = 1:numRunsRef
        for j = 1:numBL
            detName = blDetectors{j};
            if isKey(refRuns{i}.detections, detName)
                det = refRuns{i}.detections(detName);
                if det.detected && isfield(det, errorField)
                    errors(j,i)    = det.(errorField);
                    absErrors(j,i) = abs(det.(errorField));
                end
            end
        end
    end

    ax1 = axes('Parent',parentTab,'Position',[0.08,0.68,0.86,0.26]);
    ax2 = axes('Parent',parentTab,'Position',[0.08,0.38,0.86,0.26]);
    ax3 = axes('Parent',parentTab,'Position',[0.08,0.08,0.86,0.26]);
    plotErrorEffect(ax1, params(:,1), errors, 'Window Size (W)',         'Window Size: Mean Error +/- SD');
    plotErrorEffect(ax2, params(:,2), errors, 'Threshold Multiplier (T)','Threshold: Mean Error +/- SD');
    plotErrorEffect(ax3, params(:,3), errors, 'Minimum Deviation (D)',   'Min Deviation: Mean Error +/- SD');

    fprintf('\n=== BL PARAMETER ERROR ANALYSIS (%s) ===\n', upper(referenceName));
    fprintf('Analyzed %d BL detectors across %d runs with %s\n', numBL, numRunsRef, referenceName);
end

% -------------------------------------------------------------------------
function createAllGroundTruthsSummaryTab(tabGroup, allRuns, allDetectors)
    if isempty(allDetectors)
        return;
    end

    summaryTab = uitab(tabGroup, 'Title', 'All Ground Truths Summary');

    numDetectors = length(allDetectors);
    maeGT        = nan(numDetectors, 1);
    maeSBP       = nan(numDetectors, 1);
    maeCombined  = nan(numDetectors, 1);
    pct5GT       = nan(numDetectors, 1);
    pct5SBP      = nan(numDetectors, 1);
    pct5Combined = nan(numDetectors, 1);
    nGT          = zeros(numDetectors, 1);
    nSBP         = zeros(numDetectors, 1);

    for j = 1:numDetectors
        detName = allDetectors{j};
        gtAbs   = [];
        sbpAbs  = [];
        allAbs  = [];

        for i = 1:length(allRuns)
            if isKey(allRuns{i}.detections, detName)
                det = allRuns{i}.detections(detName);
                if det.detected
                    if isfield(det, 'absError') && ~isnan(det.absError)
                        gtAbs(end+1) = det.absError; %#ok<AGROW>
                        allAbs(end+1) = det.absError; %#ok<AGROW>
                    end
                    if isfield(det, 'sbpAbsError') && ~isnan(det.sbpAbsError)
                        sbpAbs(end+1) = det.sbpAbsError; %#ok<AGROW>
                        allAbs(end+1) = det.sbpAbsError; %#ok<AGROW>
                    end
                end
            end
        end

        maeGT(j)        = mean(gtAbs,  'omitnan');
        maeSBP(j)        = mean(sbpAbs, 'omitnan');
        maeCombined(j)   = mean(allAbs, 'omitnan');
        pct5GT(j)        = computePct5(gtAbs);
        pct5SBP(j)       = computePct5(sbpAbs);
        pct5Combined(j)  = computePct5(allAbs);
        nGT(j)           = numel(gtAbs);
        nSBP(j)          = numel(sbpAbs);
    end

    sortableMAE = maeCombined;
    sortableMAE(isnan(sortableMAE)) = inf;
    [~, idx] = sort(sortableMAE, 'ascend');
    topN = min(20, numDetectors);
    topIdx = idx(1:topN);

    topDetectors = allDetectors(topIdx);
    topMAE       = [maeGT(topIdx), maeSBP(topIdx), maeCombined(topIdx)];
    topPct5      = [pct5GT(topIdx), pct5SBP(topIdx), pct5Combined(topIdx)];

    ax1 = axes('Parent', summaryTab, 'Position', [0.08, 0.56, 0.86, 0.37]);
    bar(ax1, topMAE, 'grouped');
    grid(ax1, 'on');
    title(ax1, 'Top Detectors Across All Ground Truths — MAE (Lower is Better)', 'FontSize', 13);
    ylabel(ax1, 'MAE (mmHg)', 'FontSize', 11);
    set(ax1, 'XTick', 1:topN, 'XTickLabel', shortenDetectorNames(topDetectors), ...
        'TickLabelInterpreter', 'none', 'XTickLabelRotation', 90, 'FontSize', 8);
    legend(ax1, {'GT MAE','SBP MAE','Combined MAE'}, 'Location', 'northwest');

    % NEW: % within 5 mmHg grouped bar
    ax2 = axes('Parent', summaryTab, 'Position', [0.08, 0.10, 0.86, 0.33]);
    bh = bar(ax2, topPct5, 'grouped');
    grid(ax2, 'on');
    title(ax2, '% Within 5 mmHg of Reference — Top Detectors (Higher is Better)', 'FontSize', 12);
    ylabel(ax2, '% Within 5 mmHg', 'FontSize', 11);
    set(ax2, 'XTick', 1:topN, 'XTickLabel', shortenDetectorNames(topDetectors), ...
        'TickLabelInterpreter', 'none', 'XTickLabelRotation', 90, 'FontSize', 8);
    legend(ax2, {'GT %W5','SBP %W5','Combined %W5'}, 'Location', 'northwest');
    ylim(ax2, [0 110]);
    yline(ax2, 80, 'r--', 'LineWidth', 1.2);

    fprintf('\n=== ALL GROUND TRUTHS SUMMARY ===\n');
    fprintf('Ranking by combined MAE using available GT + SBP errors\n');
    fprintf('%-4s %-40s  %8s %6s  %8s %6s  %10s %6s\n', ...
        'Rank','Detector','GT MAE','(n)','SBP MAE','(n)','Combined','%%W5');
    for k = 1:topN
        detName = allDetectors{topIdx(k)};
        fprintf('%2d.  %-40s  %6.2f  (%3d)  %6.2f  (%3d)  %8.2f  %5.1f%%\n', ...
            k, detName, ...
            maeGT(topIdx(k)),       nGT(topIdx(k)), ...
            maeSBP(topIdx(k)),      nSBP(topIdx(k)), ...
            maeCombined(topIdx(k)), pct5Combined(topIdx(k)));
    end
    fprintf('\n');
end

% -------------------------------------------------------------------------
function createErrorDistributionTab(tabGroup, allRuns, allDetectors)
    distTab = uitab(tabGroup, 'Title', 'Error Distribution (GT vs SBP)');

    if isempty(allDetectors)
        annotation(distTab,'textbox',[0.3,0.4,0.4,0.2], ...
            'String','No detectors available for error distribution.','FontSize',14, ...
            'HorizontalAlignment','center','EdgeColor','none');
        return;
    end

    numDetectors = length(allDetectors);
    gtErrorsByDet  = cell(numDetectors,1);
    sbpErrorsByDet = cell(numDetectors,1);

    for j = 1:numDetectors
        detName = allDetectors{j};
        gtErr = [];
        sbpErr = [];

        for i = 1:length(allRuns)
            if isKey(allRuns{i}.detections, detName)
                det = allRuns{i}.detections(detName);
                if det.detected
                    if isfield(det, 'error') && ~isnan(det.error)
                        gtErr(end+1) = det.error; %#ok<AGROW>
                    end
                    if isfield(det, 'sbpError') && ~isnan(det.sbpError)
                        sbpErr(end+1) = det.sbpError; %#ok<AGROW>
                    end
                end
            end
        end

        gtErrorsByDet{j} = gtErr;
        sbpErrorsByDet{j} = sbpErr;
    end

    allGTErr  = [gtErrorsByDet{:}];
    allSBPErr = [sbpErrorsByDet{:}];

    if isempty(allGTErr) && isempty(allSBPErr)
        annotation(distTab,'textbox',[0.25,0.4,0.5,0.2], ...
            'String','No valid error values found for either Ground Truth or SBP reference.', ...
            'FontSize',13,'HorizontalAlignment','center','EdgeColor','none');
        return;
    end

    % Top panel: overall histogram overlay
    ax1 = axes('Parent', distTab, 'Position', [0.08, 0.56, 0.86, 0.36]);
    hold(ax1, 'on'); grid(ax1, 'on');

    rangeVals = [allGTErr(:); allSBPErr(:)];
    if isempty(rangeVals); rangeVals = [-20; 20]; end
    lo = floor(min(rangeVals) / 5) * 5;
    hi = ceil(max(rangeVals) / 5) * 5;
    if lo == hi; lo = lo - 10; hi = hi + 10; end
    edges = linspace(lo, hi, 35);

    if ~isempty(allGTErr)
        histogram(ax1, allGTErr, edges, 'Normalization', 'probability', ...
            'FaceColor', [0.2 0.5 0.9], 'FaceAlpha', 0.45, 'EdgeColor', 'none');
        xline(ax1, mean(allGTErr,'omitnan'), '--', 'Color', [0.2 0.5 0.9], 'LineWidth', 1.5);
    end
    if ~isempty(allSBPErr)
        histogram(ax1, allSBPErr, edges, 'Normalization', 'probability', ...
            'FaceColor', [0.9 0.4 0.1], 'FaceAlpha', 0.45, 'EdgeColor', 'none');
        xline(ax1, mean(allSBPErr,'omitnan'), '--', 'Color', [0.9 0.4 0.1], 'LineWidth', 1.5);
    end

    xlabel(ax1, 'Error (Detected - Reference) [mmHg]', 'FontSize', 11);
    ylabel(ax1, 'Probability', 'FontSize', 11);
    title(ax1, 'Overall Error Distribution: Ground Truth vs SBP Reference', 'FontSize', 12);
    legendEntries = {};
    if ~isempty(allGTErr); legendEntries{end+1} = sprintf('GT (n=%d)', numel(allGTErr)); end
    if ~isempty(allSBPErr); legendEntries{end+1} = sprintf('SBP (n=%d)', numel(allSBPErr)); end
    if ~isempty(legendEntries)
        legend(ax1, legendEntries, 'Location', 'northeast');
    end

    % Bottom panel: per-detector mean +/- std error (top combined MAE)
    maeCombined = inf(numDetectors,1);
    for j = 1:numDetectors
        ae = [abs(gtErrorsByDet{j}), abs(sbpErrorsByDet{j})];
        if ~isempty(ae); maeCombined(j) = mean(ae, 'omitnan'); end
    end
    [~, sortIdx] = sort(maeCombined, 'ascend');
    validIdx = sortIdx(isfinite(maeCombined(sortIdx)));

    topN = min(20, length(validIdx));
    if topN == 0; return; end
    topIdx = validIdx(1:topN);
    topNames = allDetectors(topIdx);

    meanGT  = nan(topN,1); stdGT  = nan(topN,1);
    meanSBP = nan(topN,1); stdSBP = nan(topN,1);
    pct5GT  = nan(topN,1); pct5SBP = nan(topN,1);
    for k = 1:topN
        g = gtErrorsByDet{topIdx(k)};
        s = sbpErrorsByDet{topIdx(k)};
        if ~isempty(g)
            meanGT(k) = mean(g,'omitnan');
            stdGT(k)  = std(g,0,'omitnan');
            pct5GT(k) = computePct5(abs(g));
        end
        if ~isempty(s)
            meanSBP(k)  = mean(s,'omitnan');
            stdSBP(k)   = std(s,0,'omitnan');
            pct5SBP(k)  = computePct5(abs(s));
        end
    end

    ax2 = axes('Parent', distTab, 'Position', [0.08, 0.10, 0.86, 0.34]);
    hold(ax2, 'on'); grid(ax2, 'on');
    x = 1:topN;

    errorbar(ax2, x-0.12, meanGT, stdGT, 'o', ...
        'Color', [0.2 0.5 0.9], 'MarkerFaceColor', [0.2 0.5 0.9], ...
        'LineWidth', 1.2, 'CapSize', 6);
    errorbar(ax2, x+0.12, meanSBP, stdSBP, 'o', ...
        'Color', [0.9 0.4 0.1], 'MarkerFaceColor', [0.9 0.4 0.1], ...
        'LineWidth', 1.2, 'CapSize', 6);

    yline(ax2, 0, 'k--', 'LineWidth', 1);

    % Annotate each detector with % within 5 mmHg (GT in blue, SBP in orange)
    for k = 1:topN
        if ~isnan(pct5GT(k))
            text(ax2, k-0.12, meanGT(k), sprintf('  %.0f%%', pct5GT(k)), ...
                'FontSize', 6, 'Color', [0.2 0.5 0.9], 'VerticalAlignment', 'middle');
        end
        if ~isnan(pct5SBP(k))
            text(ax2, k+0.12, meanSBP(k), sprintf('  %.0f%%', pct5SBP(k)), ...
                'FontSize', 6, 'Color', [0.9 0.4 0.1], 'VerticalAlignment', 'middle');
        end
    end

    set(ax2, 'XTick', x, 'XTickLabel', shortenDetectorNames(topNames), ...
        'TickLabelInterpreter', 'none', 'XTickLabelRotation', 90, 'FontSize', 8);
    xlabel(ax2, 'Top Detectors (Ranked by Combined MAE)', 'FontSize', 11);
    ylabel(ax2, 'Mean Error +/- SD (mmHg)', 'FontSize', 11);
    title(ax2, 'Per-Detector Error Spread — Mean±SD with % Within 5 mmHg Annotated', 'FontSize', 12);
    legend(ax2, {'GT mean+/-SD  (% shown)','SBP mean+/-SD  (% shown)'}, 'Location', 'northwest');

    fprintf('\n=== ERROR DISTRIBUTION (GT vs SBP) ===\n');
    fprintf('GT samples: %d | SBP samples: %d\n', numel(allGTErr), numel(allSBPErr));
    if ~isempty(allGTErr)
        fprintf('GT   mean=%.2f, std=%.2f, median=%.2f, within5mmHg=%.1f%%\n', ...
            mean(allGTErr,'omitnan'), std(allGTErr,0,'omitnan'), ...
            median(allGTErr,'omitnan'), computePct5(abs(allGTErr)));
    end
    if ~isempty(allSBPErr)
        fprintf('SBP  mean=%.2f, std=%.2f, median=%.2f, within5mmHg=%.1f%%\n', ...
            mean(allSBPErr,'omitnan'), std(allSBPErr,0,'omitnan'), ...
            median(allSBPErr,'omitnan'), computePct5(abs(allSBPErr)));
    end
    fprintf('\n');
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

% -------------------------------------------------------------------------
function createDetectorSubplotsTab(parentTab, runInfo, pageNum, detPerPage)
    if nargin < 3; pageNum   = 1; end
    if nargin < 4; detPerPage = 20; end

    allDetOnPage = runInfo.detectors;
    numTotal = length(allDetOnPage);
    if numTotal == 0; return; end

    startIdx = (pageNum - 1) * detPerPage + 1;
    endIdx   = min(pageNum  * detPerPage, numTotal);
    detectors = allDetOnPage(startIdx:endIdx);
    numDet = length(detectors);

    nCols = min(5, numDet);
    nRows = ceil(numDet / nCols);

    lm = 0.01; rm = 0.01;
    tm = 0.04; bm = 0.04;
    hGap = 0.008; vGap = 0.06;

    cellW = (1 - lm - rm - (nCols-1)*hGap) / nCols;
    cellH = (1 - tm - bm - (nRows-1)*vGap) / nRows;

    ausX = []; ausPressure = []; ausLeft = [];
    if runInfo.hasAusPulse && ~isempty(runInfo.pulseHeardIndices)
        idx         = runInfo.pulseHeardIndices;
        ausX        = runInfo.time(idx);
        ausPressure = runInfo.pressure(idx);
        if runInfo.hasRawPPG
            ausLeft = runInfo.rawPPG(idx);
        else
            ausLeft = runInfo.ppg(idx);
        end
    end

    for j = 1:numDet
        detName = detectors{j};

        col = mod(j-1, nCols) + 1;
        row = ceil(j / nCols);

        x = lm + (col-1)*(cellW + hGap);
        y = 1 - tm - row*(cellH) - (row-1)*vGap;

        ax = axes('Parent', parentTab, 'Position', [x, y, cellW, cellH]); %#ok<LAXES>

        yyaxis(ax, 'left');
        hold(ax, 'on');
        if runInfo.hasRawPPG
            plot(ax, runInfo.time, runInfo.rawPPG, ...
                 'Color', [1 0.72 0.72], 'LineWidth', 0.6);
        end
        plot(ax, runInfo.time, runInfo.ppg, 'r-', 'LineWidth', 0.9);
        ax.YColor = [0.8 0 0];
        set(ax, 'YTickLabel', []);

        yyaxis(ax, 'right');
        plot(ax, runInfo.time, runInfo.pressure, 'b-', 'LineWidth', 1.0);
        ax.YColor = [0 0.2 0.8];
        set(ax, 'YTickLabel', []);

        detected = false;
        if isKey(runInfo.detections, detName)
            det = runInfo.detections(detName);
            if det.detected && ~isnan(det.time)
                detected = true;
                xline(ax, det.time, '--', 'Color', [0 0.7 0], 'LineWidth', 1.4);
                yyaxis(ax, 'right');
                plot(ax, det.time, det.pressure, 'o', ...
                     'Color', [0 0.65 0], 'MarkerFaceColor', [0.4 1 0.4], ...
                     'MarkerSize', 5, 'LineWidth', 1.2);
            end
        end

        if ~isempty(ausX)
            yyaxis(ax, 'right');
            plot(ax, ausX, ausPressure, 'bx', 'MarkerSize', 7, 'LineWidth', 1.8);
        end

        if detected
            if runInfo.hasGroundTruth && ~isnan(det.error)
                ttl = sprintf('%s\n%.0f mmHg (err=%.1f)', ...
                    strrep(detName,'_','\_'), det.pressure, det.error);
            else
                ttl = sprintf('%s\n%.0f mmHg', strrep(detName,'_','\_'), det.pressure);
            end
            titleColor = [0 0.5 0];
        else
            ttl = sprintf('%s\n—', strrep(detName,'_','\_'));
            titleColor = [0.6 0 0];
        end
        title(ax, ttl, 'FontSize', 6.5, 'Color', titleColor, ...
              'Interpreter', 'tex', 'FontWeight', 'normal');

        set(ax, 'XTickLabel', [], 'FontSize', 6, 'TickLength', [0.01 0.01]);
        grid(ax, 'on'); ax.GridAlpha = 0.2;
    end

    annotation(parentTab, 'textbox', [0, 0.965, 1, 0.03], ...
        'String', sprintf('Detectors %d–%d of %d   |   %s   |   Green line = detection   Blue × = AUS pulse heard', ...
            startIdx, endIdx, numTotal, strrep(runInfo.displayName, '_', '\_')), ...
        'FontSize', 8, 'EdgeColor', 'none', ...
        'HorizontalAlignment', 'center', 'Color', [0.25 0.25 0.25], ...
        'Interpreter', 'tex');
end

% -------------------------------------------------------------------------
function createSubjectRunTabs(tabGroup, allRuns)
    numRuns    = length(allRuns);
    subjectIDs = cell(numRuns, 1);

    for i = 1:numRuns
        [~, fname, ~] = fileparts(allRuns{i}.filename);
        tok = regexpi(fname, 'subject([A-Za-z0-9]+)_', 'tokens');
        if ~isempty(tok)
            subjectIDs{i} = upper(strtrim(tok{1}{1}));
        else
            subjectIDs{i} = '';
        end
    end

    hasSubject = ~cellfun(@isempty, subjectIDs);
    if ~any(hasSubject)
        fprintf('[Subject Analysis] No subject IDs found in filenames — skipping.\n');
        return;
    end

    uniqueSubjects = unique(subjectIDs(hasSubject));
    fprintf('\n[Subject Analysis] Found %d unique subjects: %s\n', ...
        length(uniqueSubjects), strjoin(uniqueSubjects, ', '));

    refConfigs = {
        'gt',  'AUS GT';
        'sbp', 'SBP Ref';
    };

    methodConfigs = {
        'reference', 'Reference per Run';
        'BestConf',  'firstPPG Error';
        'Ensemble',  'Ensemble Error';
        'Omron',     'Omron Error';
    };

    for c = 1:size(refConfigs, 1)
        refMode  = refConfigs{c, 1};
        refShort = refConfigs{c, 2};

        for m = 1:size(methodConfigs, 1)
            methodKey   = methodConfigs{m, 1};
            methodLabel = methodConfigs{m, 2};

            tabTitle = sprintf('%s: %s', refShort, methodLabel);
            tab = uitab(tabGroup, 'Title', tabTitle);
            createSubjectRunBarTab(tab, allRuns, subjectIDs, uniqueSubjects, ...
                methodKey, refMode, ...
                sprintf('Subject Analysis — %s — %s', refShort, methodLabel));
        end
    end
end

% -------------------------------------------------------------------------
function createSubjectRunBarTab(parentTab, allRuns, subjectIDs, uniqueSubjects, ...
        methodKey, refMode, titleStr)
    nSubjects = length(uniqueSubjects);
    GAP       = 1.5;

    subjRunIndices = cell(nSubjects, 1);
    for s = 1:nSubjects
        subjRunIndices{s} = find(strcmp(subjectIDs, uniqueSubjects{s}));
    end

    barXPos    = [];
    barSubj    = [];
    barRunNum  = [];
    xTickPos   = zeros(1, nSubjects);
    xSepLines  = zeros(1, nSubjects - 1);
    cursor     = 0;

    for s = 1:nSubjects
        n          = length(subjRunIndices{s});
        groupStart = cursor + 1;
        groupEnd   = cursor + n;
        barXPos    = [barXPos,   groupStart:groupEnd];   %#ok<AGROW>
        barSubj    = [barSubj,   repmat(s, 1, n)];       %#ok<AGROW>
        barRunNum  = [barRunNum, 1:n];                   %#ok<AGROW>
        xTickPos(s) = (groupStart + groupEnd) / 2;
        if s < nSubjects
            xSepLines(s) = groupEnd + GAP / 2;
        end
        cursor = groupEnd + GAP;
    end

    nBars = length(barXPos);
    vals  = nan(1, nBars);

    barIdx = 0;
    for s = 1:nSubjects
        for r = 1:length(subjRunIndices{s})
            barIdx = barIdx + 1;
            vals(barIdx) = getSubjectBarValue( ...
                allRuns{subjRunIndices{s}(r)}, methodKey, refMode);
        end
    end

    maxRunsAny = max(cellfun(@length, subjRunIndices));
    runColors  = lines(max(maxRunsAny, 1));

    ax = axes('Parent', parentTab, 'Position', [0.07, 0.16, 0.89, 0.75]); %#ok<LAXES>
    hold(ax, 'on');
    grid(ax, 'on');

    for i = 1:nBars
        if isnan(vals(i)); continue; end
        rn = barRunNum(i);
        bar(ax, barXPos(i), vals(i), 0.75, ...
            'FaceColor', runColors(rn, :), ...
            'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 0.5);
    end

    if ~strcmp(methodKey, 'reference')
        yline(ax, 0, 'k--', 'LineWidth', 1.5);
        % Draw ±5 mmHg band for error plots
        yline(ax,  5, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);
        yline(ax, -5, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);
    end

    for s = 1:length(xSepLines)
        xline(ax, xSepLines(s), ':', 'Color', [0.65 0.65 0.65], 'LineWidth', 1);
    end

    for i = 1:nBars
        if isnan(vals(i)); continue; end
        if vals(i) >= 0; va = 'bottom'; else; va = 'top'; end
        text(ax, barXPos(i), vals(i), sprintf('%.0f', vals(i)), ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment',   va, ...
            'FontSize', 7, 'Color', [0.15 0.15 0.15]);
    end

    set(ax, 'XTick', xTickPos, 'XTickLabel', uniqueSubjects, ...
        'TickLabelInterpreter', 'none', 'FontSize', 10, 'XTickLabelRotation', 0);
    xlabel(ax, 'Subject', 'FontSize', 12);

    if strcmp(methodKey, 'reference')
        if strcmp(refMode, 'gt')
            ylabel(ax, 'Auscultatory Pressure (mmHg)', 'FontSize', 12);
        else
            ylabel(ax, 'SBP Reference from Filename (mmHg)', 'FontSize', 12);
        end
    else
        ylabel(ax, 'Error: Algorithm − Reference (mmHg)', 'FontSize', 12);
    end

    title(ax, titleStr, 'FontSize', 13, 'Interpreter', 'none');

    if maxRunsAny > 1
        lh = arrayfun(@(r) patch(ax, NaN, NaN, runColors(r, :), ...
            'EdgeColor', [0.2 0.2 0.2]), 1:maxRunsAny);
        legend(ax, lh, ...
            arrayfun(@(r) sprintf('Run %d', r), 1:maxRunsAny, 'UniformOutput', false), ...
            'Location', 'best', 'FontSize', 9);
    end
end

% -------------------------------------------------------------------------
function val = getSubjectBarValue(runInfo, methodKey, refMode)
    val = NaN;
    if strcmp(refMode, 'gt')
        if ~runInfo.hasGroundTruth; return; end
        refVal = runInfo.groundTruthPressure;
    else
        if ~runInfo.hasSBPReference; return; end
        refVal = runInfo.sbpReference;
    end
    if strcmp(methodKey, 'reference')
        val = refVal;
        return;
    end
    if strcmp(methodKey, 'Omron')
        if ~runInfo.hasOmron; return; end
        val = runInfo.omronSystolic - refVal;
        return;
    end
    if ~isfield(runInfo, 'postValues') || ~isfield(runInfo.postValues, methodKey)
        return;
    end
    val = runInfo.postValues.(methodKey) - refVal;
end

% -------------------------------------------------------------------------
function toggleLegendItem(legendObj, evt)
    peer = evt.Peer;
    if strcmp(peer.Visible, 'on')
        newVis = 'off';
    else
        newVis = 'on';
    end
    peer.Visible = newVis;
    detMap = getappdata(legendObj, 'detHandlesMap');
    if ~isempty(detMap)
        fields = fieldnames(detMap);
        for f = 1:length(fields)
            bundle = detMap.(fields{f});
            for b = 1:length(bundle)
                if isgraphics(bundle(b)) && bundle(b) == peer
                    for bb = 1:length(bundle)
                        if isgraphics(bundle(bb))
                            bundle(bb).Visible = newVis;
                        end
                    end
                    break;
                end
            end
        end
    end
    try
        entryObj = legendObj.EntryContainer.NodeChildren;
        for i = 1:length(entryObj)
            if isprop(entryObj(i), 'Object') && entryObj(i).Object == peer
                labelObj = entryObj(i).Label;
                if strcmp(newVis, 'off')
                    labelObj.Color = [0.65 0.65 0.65];
                else
                    labelObj.Color = [0 0 0];
                end
                break;
            end
        end
    catch
    end
end


function createMaxGroundTruthTabGroup(tabGroup, maxGTRuns, allDetectors)
    if isempty(maxGTRuns); return; end
    createRankingTabGroup(tabGroup, maxGTRuns, allDetectors, ...
        'maxError', 'maxAbsError', 'Max Ground Truth', 'MaxGT');
    ppTab = uitab(tabGroup, 'Title', 'Max GT: firstPPG vs Ensemble vs Omron');
    createPostProcessorReferenceTab(ppTab, maxGTRuns, 'maxgt', 'Max Ground Truth');
end


function createSBPvsAUSComparisonTab(parentTab, allRuns)
    % Collect runs that have both references
    validRuns = {};
    for i = 1:length(allRuns)
        if allRuns{i}.hasGroundTruth && allRuns{i}.hasSBPReference
            validRuns{end+1} = allRuns{i};
        end
    end
    if isempty(validRuns)
        annotation(parentTab,'textbox',[0.3,0.4,0.4,0.2], ...
            'String','No runs with both references found.','FontSize',14, ...
            'HorizontalAlignment','center','EdgeColor','none');
        return;
    end

    n = length(validRuns);
    ausVals = zeros(n,1);
    sbpVals = zeros(n,1);
    for i = 1:n
        ausVals(i) = validRuns{i}.groundTruthPressure;
        sbpVals(i) = validRuns{i}.sbpReference;
    end

    ax = axes('Parent', parentTab, 'Position', [0.13, 0.13, 0.74, 0.78]);
    hold(ax, 'on'); grid(ax, 'on');
    scatter(ax, ausVals, sbpVals, 70, 'filled', ...
        'MarkerFaceColor', [0.2 0.5 0.8], 'MarkerEdgeColor', 'k', 'LineWidth', 0.8);

    % Identity line
    lims = [min([ausVals; sbpVals]), max([ausVals; sbpVals])];
    plot(ax, lims, lims, 'k--', 'LineWidth', 1.5);

    % Compute metrics
    diffMean = mean(sbpVals - ausVals, 'omitnan');
    diffStd  = std(sbpVals - ausVals, 'omitnan');
    mae      = mean(abs(sbpVals - ausVals), 'omitnan');
    r2       = 1 - sum((sbpVals - ausVals).^2) / sum((sbpVals - mean(sbpVals)).^2);

    % Annotation
    annStr = sprintf('n = %d\nMAE = %.1f mmHg\nBias = %.1f ± %.1f mmHg\nR² = %.3f', ...
        n, mae, diffMean, diffStd, r2);
    text(ax, 0.03, 0.97, annStr, 'Units', 'normalized', ...
        'VerticalAlignment', 'top', 'BackgroundColor', [1 1 1 0.8], ...
        'EdgeColor', [0.6 0.6 0.6], 'FontSize', 10);

    xlabel(ax, 'Auscultatory Ground Truth (mmHg)', 'FontSize', 12);
    ylabel(ax, 'SBP Reference from Filename (mmHg)', 'FontSize', 12);
    title(ax, 'Comparison of Two Reference Standards', 'FontSize', 13);
end
% -------------------------------------------------------------------------
function createConfidenceHeatmapTab(parentTab, runInfo, allDetectors)
    numDet = length(allDetectors);

    errVals  = nan(1, numDet);
    presVals = nan(1, numDet);

    % Best available reference: maxGT > auscultatory GT > SBP ref
    if isfield(runInfo,'hasMaxGT') && runInfo.hasMaxGT
        refPressure = runInfo.maxGTPressure;
        refLabel    = sprintf('Max GT = %.1f mmHg', refPressure);
    elseif isfield(runInfo,'hasGroundTruth') && runInfo.hasGroundTruth
        refPressure = runInfo.groundTruthPressure;
        refLabel    = sprintf('AUS GT = %.1f mmHg', refPressure);
    elseif isfield(runInfo,'hasSBPReference') && runInfo.hasSBPReference
        refPressure = runInfo.sbpReference;
        refLabel    = sprintf('SBP Ref = %d mmHg', refPressure);
    else
        refPressure = NaN;
        refLabel    = 'No reference';
    end

    for j = 1:numDet
        if isKey(runInfo.detections, allDetectors{j})
            det = runInfo.detections(allDetectors{j});
            if det.detected
                presVals(j) = det.pressure;
                if ~isnan(refPressure)
                    errVals(j) = det.pressure - refPressure;
                end
            end
        end
    end

    panel = uipanel('Parent', parentTab, 'Units', 'normalized', ...
        'Position', [0, 0, 1, 1], 'BorderType', 'none');

    drawErrorGrid(panel, runInfo, allDetectors, errVals, presVals, refPressure, refLabel);
    panel.SizeChangedFcn = @(src, ~) drawErrorGrid( ...
        src, runInfo, allDetectors, errVals, presVals, refPressure, refLabel);
end

% -------------------------------------------------------------------------
function drawErrorGrid(panel, runInfo, allDetectors, errVals, presVals, refPressure, refLabel)
    delete(panel.Children);

    numDet = length(allDetectors);
    if numDet == 0; return; end

    % Panel pixel size
    pxPos  = getpixelposition(panel);
    panelW = max(pxPos(3), 200);
    panelH = max(pxPos(4), 200);

    % --- Perfect square grid dimensions ---
    nSide = ceil(sqrt(numDet));   % nRows == nCols == nSide

    % Color thresholds (absolute error, mmHg)
    C_GREEN   = [0.04, 0.53, 0.18];   % |err| < 3
    C_LGREEN  = [0.42, 0.80, 0.35];   % |err| < 5
    C_YELLOW  = [0.91, 0.77, 0.04];   % |err| < 8
    C_ORANGE  = [0.93, 0.49, 0.10];   % |err| < 10
    C_RED     = [0.80, 0.11, 0.11];   % |err| < 15
    C_DARKRED = [0.42, 0.00, 0.00];   % |err| >= 15
    C_GREY    = [0.46, 0.46, 0.46];   % no detection

    function rgb = pickColor(err)
        if isnan(err);          rgb = C_GREY;
        elseif abs(err) <  3;   rgb = C_GREEN;
        elseif abs(err) <  5;   rgb = C_LGREEN;
        elseif abs(err) <  8;   rgb = C_YELLOW;
        elseif abs(err) < 10;   rgb = C_ORANGE;
        elseif abs(err) < 15;   rgb = C_RED;
        else;                   rgb = C_DARKRED;
        end
    end

    % ---- Compute a pixel-square axes region --------------------------------
    % Reserve space for title (top) and legend (bottom) in pixels
    titlePx  = 32;
    legendPx = 44;
    availW   = panelW * 0.98;
    availH   = panelH - titlePx - legendPx;

    % Square side in pixels, then convert back to normalized panel coords
    squarePx = max(10, min(availW, availH));
    normW    = squarePx / panelW;
    normH    = squarePx / panelH;
    normLeft = (1 - normW) / 2;
    normBot  = legendPx / panelH;

    ax = axes('Parent', panel, ...
        'Position', [normLeft, normBot, normW, normH], ...
        'Units', 'normalized');
    hold(ax, 'on');
    axis(ax, 'off');
    xlim(ax, [0, nSide]);
    ylim(ax, [0, nSide]);

    % Font size scaled to cell pixel size
    cellPx   = squarePx / nSide;
    fontSize = max(2, min(6, floor(cellPx / 10)));

    for k = 1:numDet
        col  = mod(k-1, nSide);               % 0-based column index
        row  = nSide - 1 - floor((k-1)/nSide); % 0-based row (flipped: row 0 = top)
        xPos = col;
        yPos = row;

        cellColor = pickColor(errVals(k));

        rectangle(ax, ...
            'Position',  [xPos+0.04, yPos+0.05, 0.92, 0.88], ...
            'FaceColor', cellColor, ...
            'EdgeColor', [0.18, 0.18, 0.18], ...
            'LineWidth', 0.4, ...
            'Curvature', 0.12);

        lum       = 0.299*cellColor(1) + 0.587*cellColor(2) + 0.114*cellColor(3);
        textColor = [0, 0, 0];
        if lum < 0.45; textColor = [1, 1, 1]; end

        % Detector short name
        shortName = strrep(allDetectors{k}, 'BL_',  '');
        shortName = strrep(shortName,        'ENV_', 'ENV');
        shortName = strrep(shortName,        'DRV_', 'DRV');
        shortName = strrep(shortName,        '_',    '');
        if length(shortName) > 14
            shortName = [shortName(1:13), char(8230)];
        end

        text(ax, xPos+0.50, yPos+0.65, shortName, ...
            'FontSize', fontSize, 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', 'Interpreter', 'none', ...
            'Color', textColor, 'Clipping', 'on');

        % Signed error value
        if ~isnan(errVals(k))
            valStr = sprintf('%+.1f', errVals(k));
        elseif ~isnan(presVals(k)) && isnan(refPressure)
            valStr = sprintf('%.0f?', presVals(k));
        else
            valStr = char(8212);
        end

        text(ax, xPos+0.50, yPos+0.26, valStr, ...
            'FontSize', max(5, fontSize-1), 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', 'Interpreter', 'none', ...
            'Color', textColor, 'FontWeight', 'bold', 'Clipping', 'on');
    end

    % Fill unused cells in the square with a neutral placeholder
    for k = numDet+1 : nSide^2
        col  = mod(k-1, nSide);
        row  = nSide - 1 - floor((k-1)/nSide);
        rectangle(ax, ...
            'Position',  [col+0.04, row+0.05, 0.92, 0.88], ...
            'FaceColor', [0.93 0.93 0.93], ...
            'EdgeColor', [0.82, 0.82, 0.82], ...
            'LineWidth', 0.3, ...
            'Curvature', 0.12);
    end

    % ---- Title (above the square axes) -----------------------------------
    numDetected = sum(~isnan(errVals));
    if ~isnan(refPressure)
        numIn3  = sum(abs(errVals(~isnan(errVals))) <  3);
        numIn5  = sum(abs(errVals(~isnan(errVals))) <  5);
        titleStr = sprintf( ...
            '%s  |  %s  |  %d/%d detected  |  within 3 mmHg: %d   within 5 mmHg: %d', ...
            runInfo.displayName, refLabel, numDetected, numDet, numIn3, numIn5);
    else
        titleStr = sprintf('%s  |  No reference  |  %d/%d detected', ...
            runInfo.displayName, numDetected, numDet);
    end
    title(ax, titleStr, 'FontSize', 9, 'FontWeight', 'bold', 'Interpreter', 'none');

    % ---- Legend (below the square, fixed pixel band) ---------------------
    legAx = axes('Parent', panel, ...
        'Position', [0.01, 0.002, 0.98, legendPx/panelH - 0.004], ...
        'Units', 'normalized');
    hold(legAx, 'on');
    axis(legAx, 'off');
    xlim(legAx, [0, 1]);
    ylim(legAx, [0, 1]);

    legColors = {C_GREEN, C_LGREEN, C_YELLOW, C_ORANGE, C_RED, C_DARKRED, C_GREY};
    legLabels  = {'< 3 mmHg', '3–5 mmHg', '5–8 mmHg', '8–10 mmHg', ...
                  '10–15 mmHg', '≥ 15 mmHg', 'No detection'};

    slotW    = 1 / numel(legColors);
    legFSize = max(6, min(9, floor(panelW * slotW * 0.085)));

    for li = 1:numel(legColors)
        xL = (li-1) * slotW;
        rectangle(legAx, 'Position', [xL+0.004, 0.06, 0.022, 0.86], ...
            'FaceColor', legColors{li}, 'EdgeColor', [0.2,0.2,0.2], 'LineWidth', 0.5);
        text(legAx, xL+0.030, 0.50, legLabels{li}, ...
            'FontSize', legFSize, 'VerticalAlignment', 'middle', ...
            'Interpreter', 'none', 'Clipping', 'on', ...
            'Color', [0,0,0] );
    end
end

function exportResultsCSV(allRuns, outputPath)
% exportResultsCSV  Write a summary CSV with one row per run.
%
%   exportResultsCSV(allRuns)
%       Prompts for a save location.
%
%   exportResultsCSV(allRuns, '/path/to/output.csv')
%       Saves directly to the given path.
%
% Columns
% -------
%   Subject | Run | Auscultatory True Systolic | Visual True Systolic |
%   Omron Systolic | Algorithm Systolic (Ensemble) | Most Confident (BestConf)
%
% Notes
% -----
%   * "Visual True Systolic"  = SBP value parsed from the filename  (sbpReference).
%   * "Algorithm Systolic"    = Ensemble post-processor value.
%   * "Most Confident"        = BestConf post-processor (firstPPG in the UI).
%   * Empty cells are written as blank (not NaN / 0).

    %% ── output file path ──────────────────────────────────────────────────
    if nargin < 2 || isempty(outputPath)
        [file, path] = uiputfile('*.csv', 'Save results CSV as', 'bp_results.csv');
        if isequal(file, 0)
            disp('Export cancelled.');
            return;
        end
        outputPath = fullfile(path, file);
    end

    %% ── open file ─────────────────────────────────────────────────────────
    fid = fopen(outputPath, 'w');
    if fid == -1
        error('exportResultsCSV: could not open "%s" for writing.', outputPath);
    end

    %% ── header ────────────────────────────────────────────────────────────
    fprintf(fid, '%s\n', strjoin({ ...
        'Subject', ...
        'Run', ...
        'Auscultatory True Systolic (mmHg)', ...
        'Visual True Systolic (mmHg)', ...
        'Omron Systolic (mmHg)', ...
        'Algorithm Systolic - Ensemble (mmHg)', ...
        'Most Confident (mmHg)' ...
    }, ','));

    %% ── one row per run ───────────────────────────────────────────────────
    for i = 1:length(allRuns)
        r = allRuns{i};

        % ── Subject ──────────────────────────────────────────────────────
        % Parse subject number/letter code from filename, e.g. "subject3B_"
        % gives "3B".  Falls back to the full filename base if not found.
        [~, fname, ~] = fileparts(r.filename);
        tok = regexpi(fname, 'subject([A-Za-z0-9]+)[_\s]', 'tokens');
        if ~isempty(tok)
            subject = upper(strtrim(tok{1}{1}));
        else
            subject = fname;
        end
        subject = csvSafe(subject);

        % ── Run index ────────────────────────────────────────────────────
        runNum = num2str(r.runIdx);

        % ── Auscultatory True Systolic ───────────────────────────────────
        if isfield(r, 'hasGroundTruth') && r.hasGroundTruth
            ausCol = sprintf('%.1f', r.groundTruthPressure);
        else
            ausCol = '';
        end

        % ── Visual True Systolic (SBP from filename) ─────────────────────
        if isfield(r, 'hasSBPReference') && r.hasSBPReference
            sbpCol = sprintf('%d', r.sbpReference);
        else
            sbpCol = '';
        end

        % ── Omron Systolic ───────────────────────────────────────────────
        if isfield(r, 'hasOmron') && r.hasOmron
            omronCol = sprintf('%d', r.omronSystolic);
        else
            omronCol = '';
        end

        % ── Ensemble (Algorithm Systolic) ────────────────────────────────
        ensCol = getPostValue(r, 'Ensemble');

        % ── BestConf → "Most Confident" ──────────────────────────────────
        bestCol = getPostValue(r, 'BestConf');

        % ── write row ────────────────────────────────────────────────────
        fprintf(fid, '%s\n', strjoin({ ...
            subject, runNum, ausCol, sbpCol, omronCol, ensCol, bestCol ...
        }, ','));
    end

    fclose(fid);
    fprintf('Saved %d rows to: %s\n', length(allRuns), outputPath);
end

%% ── helpers ───────────────────────────────────────────────────────────────

function val = getPostValue(r, key)
% Return the post-processor value as a string, or '' if missing.
    val = '';
    if isfield(r, 'postValues') && isfield(r.postValues, key)
        v = r.postValues.(key);
        if isnumeric(v) && ~isnan(v)
            val = sprintf('%.1f', v);
        end
    end
end

function s = csvSafe(s)
% Wrap in quotes if the string contains a comma or double-quote.
    s = strrep(s, '"', '""');          % escape any embedded quotes
    if contains(s, ',') || contains(s, '"')
        s = ['"', s, '"'];
    end
end