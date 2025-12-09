#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <ctime>
#include <cmath>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/stat.h>
#include <dirent.h>
#endif

// Mock Arduino String class for compatibility
class String {
private:
    std::string str;
public:
    String(const char* s = "") : str(s) {}
    String(int val) : str(std::to_string(val)) {}
    String(float val) : str(std::to_string(val)) {}
    const char* c_str() const { return str.c_str(); }
    size_t length() const { return str.length(); }
    String operator+(const String& other) const {
        String result;
        result.str = str + other.str;
        return result;
    }
};

// Include headers
#include "SystolicDetector.h"
#include "MAPDetector.h"
#include "BPMonitor.h"
#include "filters.h"

// CSV Row structure
struct CSVRow {
    unsigned long time;
    float pressure;
    int ppgSignal;
    int rawPPGSignal;
    bool hasRawPPG;
    int ausPulseHeard;
};

// CSV Data structure with column tracking
struct CSVData {
    std::vector<std::string> headers;
    std::vector<CSVRow> rows;
    bool hasRawPPG;
    bool hasAusPulseHeard;
    int timeColIdx;
    int pressureColIdx;
    int ppgColIdx;
    int rawPPGColIdx;
    int ausPulseHeardIdx;
};

// Run data structure - stores data for one complete run
struct RunData {
    int runNumber;
    std::vector<CSVRow> rows;
    unsigned long startTime;
    unsigned long endTime;
    std::vector<std::pair<CSVRow, int>> detectorOutputs;  // Store row and ppgSignal for each sample
};

// Helper functions for path manipulation
std::string getBasename(const std::string& path) {
    size_t lastSlash = path.find_last_of("/\\");
    std::string filename = (lastSlash != std::string::npos) ? path.substr(lastSlash + 1) : path;
    size_t lastDot = filename.find_last_of('.');
    if (lastDot != std::string::npos) {
        return filename.substr(0, lastDot);
    }
    return filename;
}

std::string getFilename(const std::string& path) {
    size_t pos = path.find_last_of("/\\");
    if (pos == std::string::npos) return path;
    return path.substr(pos + 1);
}

std::string getDirectory(const std::string& path) {
    size_t pos = path.find_last_of("/\\");
    if (pos == std::string::npos) return ".";
    return path.substr(0, pos);
}

std::string joinPath(const std::string& dir, const std::string& file) {
    if (dir.empty()) return file;
    if (dir.back() == '/' || dir.back() == '\\') return dir + file;
#ifdef _WIN32
    return dir + "\\" + file;
#else
    return dir + "/" + file;
#endif
}

bool endsWith(const std::string& str, const std::string& suffix) {
    if (str.length() < suffix.length()) return false;
    return str.compare(str.length() - suffix.length(), suffix.length(), suffix) == 0;
}

std::string getTimestamp() {
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
#ifdef _WIN32
    localtime_s(&tstruct, &now);
#else
    localtime_r(&now, &tstruct);
#endif
    strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tstruct);
    return std::string(buf);
}

std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, (last - first + 1));
}

std::vector<std::string> parseCSVLine(const std::string& line) {
    std::vector<std::string> result;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        result.push_back(trim(cell));
    }
    return result;
}

int findColumnIndex(const std::vector<std::string>& headers, const std::vector<std::string>& possibleNames) {
    for (const auto& name : possibleNames) {
        for (size_t i = 0; i < headers.size(); i++) {
            if (headers[i] == name) {
                return i;
            }
        }
    }
    return -1;
}

CSVData loadCSV(const std::string& filename) {
    CSVData data;
    data.hasRawPPG = false;
    data.hasAusPulseHeard = false;
    data.timeColIdx = -1;
    data.pressureColIdx = -1;
    data.ppgColIdx = -1;
    data.rawPPGColIdx = -1;
    data.ausPulseHeardIdx = -1;

    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return data;
    }

    if (!std::getline(file, line)) {
        std::cerr << "Error: Empty file" << std::endl;
        return data;
    }

    data.headers = parseCSVLine(line);

    data.timeColIdx = findColumnIndex(data.headers, {"Time", "Timestamp"});
    data.pressureColIdx = findColumnIndex(data.headers, {"Pressure"});
    data.ppgColIdx = findColumnIndex(data.headers, {"PPGSignal"});
    data.rawPPGColIdx = findColumnIndex(data.headers, {"rawPPGSignal", "PPG"});
    data.ausPulseHeardIdx = findColumnIndex(data.headers, {"AUS_PULSE_HEARD"});

    if (data.timeColIdx == -1 || data.pressureColIdx == -1) {
        std::cerr << "Error: Required columns not found!" << std::endl;
        return data;
    }

    data.hasRawPPG = (data.rawPPGColIdx != -1);
    data.hasAusPulseHeard = (data.ausPulseHeardIdx != -1);

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::vector<std::string> cells = parseCSVLine(line);
        if (cells.size() < data.headers.size()) continue;

        try {
            CSVRow row;
            row.time = std::stoul(cells[data.timeColIdx]);
            row.pressure = std::stof(cells[data.pressureColIdx]);
            row.ppgSignal = (data.ppgColIdx != -1) ? std::stoi(cells[data.ppgColIdx]) : 0;
            row.rawPPGSignal = data.hasRawPPG ? std::stoi(cells[data.rawPPGColIdx]) : 0;
            row.ausPulseHeard = data.hasAusPulseHeard ? std::stoi(cells[data.ausPulseHeardIdx]) : 0;
            row.hasRawPPG = data.hasRawPPG;
            data.rows.push_back(row);
        } catch (...) {
            continue;
        }
    }

    return data;
}

bool isDirectory(const std::string& path) {
#ifdef _WIN32
    DWORD attrib = GetFileAttributesA(path.c_str());
    return (attrib != INVALID_FILE_ATTRIBUTES && (attrib & FILE_ATTRIBUTE_DIRECTORY));
#else
    struct stat st;
    if (stat(path.c_str(), &st) == 0) {
        return S_ISDIR(st.st_mode);
    }
    return false;
#endif
}

void createDirectory(const std::string& path) {
#ifdef _WIN32
    CreateDirectoryA(path.c_str(), NULL);
#else
    mkdir(path.c_str(), 0755);
#endif
}

void createDirectories(const std::string& path) {
    std::string current;
    for (size_t i = 0; i < path.length(); i++) {
        if (path[i] == '/' || path[i] == '\\') {
            if (!current.empty() && current != ".") {
                createDirectory(current);
            }
        }
        current += path[i];
    }
    if (!current.empty()) {
        createDirectory(current);
    }
}

std::string getRelativePath(const std::string& fullPath, const std::string& basePath) {
    if (fullPath.find(basePath) == 0) {
        size_t start = basePath.length();
        while (start < fullPath.length() && (fullPath[start] == '/' || fullPath[start] == '\\')) {
            start++;
        }
        return fullPath.substr(start);
    }
    return getFilename(fullPath);
}

void findCSVFilesRecursive(const std::string& directory, std::vector<std::string>& csvFiles) {
#ifdef _WIN32
    WIN32_FIND_DATAA findData;
    HANDLE hFind = FindFirstFileA((directory + "\\*").c_str(), &findData);
    if (hFind == INVALID_HANDLE_VALUE) return;

    do {
        std::string filename = findData.cFileName;
        if (filename == "." || filename == "..") continue;
        std::string fullPath = joinPath(directory, filename);

        if (findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
            findCSVFilesRecursive(fullPath, csvFiles);
        } else if (endsWith(filename, ".csv")) {
            csvFiles.push_back(fullPath);
        }
    } while (FindNextFileA(hFind, &findData));
    FindClose(hFind);
#else
    DIR* dir = opendir(directory.c_str());
    if (!dir) return;

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string filename = entry->d_name;
        if (filename == "." || filename == "..") continue;
        std::string fullPath = joinPath(directory, filename);

        if (isDirectory(fullPath)) {
            findCSVFilesRecursive(fullPath, csvFiles);
        } else if (endsWith(filename, ".csv")) {
            csvFiles.push_back(fullPath);
        }
    }
    closedir(dir);
#endif
}

std::vector<std::string> findAllCSVFiles(const std::string& directory) {
    std::vector<std::string> csvFiles;
    findCSVFilesRecursive(directory, csvFiles);
    std::sort(csvFiles.begin(), csvFiles.end());
    return csvFiles;
}

// Split data into runs based on state machine transitions to COMPLETE
std::vector<RunData> splitIntoRuns(const CSVData& data, BPMonitor& monitor) {
    std::vector<RunData> runs;
    RunData currentRun;
    currentRun.runNumber = 1;
    currentRun.startTime = 0;
    
    BPState lastState = IDLE;
    bool inRun = false;
    
    // Create filter for processing
    PPGBandpassFilter filter(50.0f);
    monitor.setFilter(&filter);
    monitor.reset();
    
    for (const auto& row : data.rows) {
        BPMeasurement measurement;
        measurement.pressure = row.pressure;
        measurement.timestamp = row.time;
        
        if (data.hasRawPPG) {
            float filtered = filter.filter((float)row.rawPPGSignal);
            measurement.ppgSignal = (int)filtered;
            measurement.rawPPGSignal = row.rawPPGSignal;
        } else {
            measurement.ppgSignal = row.ppgSignal;
            measurement.rawPPGSignal = 0;
        }
        
        monitor.update(measurement);
        BPState currentState = monitor.getState();
        
        // Detect start of run (transition from IDLE to INFLATING)
        if (lastState == IDLE && currentState == INFLATING) {
            inRun = true;
            currentRun.startTime = row.time;
            currentRun.rows.clear();
        }
        
        // Add row to current run if we're in one
        if (inRun) {
            currentRun.rows.push_back(row);
        }
        
        // Detect end of run (transition to COMPLETE)
        if (inRun && currentState == COMPLETE && lastState != COMPLETE) {
            currentRun.endTime = row.time;
            runs.push_back(currentRun);
            
            // Prepare for next run
            currentRun.runNumber++;
            currentRun.rows.clear();
            inRun = false;
            monitor.reset();
        }
        
        lastState = currentState;
    }
    
    // If we're still in a run at the end, save it
    if (inRun && !currentRun.rows.empty()) {
        currentRun.endTime = data.rows.back().time;
        runs.push_back(currentRun);
    }
    
    return runs;
}

// Print detailed results for one run
void printRunResults(BPMonitor& monitor, const std::string& filename, int runNumber) {
    std::cout << "\n========== " << filename << " - Run #" << runNumber << " ==========\n";

    float baselineBPM = monitor.getBaselineBPM();
    if (baselineBPM > 0) {
        HeartRateRange hr = monitor.getBaselineHeartRate();
        std::cout << "Baseline HR: " << std::fixed << std::setprecision(1) << baselineBPM << " BPM\n";
        std::cout << "Valid interval: " << hr.minInterval << "-" << hr.maxInterval << " ms\n";
    } else {
        std::cout << "No baseline HR detected\n";
    }

    BPResult ensembleResult = monitor.getEnsembleResult();
    std::cout << "\n*** ENSEMBLE RESULT ***\n";
    std::cout << "Systolic: " << std::fixed << std::setprecision(0) << ensembleResult.systolic << " mmHg\n";
    std::cout << "Confidence: " << std::setprecision(3) << ensembleResult.confidence << "\n";
    std::cout << "95% CI: [" << std::setprecision(0) << ensembleResult.confidenceIntervalLow 
              << " - " << ensembleResult.confidenceIntervalHigh << "] mmHg\n";
    std::cout << "Agreement: " << ensembleResult.agreementCount << "/" 
              << ensembleResult.totalDetectors << " detectors\n";

    if (monitor.hasValidMAPData()) {
        float map = monitor.getMAP();
        std::cout << "MAP: " << map << " mmHg\n";
        if (ensembleResult.systolic > 0 && map > 0) {
            float diastolic = map - (ensembleResult.systolic - map) / 3.0f;
            std::cout << "Est. Diastolic: " << diastolic << " mmHg\n";
        }
    }
}

// Output CSV for one run
void outputRunCSV(const CSVData& originalData, BPMonitor& monitor, 
                  const std::string& outputFile, const std::vector<std::pair<CSVRow, int>>& detectorOutputs) {
    std::ofstream out(outputFile);
    
    // Header
    for (size_t i = 0; i < originalData.headers.size(); i++) {
        out << originalData.headers[i];
        if (i < originalData.headers.size() - 1) out << ",";
    }
    for (int i = 0; i < monitor.getDetectorCount(); i++) {
        out << "," << monitor.getDetector(i)->getName();
    }
    out << "\n";
    
    // Output rows with detector results
    for (const auto& entry : detectorOutputs) {
        const CSVRow& row = entry.first;
        int ppgSignal = entry.second;
        
        // Output original columns
        out << row.time << "," << std::fixed << std::setprecision(2) << row.pressure 
            << "," << ppgSignal;
        
        if (originalData.hasRawPPG) {
            out << "," << row.rawPPGSignal;
        }
        if (originalData.hasAusPulseHeard) {
            out << "," << row.ausPulseHeard;
        }
        
        // Output detector results - these are captured at the time of processing
        for (int i = 0; i < monitor.getDetectorCount(); i++) {
            DetectionRecord best = monitor.getDetector(i)->getBestDetection();
            if (row.time >= best.timestamp) {
                out << "," << std::fixed << std::setprecision(0) << best.pressure;
            } else {
                out << ",0";
            }
        }
        out << "\n";
    }
}

// Output summary report for one run
void outputRunReport(BPMonitor& monitor, const std::string& outputFile, int runNumber) {
    std::ofstream out(outputFile);
    
    out << "=== Blood Pressure Measurement Report - Run #" << runNumber << " ===\n\n";
    
    float baselineBPM = monitor.getBaselineBPM();
    if (baselineBPM > 0) {
        HeartRateRange hr = monitor.getBaselineHeartRate();
        out << "Baseline HR: " << std::fixed << std::setprecision(1) << baselineBPM << " BPM\n";
        out << "Valid interval: " << hr.minInterval << "-" << hr.maxInterval << " ms\n\n";
    } else {
        out << "No baseline HR detected\n\n";
    }
    
    BPResult ensembleResult = monitor.getEnsembleResult();
    out << "*** ENSEMBLE RESULT ***\n";
    out << "Systolic: " << std::fixed << std::setprecision(0) << ensembleResult.systolic << " mmHg\n";
    out << "Confidence: " << std::setprecision(3) << ensembleResult.confidence << "\n";
    out << "95% CI: [" << std::setprecision(0) << ensembleResult.confidenceIntervalLow 
        << " - " << ensembleResult.confidenceIntervalHigh << "] mmHg\n";
    out << "Agreement: " << ensembleResult.agreementCount << "/" 
        << ensembleResult.totalDetectors << " detectors\n\n";
    
    if (monitor.hasValidMAPData()) {
        float map = monitor.getMAP();
        out << "MAP: " << map << " mmHg\n";
        if (ensembleResult.systolic > 0 && map > 0) {
            float diastolic = map - (ensembleResult.systolic - map) / 3.0f;
            out << "Est. Diastolic: " << diastolic << " mmHg\n";
        }
    }
    
    out << "\n--- Top Detectors ---\n\n";
    out << std::left << std::setw(35) << "Detector" << std::setw(12) << "Best (mmHg)" 
        << std::setw(12) << "Confidence" << "\n";
    out << std::string(60, '-') << "\n";
    
    struct DetectorResult {
        std::string name;
        DetectionRecord best;
    };
    
    std::vector<DetectorResult> results;
    for (int i = 0; i < monitor.getDetectorCount(); i++) {
        DetectorResult result;
        result.name = monitor.getDetector(i)->getName();
        result.best = monitor.getDetector(i)->getBestDetection();
        results.push_back(result);
    }
    
    std::sort(results.begin(), results.end(), 
              [](const DetectorResult& a, const DetectorResult& b) {
                  return a.best.confidence > b.best.confidence;
              });
    
    int count = 0;
    for (const auto& result : results) {
        if (count >= 10) break;
        if (result.best.confidence > 0) {
            out << std::left << std::setw(35) << result.name 
                << std::setw(12) << std::fixed << std::setprecision(0) << result.best.pressure 
                << std::setprecision(3) << result.best.confidence << "\n";
            count++;
        }
    }
}

// Process one file with multiple runs
void processFile(const std::string& inputFile, const std::string& outputDir, 
                const std::string& inputBaseDir, BPMonitor& monitor) {
    std::cout << "\n========================================\n";
    std::cout << "Processing: " << inputFile << "\n";
    
    CSVData data = loadCSV(inputFile);
    if (data.rows.empty()) {
        std::cerr << "Error: No data loaded\n";
        return;
    }
    
    std::cout << "Loaded " << data.rows.size() << " rows\n";
    
    // Setup output directory
    std::string relativePath = getRelativePath(inputFile, inputBaseDir);
    std::string relativeDir = getDirectory(relativePath);
    std::string baseName = getBasename(inputFile);
    
    std::string fileOutputDir = outputDir;
    if (!relativeDir.empty() && relativeDir != ".") {
        fileOutputDir = joinPath(outputDir, relativeDir);
        createDirectories(fileOutputDir);
    }
    
    // Create filter
    PPGBandpassFilter filter(50.0f);
    monitor.setFilter(&filter);
    monitor.reset();
    
    int runNumber = 1;
    std::vector<std::pair<CSVRow, int>> currentRunOutput;  // Store rows and ppg for CSV output
    
    // Process all rows, detecting and handling run completions
    for (const auto& row : data.rows) {
        BPMeasurement measurement;
        measurement.pressure = row.pressure;
        measurement.timestamp = row.time;
        
        if (data.hasRawPPG) {
            float filtered = filter.filter((float)row.rawPPGSignal);
            measurement.ppgSignal = (int)filtered;
            measurement.rawPPGSignal = row.rawPPGSignal;
        } else {
            measurement.ppgSignal = row.ppgSignal;
            measurement.rawPPGSignal = 0;
        }
        
        monitor.update(measurement);
        BPState currentState = monitor.getState();
        
        // Store row and processed PPG signal for current run
        currentRunOutput.push_back({row, measurement.ppgSignal});
        
        // Check if we just transitioned to COMPLETE
        if (currentState == COMPLETE) {
            // Print results for this run
            printRunResults(monitor, getFilename(inputFile), runNumber);
            
            // Generate output files for this run
            std::string runSuffix = "_run" + std::to_string(runNumber);
            std::string csvOutput = joinPath(fileOutputDir, baseName + runSuffix + "_results.csv");
            std::string reportOutput = joinPath(fileOutputDir, baseName + runSuffix + "_report.txt");
            
            outputRunCSV(data, monitor, csvOutput, currentRunOutput);
            outputRunReport(monitor, reportOutput, runNumber);
            
            std::cout << "  Run #" << runNumber << " outputs:\n";
            std::cout << "    CSV: " << csvOutput << "\n";
            std::cout << "    Report: " << reportOutput << "\n";
            
            // Reset for next run
            runNumber++;
            monitor.reset();
            filter.reset();
            monitor.setFilter(&filter);
            currentRunOutput.clear();
        }
    }
    
    // Handle case where we're still in a run at end of file
    if (monitor.getState() != IDLE && !currentRunOutput.empty()) {
        std::cout << "\n  Note: Run #" << runNumber << " incomplete (file ended mid-measurement)\n";
    }
    
    if (runNumber == 1) {
        std::cout << "No complete runs detected in this file\n";
    } else {
        std::cout << "Detected " << (runNumber - 1) << " complete measurement run(s)\n";
    }
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input_csv_or_folder> [output_folder]\n";
        return 1;
    }

    std::string inputPath = argv[1];
    std::string outputDir = (argc >= 3) ? argv[2] : ".";
    outputDir.erase(std::remove(outputDir.begin(), outputDir.end(), ' '), outputDir.end());

    std::cout << "========== BP Detector Multi-Run CSV Processor ==========\n";

    BPMonitor monitor;
    std::vector<SystolicDetector*> allocatedDetectors;

    // Add detectors
    int windows[] = {20, 30, 40, 50};
    float thresholds[] = {2.0, 2.5, 3.0, 3.5};
    int holds[] = {5, 10};
    
    for (int w : windows) {
        for (float t : thresholds) {
            for (int h : holds) {
                auto* det = new BaselineDetector(w, t, h);
                allocatedDetectors.push_back(det);
                monitor.addDetector(det);
            }
        }
    }

    std::cout << "Using " << monitor.getDetectorCount() << " detectors\n";

    // Get files
    std::vector<std::string> filesToProcess;
    if (isDirectory(inputPath)) {
        filesToProcess = findAllCSVFiles(inputPath);
        std::cout << "Found " << filesToProcess.size() << " CSV files\n";
    } else {
        filesToProcess.push_back(inputPath);
    }

    if (filesToProcess.empty()) {
        std::cerr << "No CSV files found!\n";
        return 1;
    }

    createDirectory(outputDir);

    // Process all files
    int totalRuns = 0;
    for (const auto& file : filesToProcess) {
        processFile(file, outputDir, inputPath, monitor);
    }

    std::cout << "\n========================================\n";
    std::cout << "Complete! Processed " << filesToProcess.size() << " file(s)\n";
    std::cout << "========================================\n";

    // Cleanup
    for (auto* det : allocatedDetectors) {
        delete det;
    }

    return 0;
}