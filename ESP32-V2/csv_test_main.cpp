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
#include "MotorController.h"
#include <regex>

// CSV Row structure
struct CSVRow {
    unsigned long time;
    float pressure;
    int ppgSignal;
    int rawPPGSignal;
    bool hasRawPPG;
    int ausPulseHeard;
    float oscAmplitude;  
    int mapValue;      
    int oscSystolic;   
    int oscDiastolic;  
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

    std::vector<std::string> getHeaders = parseCSVLine(line);

    data.timeColIdx = findColumnIndex(getHeaders, {"Time", "Timestamp"});
    data.pressureColIdx = findColumnIndex(getHeaders, {"Pressure"});
    data.ppgColIdx = findColumnIndex(getHeaders, {"PPGSignal"});
    data.rawPPGColIdx = findColumnIndex(getHeaders, {"rawPPGSignal", "PPG"});
    data.ausPulseHeardIdx = 4;

    if (data.timeColIdx == -1 || data.pressureColIdx == -1) {
        std::cerr << "Error: Required columns not found!" << std::endl;
        return data;
    }

    std::vector<std::string> filtered;
    filtered.push_back("Time");
    filtered.push_back("Pressure");

    if (data.ppgColIdx != -1)
        filtered.push_back("PPGSignal");

    if (data.rawPPGColIdx != -1)
        filtered.push_back("rawPPGSignal");

    if (data.ausPulseHeardIdx != -1)
        filtered.push_back("AUS_PULSE_HEARD");

    data.hasRawPPG = (data.rawPPGColIdx != -1);
    data.hasAusPulseHeard = (data.ausPulseHeardIdx != -1);

    data.headers = filtered;

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
    PPGBandpassFilter filter(1000.0f / SAMPLE_RATE_MS);
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
        if (inRun && currentState == COMPLETE && measurement.pressure < BP_MIN_IDLE_PRESSURE - PRESSURE_DROP_THRESHOLD) {
            currentRun.endTime = row.time;
            runs.push_back(currentRun);
            
            // Prepare for next run
            currentRun.runNumber++;
            currentRun.rows.clear();
            inRun = false;
            monitor.reset();
        } else if (inRun && currentState == INFLATING && measurement.pressure < BP_MIN_IDLE_PRESSURE - PRESSURE_DROP_THRESHOLD) {
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

    float map = monitor.getMAP();
    std::cout << "MAP: " << map << " mmHg\n";
    if (ensembleResult.systolic > 0 && map > 0) {
        float DBP = (3.0f * map - ensembleResult.systolic) / 2.0f;
        std::cout << "Est. Diastolic: " << DBP << " mmHg\n";
    }
    std::cout << "MAP Systolic: " << monitor.getMAPDetector()->getSystolic() << " mmHg\n";
    std::cout << "MAP Diastolic: " << monitor.getMAPDetector()->getDiastolic() << " mmHg\n";
}

// Output CSV for one run - UPDATED to match Arduino format
std::pair<int,int> outputRunCSV(const CSVData& originalData, BPMonitor& monitor, 
                  const std::string& outputFile, const std::vector<CSVRow>& outputRows) {
    std::ofstream out(outputFile);
    
    // Header - Match Arduino format
    out << "Time,Pressure,rawPPGSignal,PPGSignal";
    if (originalData.hasAusPulseHeard) {
        out << ",AUS_PULSE_HEARD";
    }
    out << "\n";
    
    int baselineBeatCount = 0;
    const unsigned long* baselineBeats = monitor.getBaselineBeats(baselineBeatCount);

    int aus_sbp = -1;
    int aus_dbp = -1;

    // Output rows with detector results
    for (const auto& row : outputRows) {
        
        // Output in Arduino format: Time, Pressure, rawPPGSignal, PPGSignal, [AUS_PULSE_HEARD]
        out << row.time << "," 
            << std::fixed << std::setprecision(2) << row.pressure << ",";
        
        // rawPPGSignal (raw unfiltered)
        if (originalData.hasRawPPG) {
            out << row.rawPPGSignal;
        } else {
            out << "0";  // Default if not present
        }
        
        out << ",";
        
        // PPGSignal (filtered)
        out << row.ppgSignal;
        
        // AUS_PULSE_HEARD (optional - button press)
        if (originalData.hasAusPulseHeard) {
            if (row.ausPulseHeard == 0) {
                if (aus_sbp == -1) aus_sbp = row.pressure;
                aus_dbp = row.pressure;
                out << ",1";
            }
            else
                out << ",0";
        }

        out << "\n";
    }
    
    // ===== SUMMARY SECTION - Match Arduino output =====
    out << "#SUMMARY_START\n";
    
    // Output all detector results
    int detectorCount = monitor.getDetectorCount();
    for (int i = 0; i < detectorCount; i++) {
        SystolicDetector* det = monitor.getDetector(i);
        const char* name = det->getName();
        
        // Get top detections (up to 20)
        DetectionRecord detections[20];
        int count = 0;
        det->getTopDetections(detections, 20, &count);
        
        // Output all detections with confidence > 0
        for (int k = 0; k < count; k++) {
            if (detections[k].confidence <= 0)
                continue;
            
            out << name << ","
                << detections[k].timestamp << ","
                << std::fixed << std::setprecision(0) << detections[k].pressure << ","
                << std::setprecision(3) << detections[k].confidence << "\n";
        }
    }
    
    // Ensemble Result
    BPResult result = monitor.getEnsembleResult();
    out << "Ensemble,0,"
        << std::fixed << std::setprecision(0) << result.systolic
        << ",0\n";
    
    out << "AusSys,0,"
        << std::fixed << std::setprecision(0) << aus_sbp
        << ",0\n";

    out << "AusDia,0,"
        << std::fixed << std::setprecision(0) << aus_dbp
        << ",0\n";
    
    // Oscillometric Results
    MAPDetector* osc = monitor.getMAPDetector();
    
    out << "OscSys,0,"
        << std::fixed << std::setprecision(0) << osc->getSystolic()
        << ",0\n";
    
    out << "OscMAP,0,"
        << std::fixed << std::setprecision(0) << osc->getMAP()
        << ",0\n";
    
    out << "OscDia,0,"
        << std::fixed << std::setprecision(0) << osc->getDiastolic()
        << ",0\n";
    
    // Estimated Diastolic (from ensemble systolic + MAP)
    float map = monitor.getMAP();
    float DBP = (3.0f * map - result.systolic) / 2.0f;
    out << "EstDia,0,"
        << std::fixed << std::setprecision(0) << DBP
        << ",0\n";

    out << "BPM,0,"
        << std::fixed << std::setprecision(0) << monitor.getBaselineBPM()
        << ",0\n";
    
    out << "#SUMMARY_END\n";
    return {aus_sbp, aus_dbp};
}

bool extractSBPFromFilename(const std::string& filename, float& sbpOut) {
    size_t pos = filename.find("SBP");
    if (pos == std::string::npos) return false;

    pos += 3; // skip "SBP"
    if (pos >= filename.size()) return false;

    try {
        sbpOut = std::stof(filename.substr(pos));
        return true;
    } catch (...) {
        return false;
    }
}


// Output summary report for one run
void outputRunReport(BPMonitor& monitor, const std::string& outputFile, int runNumber, int omron_sbp, int omron_dbp, int aus_sbp, int aus_dbp) {
    std::ofstream out(outputFile);
    
    float trueSBP = 0.0f;
    bool hasTrueSBP = extractSBPFromFilename(outputFile, trueSBP);

    out << "=== Blood Pressure Measurement Report - Run #" << runNumber << " ===\n\n";
    float map = monitor.getMAP();
    BPResult ensembleResult = monitor.getEnsembleResult();
    out << "Systolic: \n";
    out << "Auscultatory Systolic   : " << std::fixed << std::setprecision(0) << aus_sbp << " mmHg (GROUND TRUTH)\n";
    out << "Ensemble Systolic       : " << std::fixed << std::setprecision(0) << ensembleResult.systolic << " mmHg\n";
    out << "Oscillometric Systolic  : " << monitor.getMAPDetector()->getSystolic() << " mmHg\n";
    if (omron_sbp != -1) 
        out << "Omron Systolic          : " << omron_sbp << " mmHg\n";

    out << "\nDiastolic:\n";

    float DBP = -1.0;
    if (ensembleResult.systolic > 0 && map > 0) {
        DBP = (3.0f * map - ensembleResult.systolic) / 2.0f;
    }

    out << "Auscultatory Diastolic  : " << std::fixed << std::setprecision(0) << aus_dbp << " mmHg (GROUND TRUTH)\n";
    out << "Ensemble Diastolic      : " << std::fixed << std::setprecision(0) << DBP << " mmHg\n";
    out << "Oscillometric Diastolic : " << monitor.getMAPDetector()->getDiastolic() << " mmHg\n";
    if (omron_sbp != -1) 
        out << "Omron Diastolic         : " << omron_dbp << " mmHg\n";

    float baselineBPM = monitor.getBaselineBPM();
    if (baselineBPM > 0) {
        HeartRateRange hr = monitor.getBaselineHeartRate();
        out << "Baseline HR: " << std::fixed << std::setprecision(1) << baselineBPM << " BPM\n";
        out << "Valid interval: " << hr.minInterval << "-" << hr.maxInterval << " ms\n\n";
    } else {
        out << "No baseline HR detected\n\n";
    }
    
    out << "*** ENSEMBLE RESULT ***\n";
    out << "Ensemble Systolic: " << std::fixed << std::setprecision(0) << ensembleResult.systolic << " mmHg\n";
    out << "Confidence: " << std::setprecision(3) << ensembleResult.confidence << "\n";
    out << "95% CI: [" << std::setprecision(0) << ensembleResult.confidenceIntervalLow 
        << " - " << ensembleResult.confidenceIntervalHigh << "] mmHg\n";
    out << "Agreement: " << ensembleResult.agreementCount << "/" 
        << ensembleResult.totalDetectors * MAX_DETECTIONS << " detections\n\n";
    
    out << "MAP: " << map << " mmHg\n";
    out << "Est. Diastolic: " << DBP << " mmHg  [MAP=(2*DBP + SBP)/3]\n";
    out << "Calculated Oscillometric SBP: " << monitor.getMAPDetector()->getSystolic() << " mmHg\n";
    out << "Calculated Oscillometric DBP: " << monitor.getMAPDetector()->getDiastolic() << " mmHg\n";
    
    if (omron_sbp != -1) {
        out << "Omron SBP: " << omron_sbp << " mmHg\n";
        out << "Omron DBP: " << omron_dbp << " mmHg\n";
    }

    out << "\n--- DETECTOR PERFORMANCE SUMMARY ---\n\n";

    out << std::left
        << std::setw(35) << "Detector"
        << std::setw(12) << "Best"
        << std::setw(12) << "Conf";

    if (hasTrueSBP) {
        out << " | "
            << std::setw(6) << "Rank"
            << std::setw(12) << "Best"
            << std::setw(12) << "Error";
    }

    out << "\n";

    out << std::string(hasTrueSBP ? 95 : 60, '-') << "\n";

    
    struct DetectorResult {
        std::string name;
        DetectionRecord best;
        int detectorIndex;

        float error;        // |best.pressure - SBP|
        bool hasError;
    };
    
    // Collect all detector results
    int detectorCount = monitor.getDetectorCount();
    DetectorResult* results = new DetectorResult[detectorCount];
    
    for (int i = 0; i < detectorCount; i++) {
        results[i].name = monitor.getDetector(i)->getName();
        results[i].best = monitor.getDetector(i)->getBestDetection();
        results[i].detectorIndex = i;

        if (hasTrueSBP && results[i].best.confidence > 0) {
            results[i].error = std::fabs(results[i].best.pressure - trueSBP);
            results[i].hasError = true;
        } else {
            results[i].error = 0;
            results[i].hasError = false;
        }
    }

    std::vector<int> byConfidence(detectorCount);
    std::vector<int> byAccuracy;

    for (int i = 0; i < detectorCount; i++) {
        byConfidence[i] = i;
        if (results[i].hasError)
            byAccuracy.push_back(i);
    }
    std::sort(byConfidence.begin(), byConfidence.end(),
        [&](int a, int b) {
            return results[a].best.confidence > results[b].best.confidence;
        });
    std::sort(byAccuracy.begin(), byAccuracy.end(),
        [&](int a, int b) {
            return results[a].error < results[b].error;
        });
    
    // Print top 10 detectors
    for (int i = 0; i < detectorCount; i++) {
        int ci = byConfidence[i];
        const auto& c = results[ci];

        out << std::left
            << std::setw(35) << c.name
            << std::setw(12) << std::fixed << std::setprecision(0) << c.best.pressure
            << std::setw(12) << std::setprecision(3) << c.best.confidence;

        if (hasTrueSBP) {
            // Find accuracy rank
            int accRank = -1;
            for (size_t k = 0; k < byAccuracy.size(); k++) {
                if (byAccuracy[k] == ci) {
                    accRank = static_cast<int>(k + 1);
                    break;
                }
            }

            if (accRank > 0) {
                out << " | "
                    << std::setw(6) << accRank
                    << std::setw(12) << std::fixed << std::setprecision(0) << c.best.pressure
                    << std::setw(12) << std::setprecision(1) << c.error;
            } else {
                out << " | "
                    << std::setw(6) << "-"
                    << std::setw(12) << "-"
                    << std::setw(12) << "-";
            }
        }

        out << "\n";
    }

    
    // Now output detailed detector results with top 10 readings for each
    out << "\n\n=== DETAILED DETECTOR RESULTS ===\n\n";
    
    for (int i = 0; i < detectorCount; i++) {
        if (results[i].best.confidence > 0) {
            SystolicDetector* detector = monitor.getDetector(results[i].detectorIndex);
            
            out << "Detector: " << results[i].name << "\n";
            out << "Best Detection: " << std::fixed << std::setprecision(0) << results[i].best.pressure 
                << " mmHg (confidence: " << std::setprecision(3) << results[i].best.confidence << ")\n";
            
            // Get top detections using existing method
            DetectionRecord topDetections[20]; // MAX_DETECTIONS
            int actualCount = 0;
            detector->getTopDetections(topDetections, 20, &actualCount);
            
            if (actualCount > 0) {
                // Output top 10 readings (already sorted by getTopDetections)
                out << "Top 10 readings:\n";
                out << "  " << std::left << std::setw(6) << "Rank" << std::setw(12) << "Pressure" 
                    << std::setw(12) << "Confidence" << "\n";
                out << "  " << std::string(30, '-') << "\n";
                
                int maxRank = (actualCount < 10) ? actualCount : 10;
                for (int rank = 0; rank < maxRank; rank++) {
                    if (topDetections[rank].confidence > 0) {
                        out << "  " << std::left << std::setw(6) << (rank + 1)
                            << std::setw(12) << std::fixed << std::setprecision(0) << topDetections[rank].pressure 
                            << std::setprecision(3) << topDetections[rank].confidence << "\n";
                    }
                }
            }
            out << "\n";
        }
    }
    
    delete[] results;
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
    PPGBandpassFilter filter(1000.0f / SAMPLE_RATE_MS);
    monitor.setFilter(&filter);
    monitor.reset();
    
    int runNumber = 1;
    std::vector<CSVRow> currentRunOutput;  // Store rows and ppg for CSV output
    
    BPState lastState = IDLE;

    // Process all rows, detecting and handling run completions
    ////////////////need to include this functionallity
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

        float osc_amp = monitor.getMAPDetector()->getLatestAmplitude();

        float mapP = monitor.getMAPDetector()->getMAP();
        float sysP = monitor.getMAPDetector()->getSystolic();
        float diaP = monitor.getMAPDetector()->getDiastolic();

        static float lastMAP=0, lastSys=0, lastDia=0;

        if (mapP > 0 && measurement.pressure <= mapP) lastMAP = mapP;
        if (sysP > 0 && measurement.pressure <= sysP) lastSys = sysP;
        if (diaP > 0 && measurement.pressure <= diaP) lastDia = diaP;

        CSVRow r = row; 
        r.ppgSignal    = measurement.ppgSignal;
        r.oscAmplitude = osc_amp;
        r.mapValue     = lastMAP;
        r.oscSystolic  = lastSys;
        r.oscDiastolic = lastDia;
        
        // Store row and processed PPG signal for current run
        currentRunOutput.push_back(r);
        
        // Check if we did one run on COMPLETE
        if (lastState == COMPLETE) {
            // Print results for this run
            printRunResults(monitor, getFilename(inputFile), runNumber);
            
            std::string s = baseName;

            // Remove ".csv" safely
            if (s.size() > 4 && s.substr(s.size() - 4) == ".csv") {
                s = s.substr(0, s.size() - 4);
            }

            std::vector<std::string> parts;
            std::stringstream ss(s);
            std::string item;
            while (std::getline(ss, item, '_')) {
                parts.push_back(item);
            }

            int omron_sbp = -1;
            int omron_dbp = -1;

            if (parts.size() >= 3) {
                try {
                    omron_sbp = std::stoi(parts[parts.size() - 2]); // 128
                    omron_dbp = std::stoi(parts[parts.size() - 1]); // 89
                } catch (...) {
                    omron_sbp = -1;
                    omron_dbp = -1;
                }
            }

            // Generate output files for this run
            std::string csvOutput = joinPath(fileOutputDir, baseName + "_results.csv");
            std::string reportOutput = joinPath(fileOutputDir, baseName + "_report.txt");
            std::pair<int,int> aus;
            aus = outputRunCSV(data, monitor, csvOutput, currentRunOutput);
            outputRunReport(monitor, reportOutput, runNumber, omron_sbp, omron_dbp, aus.first, aus.second);
            
            std::cout << "  Run #" << runNumber << " outputs:\n";
            std::cout << "    CSV: " << csvOutput << "\n";
            std::cout << "    Report: " << reportOutput << "\n";
            
            // Reset for next run
            break;
            runNumber++;
            monitor.reset();
            filter.reset();
            monitor.setFilter(&filter);
            currentRunOutput.clear();
        }

        lastState = monitor.getState();
    }
    
    if (runNumber == 1) {
        // std::cout << "No complete runs detected in this file\n";
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
    MotorController motor;
    MAPDetector mapDetector;

    monitor.setMAPDetector(&mapDetector);
    monitor.setMotorController(&motor);

    std::vector<SystolicDetector*> allocatedDetectors;

    // Add detectors
    // int windows[] = {50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160};
    // float thresholds[] = {1.5, 1.7, 2.0, 2.2, 2.4, 2.7, 3.0};
    // int holds[] = {10, 15, 20, 25, 30};

    // int windows[] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220};
    // // int windows[] = {50, 60, 70, 80, 90, 100, 110, 120, 130};
    // // int windows[] = {10, 20, 30, 40, 50};
    // // int windows[] = {130, 140, 150, 160, 170, 180, 190, 200, 220, 230, 250, 270, 290, 320, 350, 380, 400};
    // // int windows[] = {120, 130, 140, 150, 160, 170};
    // // // float thresholds[] = {1.8, 2.0, 2.2, 2.4};
    // float thresholds[] = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0};
    // int minDev[] = {10, 20, 30};
    // for (int w : windows) {
    //     for (float t : thresholds) {
    //         for (int d : minDev) {
    //             auto* det = new BaselineDetector(w, t, d);
    //             allocatedDetectors.push_back(det);
    //             monitor.addDetector(det);
    //         }
    //     }
    // }
    
    
    // Final detector decisions:
    // TIER ONE (BEST PERFORMING DETECTORS, 80 detectors) 
    int t1Windows[] = {60, 70, 80, 90, 100};
    float t1Thresh[] = {1.8, 1.9, 2.0, 2.2};
    int t1Dev[] = {5, 10, 12, 16};

    for (int w : t1Windows) {
        for (float t : t1Thresh) {
            for (int d : t1Dev) {
                auto* det = new BaselineDetector(w, t, d);
                allocatedDetectors.push_back(det);
                monitor.addDetector(det);
            }
        }
    }

    // TIER TWO (HIGH WINDOW DETECTORS, 40 detectors) 
    int t2Windows[] = {110, 120, 130, 140, 150};
    float t2Thresh[] = {2.0, 2.4, 2.8, 3.3};
    int t2Dev[] = {12, 18};

    for (int w : t2Windows) {
        for (float t : t2Thresh) {
            for (int d : t2Dev) {
                auto* det = new BaselineDetector(w, t, d);
                allocatedDetectors.push_back(det);
                monitor.addDetector(det);
            }
        }
    }

    // TIER THREE (LOW WINDOW DETECTORS, 24 detectors) 
    int t3Windows[] = {20, 30, 40, 50};
    float t3Thresh[] = {1.65, 1.8, 2.0};
    int t3Dev[] = {12, 15};

    for (int w : t3Windows) {
        for (float t : t3Thresh) {
            for (int d : t3Dev) {
                auto* det = new BaselineDetector(w, t, d);
                allocatedDetectors.push_back(det);
                monitor.addDetector(det);
            }
        }
    }

    
    // int drv_th[] = {4, 6, 8, 10, 12, 16, 20};

    // for (int w : drv_th) {
    //     auto* env_det = new EnvelopeSystolicDetector(w);
    //     allocatedDetectors.push_back(env_det);
    //     monitor.addDetector(env_det);

    //     auto* der_det = new DerivativeDetector(w);
    //     allocatedDetectors.push_back(der_det);
    //     monitor.addDetector(der_det);
    // }

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