#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>
#include <ctime>

#ifdef _WIN32
    #include <windows.h>
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

// Include only headers from src/ directory
#include "PulseDetector.h"
#include "BPMonitor.h"
#include "filters.h"

// CSV Row structure
struct CSVRow {
    unsigned long time;
    float pressure;
    int ppgSignal;
    int rawPPGSignal;
    bool hasRawPPG;
};

// Simple helper to get base filename
std::string getBasename(const std::string& path) {
    size_t lastSlash = path.find_last_of("/\\");
    std::string filename = (lastSlash != std::string::npos) ? path.substr(lastSlash + 1) : path;
    size_t lastDot = filename.find_last_of('.');
    if (lastDot != std::string::npos) {
        return filename.substr(0, lastDot);
    }
    return filename;
}

// Get timestamp for filename
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

// Parse CSV file header to check for rawPPGSignal
bool hasRawPPGColumn(const std::string& headerLine) {
    return (headerLine.find("rawPPGSignal") != std::string::npos || 
            headerLine.find("RawPPGSignal") != std::string::npos);
}

// Parse CSV file
std::vector<CSVRow> loadCSV(const std::string& filename, bool& hasRaw) {
    std::vector<CSVRow> data;
    std::ifstream file(filename);
    std::string line;
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return data;
    }
    
    // Check header for rawPPGSignal
    std::getline(file, line);
    hasRaw = hasRawPPGColumn(line);
    
    while (std::getline(file, line)) {
        // Skip comment lines and empty lines
        if (line.empty() || line[0] == '#') continue;
        
        std::stringstream ss(line);
        std::string token;
        CSVRow row;
        row.hasRawPPG = hasRaw;
        
        // Parse: Time,Pressure,PPGSignal,rawPPGSignal,...
        try {
            std::getline(ss, token, ',');
            row.time = std::stoul(token);
            
            std::getline(ss, token, ',');
            row.pressure = std::stof(token);
            
            std::getline(ss, token, ',');
            row.ppgSignal = std::stoi(token);
            
            if (hasRaw) {
                std::getline(ss, token, ',');
                row.rawPPGSignal = std::stoi(token);
            } else {
                row.rawPPGSignal = 0;
            }
            
            // Ignore remaining columns
            data.push_back(row);
        } catch (...) {
            // Skip malformed rows
            continue;
        }
    }
    
    return data;
}

// Get list of CSV files in directory (Windows only for now)
std::vector<std::string> getCSVFiles(const std::string& path) {
    std::vector<std::string> files;
    
#ifdef _WIN32
    WIN32_FIND_DATAA findData;
    std::string searchPath = path + "\\*.csv";
    
    HANDLE hFind = FindFirstFileA(searchPath.c_str(), &findData);
    if (hFind != INVALID_HANDLE_VALUE) {
        do {
            if (!(findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
                files.push_back(path + "\\" + findData.cFileName);
            }
        } while (FindNextFileA(hFind, &findData));
        FindClose(hFind);
    }
#else
    // On Linux/Mac, just process single file
    files.push_back(path);
#endif
    
    return files;
}

// Check if path is a directory
bool isDirectory(const std::string& path) {
#ifdef _WIN32
    DWORD attrib = GetFileAttributesA(path.c_str());
    return (attrib != INVALID_FILE_ATTRIBUTES && (attrib & FILE_ATTRIBUTE_DIRECTORY));
#else
    return false;  // For simplicity on non-Windows
#endif
}

// Print results table
void printResults(BPMonitor& monitor, const std::string& filename) {
    std::cout << "\n========== " << filename << " ==========\n";
    std::cout << std::left << std::setw(30) << "Detector" 
              << std::setw(15) << "Systolic (mmHg)" << std::endl;
    std::cout << std::string(45, '-') << std::endl;
    
    for (int i = 0; i < monitor.getDetectorCount(); i++) {
        PulseDetector* detector = monitor.getDetector(i);
        std::cout << std::left << std::setw(30) << detector->getName()
                  << std::setw(15) << detector->getSystolic() << std::endl;
    }
}

// Output detailed CSV with all detector results
void outputDetailedCSV(const std::vector<CSVRow>& data,
                       BPMonitor& monitor,
                       const std::string& outputFile,
                       bool hasRawPPG) {
    std::ofstream out(outputFile);
    
    // Header
    out << "Time,Pressure,PPGSignal";
    if (hasRawPPG) {
        out << ",rawPPGSignal";
    }
    for (int i = 0; i < monitor.getDetectorCount(); i++) {
        out << "," << monitor.getDetector(i)->getName();
    }
    out << "\n";
    
    // Reset monitor and all detectors
    monitor.reset();
    
    // Create filter if using raw PPG
    PPGBandpassFilter filter;
    
    // Process each row
    for (const auto& row : data) {
        BPMeasurement measurement;
        measurement.pressure = row.pressure;
        measurement.timestamp = row.time;
        
        // Use raw PPG if available (filter it), otherwise use PPGSignal as-is
        if (hasRawPPG) {
            float filtered = filter.filter((float)row.rawPPGSignal);
            measurement.ppgSignal = (int)filtered;
            measurement.rawPPGSignal = row.rawPPGSignal;
        } else {
            measurement.ppgSignal = row.ppgSignal;
            measurement.rawPPGSignal = 0;
        }
        
        // Update monitor (runs all detectors)
        monitor.update(measurement);
        
        // Output row
        out << row.time << ","
            << std::fixed << std::setprecision(2) << row.pressure << ","
            << measurement.ppgSignal;
        
        if (hasRawPPG) {
            out << "," << row.rawPPGSignal;
        }
        
        // Output detector results
        for (int i = 0; i < monitor.getDetectorCount(); i++) {
            out << "," << monitor.getDetector(i)->getSystolic();
        }
        out << "\n";
    }
    
    std::cout << "Results written to: " << outputFile << std::endl;
}

// Process a single file
void processFile(const std::string& inputFile, const std::string& outputDir, BPMonitor& monitor) {
    std::cout << "\n========================================\n";
    std::cout << "Processing: " << inputFile << "\n";
    
    bool hasRawPPG = false;
    std::vector<CSVRow> data = loadCSV(inputFile, hasRawPPG);
    
    if (data.empty()) {
        std::cerr << "Error: No data loaded\n";
        return;
    }
    
    std::cout << "Loaded " << data.size() << " rows\n";
    if (hasRawPPG) {
        std::cout << "Note: Using rawPPGSignal (will filter fresh)\n";
    } else {
        std::cout << "Note: Using PPGSignal as-is (no rawPPGSignal found)\n";
    }
    
    // Run detectors
    monitor.reset();
    PPGBandpassFilter filter;
    
    for (const auto& row : data) {
        BPMeasurement measurement;
        measurement.pressure = row.pressure;
        measurement.timestamp = row.time;
        
        if (hasRawPPG) {
            float filtered = filter.filter((float)row.rawPPGSignal);
            measurement.ppgSignal = (int)filtered;
            measurement.rawPPGSignal = row.rawPPGSignal;
        } else {
            measurement.ppgSignal = row.ppgSignal;
            measurement.rawPPGSignal = 0;
        }
        
        monitor.update(measurement);
    }
    
    // Get filename for display
    size_t lastSlash = inputFile.find_last_of("/\\");
    std::string filename = (lastSlash != std::string::npos) ? inputFile.substr(lastSlash + 1) : inputFile;
    
    // Print results
    printResults(monitor, filename);
    
    // Create output filename with timestamp
    std::string baseName = getBasename(inputFile);
    std::string timestamp = getTimestamp();
    std::string outputFile = outputDir + "\\" + baseName + "_results_" + timestamp + ".csv";
    
    // Output detailed CSV
    outputDetailedCSV(data, monitor, outputFile, hasRawPPG);
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <csv_file_or_folder> [output_folder]\n";
        std::cerr << "\nExamples:\n";
        std::cerr << "  " << argv[0] << " data.csv\n";
        std::cerr << "  " << argv[0] << " data_folder\n";
        std::cerr << "  " << argv[0] << " data_folder results_folder\n";
        return 1;
    }
    
    std::string inputPath = argv[1];
    std::string outputDir = (argc >= 3) ? argv[2] : ".";
    
    std::cout << "========== BP Detector CSV Test ==========\n";
    
    // Create BPMonitor
    BPMonitor monitor;
    
    // Store dynamically allocated detectors for cleanup
    std::vector<PulseDetector*> allocatedDetectors;
    
    // Add detectors to test
    // =========================================================
    //  FIXED AUTO-GENERATED BASELINE + DERIVATIVE DETECTORS
    //  (Dynamic allocation – NO dangling pointers)
    // =========================================================

    //
    // ------------ BASELINE DETECTORS ------------
    // Format: BaselineDetector(w, t, h, m)
    //

    #define ADD_BASE(w, t, h, m) \
        { \
            auto* det = new BaselineDetector(w, t, h, m); \
            allocatedDetectors.push_back(det); \
            monitor.addDetector(det); \
        }

    {
        int windows[]      = {5, 20, 35, 50, 65, 80};
        float thresholds[] = {1.0, 1.5, 2.0, 2.5, 3.0, 4.0};
        int holds[]        = {0, 2, 5, 10};
        int modes[]        = {1, 2};

        for (int w : windows) {
            for (float t : thresholds) {
                for (int h : holds) {
                    for (int m : modes) {
                        ADD_BASE(w, t, h, m);
                    }
                }
            }
        }
    }

    //
    // ------------ DERIVATIVE DETECTORS ------------
    // Format: DerivativeDetector(w, g)
    //

    #define ADD_DERIV(w, g) \
        { \
            auto* det = new DerivativeDetector(w, g); \
            allocatedDetectors.push_back(det); \
            monitor.addDetector(det); \
        }

    {
        // Small windows (2–8), gain step 0.2
        int smallW[] = {2, 4, 6, 8};
        for (int w : smallW) {
            for (float g = 0.2f; g <= 2.0f + 1e-6f; g += 0.2f) {
                ADD_DERIV(w, g);
            }
        }

        // Medium windows (10–16), gain step 0.5
        int medW[] = {10, 12, 14, 16};
        for (int w : medW) {
            for (float g = 0.5f; g <= 2.0f + 1e-6f; g += 0.5f) {
                ADD_DERIV(w, g);
            }
        }

        // Large windows (18–20), gain step 1.0
        int largeW[] = {18, 20};
        for (int w : largeW) {
            for (float g = 1.0f; g <= 2.0f + 1e-6f; g += 1.0f) {
                ADD_DERIV(w, g);
            }
        }
    }
    
    std::cout << "Using " << monitor.getDetectorCount() << " detectors\n";
    
    // Get list of files to process
    std::vector<std::string> filesToProcess;
    
    if (isDirectory(inputPath)) {
        filesToProcess = getCSVFiles(inputPath);
        std::cout << "Found " << filesToProcess.size() << " CSV files in directory\n";
    } else {
        filesToProcess.push_back(inputPath);
    }
    
    // Process all files
    for (const auto& file : filesToProcess) {
        processFile(file, outputDir, monitor);
    }
    
    std::cout << "\n========================================\n";
    std::cout << "Processing Complete!\n";
    std::cout << "Processed " << filesToProcess.size() << " file(s)\n";
    std::cout << "========================================\n";
    
    // Cleanup
    for (auto* det : allocatedDetectors) {
        delete det;
    }
    
    return 0;
}