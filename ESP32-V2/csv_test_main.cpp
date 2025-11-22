#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>

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

// CSV Row structure
struct CSVRow {
    unsigned long time;
    float pressure;
    int ppgSignal;
    int rawPPGSignal;
};

// Parse CSV file
std::vector<CSVRow> loadCSV(const std::string& filename) {
    std::vector<CSVRow> data;
    std::ifstream file(filename);
    std::string line;
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return data;
    }
    
    // Skip header
    std::getline(file, line);
    
    while (std::getline(file, line)) {
        // Skip comment lines and empty lines
        if (line.empty() || line[0] == '#') continue;
        
        std::stringstream ss(line);
        std::string token;
        CSVRow row;
        
        // Parse: Time,Pressure,PPGSignal,rawPPGSignal,...
        try {
            std::getline(ss, token, ',');
            row.time = std::stoul(token);
            
            std::getline(ss, token, ',');
            row.pressure = std::stof(token);
            
            std::getline(ss, token, ',');
            row.ppgSignal = std::stoi(token);
            
            std::getline(ss, token, ',');
            row.rawPPGSignal = std::stoi(token);
            
            // Ignore remaining columns
            data.push_back(row);
        } catch (...) {
            // Skip malformed rows
            continue;
        }
    }
    
    return data;
}

// Print results table
void printResults(BPMonitor& monitor) {
    std::cout << "\n========== Detection Results ==========\n";
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
                       const std::string& outputFile) {
    std::ofstream out(outputFile);
    
    // Header
    out << "Time,Pressure,PPGSignal,rawPPGSignal";
    for (int i = 0; i < monitor.getDetectorCount(); i++) {
        out << "," << monitor.getDetector(i)->getName();
    }
    out << "\n";
    
    // Reset monitor and all detectors
    monitor.reset();
    
    // Process each row
    for (const auto& row : data) {
        BPMeasurement measurement;
        measurement.pressure = row.pressure;
        measurement.ppgSignal = row.ppgSignal;
        measurement.rawPPGSignal = row.rawPPGSignal;
        measurement.timestamp = row.time;
        
        // Update monitor (runs all detectors)
        monitor.update(measurement);
        
        // Output row
        out << row.time << ","
            << std::fixed << std::setprecision(2) << row.pressure << ","
            << row.ppgSignal << ","
            << row.rawPPGSignal;
        
        // Output detector results
        for (int i = 0; i < monitor.getDetectorCount(); i++) {
            out << "," << monitor.getDetector(i)->getSystolic();
        }
        out << "\n";
    }
    
    std::cout << "Detailed results written to: " << outputFile << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <csv_file> [output_file]\n";
        return 1;
    }
    
    std::string inputFile = argv[1];
    std::string outputFile = (argc >= 3) ? argv[2] : "detector_results.csv";
    
    std::cout << "========== BP Detector CSV Test ==========\n";
    std::cout << "Loading CSV: " << inputFile << "...";
    
    // Load data
    std::vector<CSVRow> data = loadCSV(inputFile);
    if (data.empty()) {
        std::cerr << "\nError: No data loaded from CSV file.\n";
        return 1;
    }
    std::cout << " Done! (" << data.size() << " rows)\n";
    
    // Create BPMonitor
    BPMonitor monitor;
    
    // Add detectors to test
    BaselineDetector det1(20, 2.5, 5, 2);
    BaselineDetector det2(40, 2.5, 5, 2);
    BaselineDetector det3(60, 2.5, 5, 2);
    BaselineDetector det4(10, 2.5, 5, 2);
    BaselineDetector det5(20, 1.0, 5, 2);
    BaselineDetector det6(20, 1.5, 5, 1);
    BaselineDetector det7(10, 1.0, 5, 1);
    BaselineDetector det8(20, 2.5, 5, 1);
    
    DerivativeDetector det11(5, 20);
    DerivativeDetector det12(10, 20);
    DerivativeDetector det13(5, 30);
    DerivativeDetector det14(5, 1);
    
    monitor.addDetector(&det1);
    monitor.addDetector(&det2);
    monitor.addDetector(&det3);
    monitor.addDetector(&det4);
    monitor.addDetector(&det5);
    monitor.addDetector(&det6);
    monitor.addDetector(&det7);
    monitor.addDetector(&det8);
    monitor.addDetector(&det11);
    monitor.addDetector(&det12);
    monitor.addDetector(&det13);
    monitor.addDetector(&det14);
    
    std::cout << "Testing " << monitor.getDetectorCount() << " detectors...\n";
    
    // Run detectors on data using BPMonitor
    for (const auto& row : data) {
        BPMeasurement measurement;
        measurement.pressure = row.pressure;
        measurement.ppgSignal = row.ppgSignal;
        measurement.rawPPGSignal = row.rawPPGSignal;
        measurement.timestamp = row.time;
        
        monitor.update(measurement);
    }
    
    // Print results
    printResults(monitor);
    
    // Output detailed CSV (resets and reruns)
    outputDetailedCSV(data, monitor, outputFile);
    
    std::cout << "\n========== Test Complete ==========\n";
    
    return 0;
}