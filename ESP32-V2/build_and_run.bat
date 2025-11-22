@echo off
echo Compiling CSV tester...
g++ -std=c++11 -O2 -I./src csv_test_main.cpp src/PulseDetector.cpp src/BPMonitor.cpp -o bp_detector_test.exe
if %errorlevel% neq 0 (
    echo Compilation failed!
    pause
    exit /b %errorlevel%
)

echo Running test on Harleenthumblowreadings11-19.csv...
bp_detector_test.exe Harleenthumblowreadings11-19.csv

pause