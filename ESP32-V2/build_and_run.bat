@echo off
REM ============================================================
REM  Blood Pressure Detector - CSV Testing Tool (Windows)
REM ============================================================
REM
REM  USAGE:
REM    build_and_run.bat <data_file_or_folder> [output_folder]
REM
REM  EXAMPLES:
REM    build_and_run.bat Harleen.csv
REM    build_and_run.bat data_folder
REM    build_and_run.bat data_folder results_folder
REM
REM  WHAT IT DOES:
REM    1. Compiles the CSV testing tool
REM    2. Runs it on your specified CSV file(s)
REM    3. Creates timestamped output files with detector results
REM
REM  CSV REQUIREMENTS:
REM    - Must have columns: Time, Pressure, PPGSignal
REM    - Optional: rawPPGSignal (recommended - will filter fresh)
REM
REM  OUTPUT:
REM    - Console: Summary table of detector results
REM    - CSV Files: <filename>_results_<timestamp>.csv
REM
REM ============================================================

echo ========== BP Detector CSV Tester ==========
echo.

echo Compiling...
g++ -std=c++11 -O2 -I./src csv_test_main.cpp src/PulseDetector.cpp src/BPMonitor.cpp src/filters.cpp -o bp_detector_test.exe

if %errorlevel% neq 0 (
    echo.
    echo ERROR: Compilation failed!
    echo.
    pause
    exit /b %errorlevel%
)

echo Compilation successful!
echo.

REM Check if argument provided
if "%~1"=="" (
    echo Usage: build_and_run.bat ^<file_or_folder^> [output_folder]
    echo.
    echo Examples:
    echo   build_and_run.bat Harleen.csv
    echo   build_and_run.bat data_folder
    echo   build_and_run.bat data_folder results_folder
    echo.
    echo Running with default: Harleen.csv
    echo.
    bp_detector_test.exe Harleen.csv
) else if "%~2"=="" (
    echo Running on: %~1
    echo Output: Current directory
    echo.
    bp_detector_test.exe "%~1"
) else (
    echo Running on: %~1
    echo Output: %~2
    echo.
    bp_detector_test.exe "%~1" "%~2"
)

echo.
echo ========== Complete ==========
pause