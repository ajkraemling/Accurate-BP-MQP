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
REM    3. Searches ALL subdirectories for CSV files
REM    4. Creates timestamped output files with detector results
REM    5. Preserves AUS_PULSE_HEARD column if present
REM
REM  CSV REQUIREMENTS:
REM    - Must have columns: Time or Timestamp, Pressure, PPGSignal
REM    - Optional: rawPPGSignal or PPG (recommended - will filter fresh)
REM    - Optional: AUS_PULSE_HEARD (will be preserved in output)
REM
REM  NEW FEATURES:
REM    - Accepts "Time" OR "Timestamp" as time column
REM    - Accepts "rawPPGSignal" OR "PPG" as raw PPG column
REM    - Recursively searches all subdirectories for CSV files
REM    - Preserves subdirectory structure in output
REM    - Preserves AUS_PULSE_HEARD column in output
REM
REM  OUTPUT:
REM    - Console: Summary table of detector results
REM    - CSV Files: <filename>_results_<timestamp>.csv
REM    - Text Files: <filename>_report_<timestamp>.txt
REM
REM ============================================================

echo ========== BP Detector CSV Tester ==========
echo.

echo Compiling...
g++ -std=c++11 -O2 -I./src csv_test_main.cpp src/SystolicDetector.cpp src/BPMonitor.cpp src/MAPDetector.cpp src/filters.cpp src/MotorController.cpp -o bp_detector_test.exe

if %errorlevel% neq 0 (
    echo.
    echo ERROR: Compilation failed!
    echo.
    pause
    exit /b %errorlevel%
)

echo Compilation successful!
echo.
REM Check if argument provided (only for printing usage)
if "%~1"=="" (
    echo Usage: build_and_run.bat ^<file_or_folder^> [output_folder]
    echo.
    echo Examples:
    echo   build_and_run.bat Harleen.csv
    echo   build_and_run.bat data_folder
    echo   build_and_run.bat data_folder results_folder
    echo.
    echo New Features:
    echo   - Recursively searches all subdirectories
    echo   - Accepts "Time" or "Timestamp" column names
    echo   - Accepts "rawPPGSignal" or "PPG" column names
    echo   - Preserves AUS_PULSE_HEARD column if present
    goto :eof
)

REM === SINGLE EXECUTION LINE FIX ===
echo Running on: %~1
if "%~2"=="" (
    echo Output: Current directory
) else (
    echo Output: %~2
)
echo Searching all subdirectories for CSV files...
echo.

REM This single call passes ALL arguments received: "%~1" "%~2" etc.
bp_detector_test.exe %*

echo.
echo ========== Complete ==========
pause