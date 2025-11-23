CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2 -I./src

# Source files from src/ directory
SOURCES = csv_test_main.cpp \
          src/PulseDetector.cpp \
          src/BPMonitor.cpp \
          src/filters.cpp

OBJECTS = csv_test_main.o \
          PulseDetector.o \
          BPMonitor.o \
          filters.o

TARGET = bp_detector_test

# Default target
all: $(TARGET)

# Link all object files
$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) $(OBJECTS) -o $(TARGET) -lm

# Compile main from current directory
csv_test_main.o: csv_test_main.cpp
	$(CXX) $(CXXFLAGS) -c csv_test_main.cpp -o csv_test_main.o

# Compile sources from src/ directory
PulseDetector.o: src/PulseDetector.cpp src/PulseDetector.h
	$(CXX) $(CXXFLAGS) -c src/PulseDetector.cpp -o PulseDetector.o

BPMonitor.o: src/BPMonitor.cpp src/BPMonitor.h
	$(CXX) $(CXXFLAGS) -c src/BPMonitor.cpp -o BPMonitor.o

filters.o: src/filters.cpp src/filters.h
	$(CXX) $(CXXFLAGS) -c src/filters.cpp -o filters.o

# Clean build files
clean:
	rm -f $(OBJECTS) $(TARGET)

# Run with single file
run: $(TARGET)
	./$(TARGET) Harleen.csv

# Run with directory
run-dir: $(TARGET)
	./$(TARGET) ./data/ ./results/

# Phony targets
.PHONY: all clean run run-dir