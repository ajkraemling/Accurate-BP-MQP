CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2 -I./src

# Source files from src/ directory
SOURCES = csv_test_main.cpp \
          src/SystolicDetector.cpp \
          src/BPMonitor.cpp \
          src/MAPDetector.cpp \
          src/filters.cpp \
		  src/MotorController.cpp

OBJECTS = csv_test_main.o \
          SystolicDetector.o \
          BPMonitor.o \
          MAPDetector.o \
          filters.o \ 
		  MotorController.o

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
SystolicDetector.o: src/SystolicDetector.cpp src/SystolicDetector.h
	$(CXX) $(CXXFLAGS) -c src/SystolicDetector.cpp -o SystolicDetector.o

BPMonitor.o: src/BPMonitor.cpp src/BPMonitor.h
	$(CXX) $(CXXFLAGS) -c src/BPMonitor.cpp -o BPMonitor.o

MAPDetector.o: src/MAPDetector.cpp src/MAPDetector.h
	$(CXX) $(CXXFLAGS) -c src/MAPDetector.cpp -o MAPDetector.o

filters.o: src/filters.cpp src/filters.h
	$(CXX) $(CXXFLAGS) -c src/filters.cpp -o filters.o

filters.o: src/MotorController.cpp src/MotorController.h
	$(CXX) $(CXXFLAGS) -c src/MotorController.cpp -o MotorController.o

# Clean build files
clean:
	rm -f $(OBJECTS) $(TARGET)

# Run with single file
run: $(TARGET)
	./$(TARGET) Harleen.csv

# Run with directory (will now search recursively)
run-dir: $(TARGET)
	./$(TARGET) ./data/ ./results/

# Phony targets
.PHONY: all clean run run-dir