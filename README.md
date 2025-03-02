# AttitudeDetermination
An in-depth analysis of the performance of advanced attitude determination algorithms for small satellites.

## Introduction
This repository contains a MATLAB-based software for analyzing and comparing different **attitude determination algorithms** used in small satellite applications. The project is part of a university thesis focusing on evaluating and optimizing attitude estimation techniques.

## Features
- Simulated sensor data generation in free-noise and noisy scenarios (sun sensor, magnetometer and gyroscope).
- Three operational conditions applied: nominal, reorientation and failure modes.
- Implementation of classical and advanced attitude determination algorithms, including:
  - TRIAD (Three-Axis Attitude Determination).
  - QUEST (Quaternion Estimatior Algorithm).
  - REQUEST (Recursive Quaternion Estimator Algorithm).
  - EKF (Extended Kalman Filter).
  - Optimal REQUEST.
  - Optimal TRIAD.
  - Fusion algorithms.
- Analysis of parameters such as time step and fading memory factor (this last one for REQUEST algorithm).
- Comparison of algorithm performance based on: efficiency, accuracy, precision and robusness.
- Visualization of attitude estimation results over time, including true and estimated Euler Angles and respective errors.

## Project structure
```plaintext
attitude-determination/
│── AttitudeSoftware/      # Main MATLAB source code directory
│   │── GUI_main.m               # Main script to run the graphical user interface (GUI)
│   │
│   ├── Algorithms/              # Implementation of attitude determination algorithms
│   │   │── EKF.m                      # Extended Kalman Filter algorithm
│   │   │── OptREQUEST.m               # Optimized REQUEST algorithm
│   │   │── QUEST.m                    # Quaternion Estimator algorithm
│   │   │── REQUEST.m                  # Recursive Quaternion Estimator algorithm
│   │   │── TRIAD.m                    # Three-Axis Attitude Determination algorihtm
│   │
│   ├── Analysis/                # Functions for evaluating performance and parameter analysis
│   │   │── algorithm_analysis.m       # Compares performance of different algorithms
│   │   │── fadingMemory_analysis.m    # Analyzes the effect of the fading memory factor
│   │   │── timeStep_analysis.m        # Examines the impact of time step variations
│   │
│   ├── Classes/                 # Object-oriented MATLAB classes for modular algorithm implementation
│   │   │── AttitudeAlgorithm.m        # Base class for attitude algorithms
│   │   │── EKF_Algorithm.m            # EKF class 
│   │   │── Merged_Algorithm.m         # Fusion algorithms class
│   │   │── OptRequest_Algorithm.m     # Optimized REQUEST class
│   │   │── OptTriad_Algorithm.m       # Optimized TRIAD class
│   │   │── Quest_Algorithm.m          # QUEST class
│   │   │── Request_Algorithm.m        # REQUEST class
│   │   │── Satellite.m                # Class representing satellite properties and dynamics
│   │   │── Triad_Algorithm.m          # TRIAD class
│   │
│   ├── Data/                   # Data files for simulation scenarios
│   │   │── Table1.xlsx                # Nominal operations dataset
│   │   │── Table2.xlsx                # Reorientation operations dataset
│   │
│   ├── ExcelResults/           # Directory for storing Excel output results
│   ├── Figures/                # Directory for storing plots and visualization results
│   │
│   ├── Functions/              # Utility functions used throughout the project
│   │   │── attitudeParameters.m      # Computes attitude-related parameters
│   │   │── calculateErrors.m         # Calculates attitude estimation errors
│   │   │── extract_ypr_matrix.m      # Extracts yaw, pitch, and roll angles from Data directory
│   │   │── gyroMeasure.m             # Simulates gyroscope measurements
│   │   │── plotResults.m             # Plots attitude estimation results
│   │   │── savePlot.m                # Saves generated plots to the Figures directory
│   │   │── saveToExcel.m             # Saves numerical results to ExcelResults directory (Excel files)
│   │
│   ├── GUIfunctions/           # Functions related to graphical user interface implementation
│   │
│   ├── Scripts/                # Scripts for executing different simulation and analysis cases
│   │   │── algorithmsStudy.m         # Runs a complete study on attitude determination algorithms
│   │   │── casesStudy_dt.m           # Studies the effect of different time steps
│   │   │── casesStudy_memory.m       # Evaluates memory factor impact on algorithms
│   │
│   ├── Validation/             # Scripts for validating correct implementation
│
│── README.md              # Project documentation
```

## YouTube videos
- [Satellite Attitude Determination with MATLAB | Interactive GUI Demonstration](https://youtu.be/zrsyzi90aA0)
- [Satellite Attitude Determination with MATLAB | Step-by-Step Software Overview](https://youtu.be/piGuVyWjyy0)

## Installation
1. Clone this repository.
2. Open MATLAB and add the directory to the MATLAB path.
3. Run the main script.

## Contributors
- [Maria Castellanos Gallardo](https://github.com/MCG811)
