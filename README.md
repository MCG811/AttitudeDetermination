# AttitudeDetermination
An in-depth analysis of the performance of advanced attitude determination algorithms for small satellites

## 🚀 Introduction
This repository contains a MATLAB-based software for analyzing and comparing different **attitude determination algorithms** used in small satellite applications. The project is part of a university thesis focusing on evaluating and optimizing attitude estimation techniques.

## 🛰️ Features
- Simulated sensor data generation in free-noise and noisy scenarios (sun sensor, magnetometer and gyroscope)
- Three operational conditions applied: nominal, reorientation and failure modes
- Analysis of parameters such as time step and fading memory factor (this last one for REQUEST algorithm)
- Implementation of classical and advanced attitude determination algorithms, including:
  - TRIAD (Three-Axis Attitude Determination)
  - QUEST (Quaternion Estimatior Algorithm)
  - REQUEST (Recursive Quaternion Estimator Algorithm)
  - EKF (Extended Kalman Filter)
  - Optimal REQUEST
  - Optimal TRIAD
  - Fusion algorithms
- Comparison of algorithm performance based on: efficiency, accuracy, precision and robusness
- Visualization of attitude estimation results over time, including true and estimated Euler Angles and respective errors.

## 📁 Project structure
```plaintext
attitude-determination/
│── AttitudeSoftware/         # MATLAB source code for attitude estimation
│   │── GUI_main.m            # Main script to execute
│   │── Algorithms/               #
│   │   |── EKF.m
│   │   |── OptREQUEST.m
│   │   |── QUEST.m
│   │   |── REQUEST.m
│   │   |── TRIAD.m
│   │── Analysis/       
│   │   |── algorithm_analysis.m
│   │   |── fadingMemory_analysis.m
│   │   |── timeStep_analysis.m
│   │── Classes/
│   │   |── AttitudeAlgorithm.m
│   │   |── EKF_Algorithm.m
│   │   |── Merged_Algorithm.m
│   │   |── OptRequest_Algorithm.m
│   │   |── OptTriad_Algorithm.m
│   │   |── Quest_Algorithm.m
│   │   |── Request_Algorithm.m
│   │   |── Satellite.m
│   │   |── Triad_Algoirthm.m        
│   │── Data/
│   │   |── Table1.xlsx # Nominal Operations
│   │   |── Table2.xlsx # Reorientation Operations     
│   │── ExcelResults               # To save excel results
│   │── Figures               # To save figures
│   │── Functions               # Utility functions
│   │   |── attitudeParameters.m
│   │   |── calculateErrors.m
│   │   |── extract_ypr_matrix.m
│   │   |── gyroMeasure.m
│   │   |── plotResults.m
│   │   |── savePlot.m
│   │   |── saveToExcel.m
│   │── GUIfunctions
│   │── Scripts
│   │   |── algorithmsStudy.m
│   │   |── casesStudy_dt.m
│   │   |── casesStudy_memory.m
│   │── Validation               # Utility functions
│── README.md                 # Project documentation
│── LICENSE                   # License file
```

## 🔧 Installation
1. Clone this repository
2. Open MATLAB and add the directory to the MATLAB path
3. Run the main script

## 👥 Contributors
Maria Castellanos Gallardo - 
