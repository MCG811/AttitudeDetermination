clear all
close all

%% ----------- CONFIGURATION -----------
% Get the path of the "Main" script
mainScriptPath = fileparts(mfilename('fullpath'));

% Move up one level to the "Software" folder
softwareFolderPath = fileparts(mainScriptPath);


% Define paths for "Functions," "Classes," and "Algorithms" folders
functionsPath = fullfile(softwareFolderPath, 'Functions');
classesPath = fullfile(softwareFolderPath, 'Classes');
algorithmsPath = fullfile(softwareFolderPath, 'Algorithms');

% Add these paths to MATLAB’s search path
addpath(functionsPath, classesPath, algorithmsPath);

%% ------------ PARAMETER DEFINITION ------------
% Initial parameters
dt_values = [0.1 0.2 0.4 0.6 0.8 1];  % Different time step values to analyze
max_time = 120; % Maximum simulation time

% Load reference attitude data from an Excel file
file_path = fullfile(softwareFolderPath, 'Data/Table1.xlsx');  % Path to the Excel file
[true_eulerAngles, true_eulerRates] = extract_ypr_matrix(file_path, 0.1, max_time);
true_time_vec = 0 : 0.1 : max_time; % Reference time vector

% List of attitude estimation algorithms to analyze
algorithms = {'TRIAD', 'QUEST', 'REQUEST', 'EKF'}; % Algorithms to analyze
numAlgorithms = length(algorithms);
numIterations = 1; % Number of iterations per algorithm

% Generate inertial vectors with noise
sun_vector_i = [0.6; 0.8; 0.1]; % [rad] Solar vector
sun_vector_norm = sun_vector_i/norm(sun_vector_i);

mag_vector_i = [10342; 1823; 18186]; % [nT] Magnetic field vector
mag_vector_norm = mag_vector_i/norm(mag_vector_i);

% Reference data containing unit vectors
refData = [sun_vector_norm, mag_vector_norm]';

%% --------------- INITIALIZATION ---------------
% Define performance metrics for the results table
metrics = {'Time', ...
           'MAE_yaw', 'MAE_pitch', 'MAE_roll', 'MAE_rotation', ...
           'MSE_yaw', 'MSE_pitch', 'MSE_roll', ...
           'RMSE_yaw', 'RMSE_pitch', 'RMSE_roll', ...
           'MaxError_yaw', 'MaxError_pitch', 'MaxError_roll', ...
           'STD_yaw', 'STD_pitch', 'STD_roll'};

% Structure to store results for each algorithm
results = struct();

% Initialize storage for each algorithm's results
for algIdx = 1:numAlgorithms
    algorithmName = algorithms{algIdx};
    results.(algorithmName) = zeros(length(metrics), length(dt_values));
end

%% -------------- MAIN LOOP --------------
% Loop over different time step values (dt)
for dtIdx = 1:length(dt_values)
    dt = dt_values(dtIdx); % Current time step value
    fprintf('Analizando para dt = %d...\n', dt);

    % Simulate input parameters 
    time_vec = 0:dt:max_time; % Time vector for current dt
    samp_freq = 1 / dt; % Sampling frequency
    length_time_vec = length(time_vec);

    % Sensor variances
    mag_var = (0.012 * sqrt(samp_freq))^2;
    sun_var = 0.00349^2;
    gyro_var = (0.004 * (pi/180) * sqrt(samp_freq))^2; % gyro noise density: 0.011 [degrees/(s*sqrt(Hz))]

    % Select appropriate indices for resampling the reference attitude data
    indx = round(1 : dt/0.1 : (max_time/0.1 +1));
    eulerAngles = true_eulerAngles(:,indx);
    eulerRates = true_eulerRates(:, indx);
    % Generate simulated gyroscope measurements
    [gyro_meas] = gyroMeasure(eulerAngles(2,:), eulerAngles(3,:), eulerRates, 0);

    % Create a Satellite object with sensor noise
    satellite = Satellite(sun_vector_i, mag_vector_i, sun_var, mag_var);
    quaternions = satellite.getQuaternion(eulerAngles(1,:), eulerAngles(2,:), eulerAngles(3,:));
    A_ref = satellite.getDCM(eulerAngles(1,:), eulerAngles(2,:), eulerAngles(3,:));

    % Initialize storage for error metrics
    MAE = zeros(3, numIterations, numAlgorithms);
    MSE = zeros(3, numIterations, numAlgorithms);
    RMSE = zeros(3, numIterations, numAlgorithms);
    MaxError = zeros(3, numIterations, numAlgorithms);
    MAE_rot_angle = zeros(numIterations, numAlgorithms);

    errorsYaw = zeros(numIterations, length_time_vec, numAlgorithms);
    errorsPitch = zeros(numIterations, length_time_vec, numAlgorithms);
    errorsRoll = zeros(numIterations, length_time_vec, numAlgorithms);

    timeComputation = zeros(numIterations, numAlgorithms);

    % Loop over iterations
    for iter = 1:numIterations % cambiar variable --> run
        fprintf('  Iteración %d de %d...\n', iter, numIterations);

        % Transform inertial vectors to body frame using quaternion rotation
        [bodyData] = satellite.Intertial2BodyFrame(quaternions);

        % Loop over each algorithm
        for algIdx = 1:numAlgorithms
            algorithmName = algorithms{algIdx};

            % Run the corresponding algorithm
            switch algorithmName
                case 'TRIAD'
                    algorithmInstance = Triad_Algorithm(2);
                    algResults = algorithmInstance.run(bodyData, refData, A_ref);

                case 'QUEST'
                    algorithmInstance = Quest_Algorithm([0.5, 0.5]);
                    algResults = algorithmInstance.run(bodyData, refData, A_ref);

                case 'REQUEST'
                    REQUESTparameters.weights = [0.5, 0.5];
                    REQUESTparameters.fadingMemory = 0.001;
                    algorithmInstance = Request_Algorithm(REQUESTparameters);
                    algResults = algorithmInstance.run(bodyData, refData, A_ref, gyro_meas, dt);

                case 'EKF'
                    EKFparameters.Quat = quaternions(:,1);
                    EKFparameters.R = [(sun_var)*eye(3) , zeros(3,3);
                                        zeros(3,3) (mag_var)*eye(3)];
                    algorithmInstance = EKF_Algorithm(EKFparameters);
                    algResults = algorithmInstance.run(bodyData, refData, A_ref, gyro_meas, dt);

                otherwise
                    error('Algoritmo no soportado: %s', algorithmName);
            end
            timeComputation(iter,algIdx) = mean(algResults.time(1,:));

            % Compute errors for this iteration
            iterationMetrics = calculateErrors(eulerAngles * 180/pi, algResults.eulerAngles, algResults.errorRotation);
            
            % Plot results for the last iteration
            if iter == numIterations
               plotResults(time_vec, true_time_vec, iterationMetrics, true_eulerAngles* 180/pi, algResults.eulerAngles, algorithmName, 'dt', dt);
            end

            % Store error metrics for this iteration
            MAE(:, iter, algIdx) = [iterationMetrics.MAE.yaw; iterationMetrics.MAE.pitch; iterationMetrics.MAE.roll];
            MSE(:, iter, algIdx) = [iterationMetrics.MSE.yaw; iterationMetrics.MSE.pitch; iterationMetrics.MSE.roll];
            RMSE(:, iter, algIdx) = [iterationMetrics.RMSE.yaw; iterationMetrics.RMSE.pitch; iterationMetrics.RMSE.roll];
            MaxError(:, iter, algIdx) = [iterationMetrics.maxError.yaw; iterationMetrics.maxError.pitch; iterationMetrics.maxError.roll];
            MAE_rot_angle(iter, algIdx) = iterationMetrics.MAE_rot_angle;

            % Store errors for standard deviation calculations
            errorsYaw(iter, :, algIdx) = iterationMetrics.error.yaw;
            errorsPitch(iter, :, algIdx) = iterationMetrics.error.pitch;
            errorsRoll(iter, :, algIdx) = iterationMetrics.error.roll;
        end
    end

    % Compute average metrics over iterations and store results for each algorithm
    for algIdx = 1:numAlgorithms
        MAE_avg = mean(MAE(:, :, algIdx), 2);
        MSE_avg = mean(MSE(:, :, algIdx), 2);
        RMSE_avg = mean(RMSE(:, :, algIdx), 2);
        MaxError_avg = mean(MaxError(:, :, algIdx), 2);
        MAE_rot_avg = mean(MAE_rot_angle(:, algIdx));

        % Compute standard deviation of errors over time and average the values
        MeanSTD = [mean(std(errorsYaw(:, :, algIdx), 0, 1)); ...
                   mean(std(errorsPitch(:, :, algIdx), 0, 1)); ...
                   mean(std(errorsRoll(:, :, algIdx), 0, 1))];

        % Compute average computation time for the algorithm
        time_avg = mean(timeComputation(:,algIdx));

        % Store computed metrics in the results matrix for the algorithm
        results.(algorithms{algIdx})(:, dtIdx) = [
            time_avg; ...
            MAE_avg; MAE_rot_avg; ...
            MSE_avg; ...
            RMSE_avg; ...
            MaxError_avg; ...
            MeanSTD];
    end
end

%% -------------- SAVE RESULTS --------------
% Convert numerical results into tables for each algorithm
for algIdx = 1:numAlgorithms
    algorithmName = algorithms{algIdx};
    resultsTable = array2table(results.(algorithmName), ...
        'RowNames', metrics, ...
        'VariableNames', strcat('dt_', string(dt_values)));
    results.(algorithmName) = resultsTable;
end

% Save the results into an Excel file
saveToExcel(results, 'Results_casesStudy.xlsx')
fprintf('Análisis completado.\n');
