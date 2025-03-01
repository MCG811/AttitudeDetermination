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
dt = 0.1; % Time step
max_time = 120; % Maximum simulation time

% Parameter configuration
file_path = fullfile(softwareFolderPath, 'Data/Table1.xlsx');  % Path to the Excel file
% Call the function to extract Euler angles and rates from the file
[eulerAngles, eulerRates] = extract_ypr_matrix(file_path, 0.1, max_time);
true_eulerAngles = eulerAngles; % Store reference Euler angles

% List of algorithms to analyze
algorithms = {'REQUEST'}; % Algorithms to be tested
numAlgorithms = length(algorithms); % Number of algorithms
numIterations = 1; % Number of iterations per algorithm
weights = [0.5 0.5]; % Sensor weights for the REQUEST algorithm
fading_memory = [0.001 0.01 0.1]; % Different fading memory values to test

% Generate inertial vectors with noise
sun_vector_i = [0.6; 0.8; 0.1]; % [rad] Sun vector
sun_vector_norm = sun_vector_i/norm(sun_vector_i); % Normalize sun vector

mag_vector_i = [10342; 1823; 18186]; % [nT] Magnetic field vector
mag_vector_norm = mag_vector_i/norm(mag_vector_i); % Normalize magnetic vector

% Create reference data matrix (normalized vectors)
refData = [sun_vector_norm, mag_vector_norm]';

% Simulate input parameters
time_vec = 0:dt:max_time; % Time vector for the simulation
true_time_vec = time_vec; % Store reference time vector
samp_freq = 1 / dt; % Sampling frequency
length_time_vec = length(time_vec); % Length of time vector

% Sensor variances
mag_var = (0.012 * sqrt(samp_freq))^2; % Magnetic field sensor variance
sun_var = 0.00349^2; % Sun sensor variance
gyro_var = (0.004 * (pi/180) * sqrt(samp_freq))^2; % gyro noise density: 0.011 [degrees/(s*sqrt(Hz))]

% Generate gyroscope measurements with noise
[gyro_meas] = gyroMeasure(eulerAngles(2,:), eulerAngles(3,:), eulerRates, 0);

%% --------------- INITIALIZATION ---------------
% Define performance metrics for result tables
metrics = {'Time', ...
           'MAE_yaw', 'MAE_pitch', 'MAE_roll', 'MAE_rotation', ...
           'MSE_yaw', 'MSE_pitch', 'MSE_roll', ...
           'RMSE_yaw', 'RMSE_pitch', 'RMSE_roll', ...
           'MaxError_yaw', 'MaxError_pitch', 'MaxError_roll', ...
           'STD_yaw', 'STD_pitch', 'STD_roll'};

% Initialize a structure to store results for each algorithm
results = struct();

% Preallocate storage for results per algorithm
for algIdx = 1:numAlgorithms
    algorithmName = algorithms{algIdx};
    results.(algorithmName) = zeros(length(metrics), length(fading_memory));
end

%% -------------- MAIN LOOP --------------
% Loop over different fading memory values
for faIdx = 1:length(fading_memory)
    fprintf('Analizando para Fading Memory = %d \n', fading_memory(faIdx));

    % Create Satellite object with defined sensor parameters
    satellite = Satellite(sun_vector_i, mag_vector_i, sun_var, mag_var);
    % Compute quaternions and rotation matrices for reference attitude
    quaternions = satellite.getQuaternion(eulerAngles(1,:), eulerAngles(2,:), eulerAngles(3,:));
    A_ref = satellite.getDCM(eulerAngles(1,:), eulerAngles(2,:), eulerAngles(3,:));

    % Initialize accumulators for performance metrics
    MAE = zeros(3, numIterations, numAlgorithms);
    MSE = zeros(3, numIterations, numAlgorithms);
    RMSE = zeros(3, numIterations, numAlgorithms);
    MaxError = zeros(3, numIterations, numAlgorithms);
    MAE_rot_angle = zeros(numIterations, numAlgorithms);

    % Initialize storage for error values over time
    errorsYaw = zeros(numIterations, length_time_vec, numAlgorithms);
    errorsPitch = zeros(numIterations, length_time_vec, numAlgorithms);
    errorsRoll = zeros(numIterations, length_time_vec, numAlgorithms);

    % Initialize time computation storage
    timeComputation = zeros(numIterations, numAlgorithms);

    % Loop over iterations
    for iter = 1:numIterations % cambiar variable --> run
        fprintf('  Iteración %d de %d...\n', iter, numIterations);

        % Convert inertial data to body frame
        [bodyData] = satellite.Intertial2BodyFrame(quaternions);

        % Loop over algorithms
        for algIdx = 1:numAlgorithms
            algorithmName = algorithms{algIdx};

            % Execute the corresponding algorithm
            switch algorithmName
                case 'REQUEST'
                    REQUESTparameters.weights = weights;
                    REQUESTparameters.fadingMemory = fading_memory(faIdx);
                    algorithmInstance = Request_Algorithm(REQUESTparameters);
                    algResults = algorithmInstance.run(bodyData, refData, A_ref, gyro_meas, dt);
                otherwise
                    error('Algoritmo no soportado: %s', algorithmName);
            end
            % Store computation time
            timeComputation(iter,algIdx) = sum(algResults.time(1,:));

            % Compute error metrics for this iteration
            iterationMetrics = calculateErrors(eulerAngles * 180/pi, algResults.eulerAngles, algResults.errorRotation);


            % If this is the last iteration, generate result plots
            if iter == numIterations
               plotResults(time_vec, time_vec, iterationMetrics, eulerAngles* 180/pi, algResults.eulerAngles, algorithmName, 'fading_memory', fading_memory(faIdx))
            end

            % Store error metrics per iteration
            MAE(:, iter, algIdx) = [iterationMetrics.MAE.yaw; iterationMetrics.MAE.pitch; iterationMetrics.MAE.roll];
            MSE(:, iter, algIdx) = [iterationMetrics.MSE.yaw; iterationMetrics.MSE.pitch; iterationMetrics.MSE.roll];
            RMSE(:, iter, algIdx) = [iterationMetrics.RMSE.yaw; iterationMetrics.RMSE.pitch; iterationMetrics.RMSE.roll];
            MaxError(:, iter, algIdx) = [iterationMetrics.maxError.yaw; iterationMetrics.maxError.pitch; iterationMetrics.maxError.roll];
            MAE_rot_angle(iter, algIdx) = iterationMetrics.MAE_rot_angle;

            % Store error values over time for standard deviation computation
            errorsYaw(iter, :, algIdx) = iterationMetrics.error.yaw;
            errorsPitch(iter, :, algIdx) = iterationMetrics.error.pitch;
            errorsRoll(iter, :, algIdx) = iterationMetrics.error.roll;
        end
    end


    % Compute average metrics over iterations and store them per algorithm
    for algIdx = 1:numAlgorithms
        MAE_avg = mean(MAE(:, :, algIdx), 2);
        MSE_avg = mean(MSE(:, :, algIdx), 2);
        RMSE_avg = mean(RMSE(:, :, algIdx), 2);
        MaxError_avg = mean(MaxError(:, :, algIdx), 2);
        MAE_rot_avg = mean(MAE_rot_angle(:, algIdx));

        % Compute standard deviation over time and average values
        MeanSTD = [mean(std(errorsYaw(:, :, algIdx), 0, 1)); ...
                   mean(std(errorsPitch(:, :, algIdx), 0, 1)); ...
                   mean(std(errorsRoll(:, :, algIdx), 0, 1))];

        % Compute average computation time
        time_avg = mean(timeComputation(:,algIdx));

        % Store computed metrics in the results matrix for the algorithm
        results.(algorithms{algIdx})(:, faIdx) = [
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
        'VariableNames', strcat('F_', string(fading_memory)));
    results.(algorithmName) = resultsTable;
end

% Save the results into an Excel file
saveToExcel(results, 'ResultsFadingMemory.xlsx')
fprintf('Análisis completado.\n');