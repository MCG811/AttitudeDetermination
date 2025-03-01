clear all
close all

%% ----------- CONFIGURATION -----------
% Get the path of the "Main" script
mainScriptPath = fileparts(mfilename('fullpath'));

% Move up one level to the "Software" folder
softwareFolderPath = fileparts(mainScriptPath);

% Define paths for "Functions", "Classes", and "Algorithms"
functionsPath = fullfile(softwareFolderPath, 'Functions');
classesPath = fullfile(softwareFolderPath, 'Classes');
algorithmsPath = fullfile(softwareFolderPath, 'Algorithms');

% Add these paths to MATLAB’s search path
addpath(functionsPath, classesPath, algorithmsPath);

%% ------------ PARAMETER DEFINITION ------------
% Define simulation parameters
dt = 0.1; % Time step value
max_time = 120; % Maximum simulation time
time_vec = 0:dt:max_time; % Time vector
samp_freq = 1 / dt; % Sampling frequency
length_time_vec = length(time_vec); % Number of time steps

% Define sensor noise variances
mag_var = (0.012 * sqrt(samp_freq))^2; % Magnetic field sensor varianc
sun_var = 0.00349^2; % Sun sensor variance
gyro_var = (0.004 * sqrt(samp_freq))^2; % Gyroscope variance

% Load reference data from an Excel file
file_path = fullfile(softwareFolderPath, 'Data/Table1.xlsx'); % Path to the Excel file
[eulerAngles, eulerRates] = extract_ypr_matrix(file_path, dt, max_time);  % Extract Euler angles and rates
[gyro_meas] = gyroMeasure(eulerAngles(2,:), eulerAngles(3,:), eulerRates, 0);  % Generate gyroscope measurements

% Generate inertial vectors
sun_vector_i = [0.6; 0.8; 0.1]; % [rad] Sun vector in inertial frame
sun_vector_norm = sun_vector_i/norm(sun_vector_i); % Normalize

mag_vector_i = [10342; 1823; 18186]; % [nT] Magnetic field vector in inertial frame
mag_vector_norm = mag_vector_i/norm(mag_vector_i); % Normalize

refData = [sun_vector_norm, mag_vector_norm]'; % Store reference vectors

% Create a Satellite object
satellite = Satellite(sun_vector_i, mag_vector_i, sun_var, mag_var);
quaternions = satellite.getQuaternion(eulerAngles(1,:), eulerAngles(2,:), eulerAngles(3,:));
A_ref = satellite.getDCM(eulerAngles(1,:), eulerAngles(2,:), eulerAngles(3,:));

% Define the list of attitude estimation algorithms to analyze
algorithms = {'TRIAD', 'QUEST', 'REQUEST', 'EKF', 'OptREQUEST', 'OptTRIAD'}; % Standard algorithms
algorithms_2 = {'Fusion E-T','Fusion E-Q'}; % Fusion-based algorithms

numAlgorithms = length(algorithms) + length(algorithms_2); % Total number of algorithms
numIterations = 1; % Number of iterations per algorithm


%% --------------- INITIALIZATION ---------------
% Define performance metrics for the results table
metrics = {'Time', ...
           'MAE_yaw', 'MAE_pitch', 'MAE_roll', 'MAE_rotation', ...
           'MSE_yaw', 'MSE_pitch', 'MSE_roll', ...
           'RMSE_yaw', 'RMSE_pitch', 'RMSE_roll', ...
           'MaxError_yaw', 'MaxError_pitch', 'MaxError_roll', ...
           'Mean_STD_yaw', 'Mean_STD_pitch', 'Mean_STD_roll'};

% Initialize storage for simulated Euler angles
eulerAngles_Triad = zeros(3,length_time_vec,numIterations);
eulerAngles_Quest = zeros(3,length_time_vec,numIterations);
eulerAngles_EKF = zeros(3,length_time_vec,numIterations);

% Initialize matrices for error accumulation
MAE = zeros(3, numIterations, numAlgorithms); % Mean Absolute Error
MSE = zeros(3, numIterations, numAlgorithms); % Mean Squared Error
RMSE = zeros(3, numIterations, numAlgorithms); % Root Mean Squared Error
MaxError = zeros(3, numIterations, numAlgorithms); % Maximum Error
MAE_rot_angle = zeros(numIterations, numAlgorithms); % MAE for rotation angle

errorsYaw = zeros(numIterations, length_time_vec, numAlgorithms); % Yaw errors over time
errorsPitch = zeros(numIterations, length_time_vec, numAlgorithms); % Pitch errors over time
errorsRoll = zeros(numIterations, length_time_vec, numAlgorithms); % Roll errors over time

timeComputation = zeros(numIterations, numAlgorithms); % Computation time storage

% Initialize results matrix
resultsMatrix = zeros(length(metrics), numAlgorithms);

%% -------------- MAIN LOOP --------------
% Iterate through simulation runs
for iter = 1:numIterations
    fprintf('  Iteración %d de %d...\n', iter, numIterations);

    % Transform inertial vectors to body frame
    [bodyData] = satellite.Intertial2BodyFrame(quaternions);

    % Loop over algorithms
    for algIdx = 1:length(algorithms)
        algorithmName = algorithms{algIdx};

        % Execute the corresponding algorithm
        switch algorithmName
            case 'TRIAD'
                algorithmInstance = Triad_Algorithm(1); % Parámetro del algoritmo TRIAD
                algResults = algorithmInstance.run(bodyData, refData, A_ref);
                eulerAngles_Triad(:,:,iter) = algResults.eulerAngles * pi/180;

            case 'QUEST'
                algorithmInstance = Quest_Algorithm([0.1, 0.9]); % Pesos para QUEST
                algResults = algorithmInstance.run(bodyData, refData, A_ref);
                eulerAngles_Quest(:,:,iter) = algResults.eulerAngles * pi/180;

            case 'REQUEST'
                REQUESTparameters.weights = [0.1, 0.9];
                REQUESTparameters.fadingMemory = 0.001;
                algorithmInstance = Request_Algorithm(REQUESTparameters);
                algResults = algorithmInstance.run(bodyData, refData, A_ref, gyro_meas, dt);
    
            case 'EKF'
                EKFparameters.Quat = quaternions(:,1);
                EKFparameters.R = [(sun_var)*eye(3) , zeros(3,3);
                                    zeros(3,3) (mag_var)*eye(3)];
                algorithmInstance = EKF_Algorithm(EKFparameters);
                algResults = algorithmInstance.run(bodyData, refData, A_ref, gyro_meas, dt);
                eulerAngles_EKF(:,:,iter) = algResults.eulerAngles * pi/180;

            case 'OptREQUEST'
                OptREQUESTparameters.weights = [0.1, 0.9];
                OptREQUESTparameters.sunvar = sun_var;
                OptREQUESTparameters.magvar = mag_var;
                OptREQUESTparameters.gyrovar = gyro_var; 
                algorithmInstance = OptRequest_Algorithm(OptREQUESTparameters);
                algResults = algorithmInstance.run(bodyData, refData, A_ref, gyro_meas, dt);

            case 'OptTRIAD'
                OptTRIADparameters.sunvar = sun_var;
                OptTRIADparameters.magvar = mag_var;
                algorithmInstance = OptTriad_Algorithm(OptTRIADparameters);
                algResults = algorithmInstance.run(bodyData, refData, A_ref);

            otherwise
                error('Algoritmo no soportado: %s', algorithmName);
        end

        % Store computation time
        timeComputation(iter,algIdx) = mean(algResults.time(1,:));

        % Compute error metrics for this iteration
        iterationMetrics = calculateErrors(eulerAngles*180/pi, algResults.eulerAngles, algResults.errorRotation);

        % Plot results for the last iteration
        if iter == numIterations
            plotResults(time_vec, time_vec, iterationMetrics, eulerAngles* 180/pi, algResults.eulerAngles, algorithmName, 'dt', dt);
        end

        % Store error metrics for this algorithm
        MAE(:, iter, algIdx) = [iterationMetrics.MAE.yaw; iterationMetrics.MAE.pitch; iterationMetrics.MAE.roll];
        MSE(:, iter, algIdx) = [iterationMetrics.MSE.yaw; iterationMetrics.MSE.pitch; iterationMetrics.MSE.roll];
        RMSE(:, iter, algIdx) = [iterationMetrics.RMSE.yaw; iterationMetrics.RMSE.pitch; iterationMetrics.RMSE.roll];
        MaxError(:, iter, algIdx) = [iterationMetrics.maxError.yaw; iterationMetrics.maxError.pitch; iterationMetrics.maxError.roll];
        MAE_rot_angle(iter, algIdx) = iterationMetrics.MAE_rot_angle;

        errorsYaw(iter, :, algIdx) = iterationMetrics.error.yaw;
        errorsPitch(iter, :, algIdx) = iterationMetrics.error.pitch;
        errorsRoll(iter, :, algIdx) = iterationMetrics.error.roll;
    end
end

% Store STD for fusion algorithms
algIdxTriad = find(strcmp(algorithms, 'TRIAD'));
stdTriad = [std(errorsYaw(:, :, algIdxTriad), 0, 1);
            std(errorsPitch(:, :, algIdxTriad), 0, 1);
            std(errorsRoll(:, :, algIdxTriad), 0, 1)];

algIdxEKF = find(strcmp(algorithms, 'EKF'));
stdEKF = [std(errorsYaw(:, :, algIdxEKF), 0, 1);
          std(errorsPitch(:, :, algIdxEKF), 0, 1);
          std(errorsRoll(:, :, algIdxEKF), 0, 1)];

algIdxQuest = find(strcmp(algorithms, 'QUEST'));
stdQuest = [std(errorsYaw(:, :, algIdxQuest), 0, 1);
          std(errorsPitch(:, :, algIdxQuest), 0, 1);
          std(errorsRoll(:, :, algIdxQuest), 0, 1)];

realNum_alg = length(algorithms);

%% -------------- MAIN LOOP FUSION ALGORITHMS --------------
for iter = 1:numIterations
    fprintf('  Iteración %d de %d...\n', iter, numIterations);
    fieldName = sprintf('iter_%d', iter); % Genera el nombre del campo como un string
    % Loop over algorithms
    for algIdx = 1:length(algorithms_2)

        algorithmName = algorithms_2{algIdx};

        % Execute the corresponding algorithm
        switch algorithmName
            case 'Fusion E-T'
                algorithmInstance = Merged_Algorithm(NaN);
                algResults = algorithmInstance.runTriad(eulerAngles_Triad(:,:,iter), eulerAngles_EKF(:,:,iter), stdTriad, stdEKF, A_ref);
                
                % Store computation time
                timeComputation(iter,realNum_alg+algIdx) = sum(algResults.time(1,:)) + mean(timeComputation(:,algIdxTriad)) + mean(timeComputation(:,algIdxEKF));
            case 'Fusion E-Q'
                algorithmInstance = Merged_Algorithm(NaN);
                algResults = algorithmInstance.runQuest(eulerAngles_Quest(:,:,iter), eulerAngles_EKF(:,:,iter), stdTriad, stdEKF, A_ref);
                
                % Store computation time
                timeComputation(iter,realNum_alg+algIdx) = sum(algResults.time(1,:)) + mean(timeComputation(:,algIdxQuest)) + mean(timeComputation(:,algIdxEKF));
            otherwise
                error('Algoritmo no soportado: %s', algorithmName);
        end
    
        % Compute error metrics for this iteration
        iterationMetrics = calculateErrors(eulerAngles*180/pi, algResults.eulerAngles, algResults.errorRotation);
        
        % Plot results for the last iteration
        if iter == numIterations
            plotResults(time_vec, time_vec, iterationMetrics, eulerAngles* 180/pi, algResults.eulerAngles, algorithmName, 'dt', dt);
        end

        % Store error metrics for this algorithm
        MAE(:, iter, realNum_alg+algIdx) = [iterationMetrics.MAE.yaw; iterationMetrics.MAE.pitch; iterationMetrics.MAE.roll];
        MSE(:, iter, realNum_alg+algIdx) = [iterationMetrics.MSE.yaw; iterationMetrics.MSE.pitch; iterationMetrics.MSE.roll];
        RMSE(:, iter, realNum_alg+algIdx) = [iterationMetrics.RMSE.yaw; iterationMetrics.RMSE.pitch; iterationMetrics.RMSE.roll];
        MaxError(:, iter, realNum_alg+algIdx) = [iterationMetrics.maxError.yaw; iterationMetrics.maxError.pitch; iterationMetrics.maxError.roll];
        MAE_rot_angle(iter, realNum_alg+algIdx) = iterationMetrics.MAE_rot_angle;
    
        errorsYaw(iter, :, realNum_alg+algIdx) = iterationMetrics.error.yaw;
        errorsPitch(iter, :, realNum_alg+algIdx) = iterationMetrics.error.pitch;
        errorsRoll(iter, :, realNum_alg+algIdx) = iterationMetrics.error.roll;
    end
end

%% -------------- COMPUTE RESULTS --------------
% Compute average metrics over iterations
for algIdx = 1:numAlgorithms
    MAE_avg = mean(MAE(:, :, algIdx), 2);
    MSE_avg = mean(MSE(:, :, algIdx), 2);
    RMSE_avg = mean(RMSE(:, :, algIdx), 2);
    MaxError_avg = mean(MaxError(:, :, algIdx), 2);
    MAE_rot_avg = mean(MAE_rot_angle(:, algIdx));

    % Compute standard deviation of errors over time
    MeanSTD = [mean(std(errorsYaw(:, :, algIdx), 0, 1)); ...
               mean(std(errorsPitch(:, :, algIdx), 0, 1)); ...
               mean(std(errorsRoll(:, :, algIdx), 0, 1))];
    % Compute average computation time
    time_avg = mean(timeComputation(:,algIdx));

    % Store computed metrics in the results matrix
    resultsMatrix(:, algIdx) = [
        time_avg; ...
        MAE_avg; MAE_rot_avg; ...
        MSE_avg; ...
        RMSE_avg; ...
        MaxError_avg; ...
        MeanSTD
    ];
end

%% -------------- SAVE RESULTS --------------
% Create a results table
resultsTable = array2table(resultsMatrix, ...
    'RowNames', metrics, ... % Row names correspond to the defined metrics
    'VariableNames', [algorithms, algorithms_2]); % Column names correspond to analyzed algorithms

% Save results to an Excel file
results = struct();
results.Algorithms = resultsTable;
saveToExcel(results, 'ResultsAlgorithms.xlsx');

% Display the results table
disp('Tabla de resultados: ');
disp(resultsTable);

fprintf('Análisis completado.\n');