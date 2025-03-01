function results = timeStep_analysis(dt_values,max_time, scenario, noise, numIterations, algorithms_analysis, algorithmConfig)
    
% TIMESTEP_ANALYSIS: Evaluates attitude determination algorithms across different time step values.
    %
    % INPUTS:
    % - dt_values: Array of time step values to be analyzed
    % - max_time: Total simulation time (scalar)
    % - scenario: Simulation scenario ('Nominal operations', 'Failure operations', 'Reorientation operations')
    % - noise: Noise condition ('Noise-free' or 'Noisy')
    % - numIterations: Number of Monte Carlo iterations (scalar)
    % - algorithms_analysis: Cell array of algorithms to be analyzed
    % - algorithmConfig: Struct containing configuration parameters for each algorithm
    %
    % OUTPUT:
    % - results: Struct containing tables of error metrics for each algorithm

    % Select dataset based on the scenario
    if strcmp(scenario, 'Nominal operations') || strcmp(scenario, 'Failure operations')
        file_path = 'data/Table1.xlsx';  % Ruta al archivo Excel
    elseif strcmp(scenario, 'Reorientation operations')
        file_path = 'data/Table2.xlsx';  % Ruta al archivo Excel
    end

    % Extract true Euler angles and rates from dataset
    [true_eulerAngles, true_eulerRates] = extract_ypr_matrix(file_path, 0.1, max_time);
    true_time_vec = 0 : 0.1 : max_time;
    
    % Extract true Euler angles and rates from dataset
    algorithms = algorithms_analysis;
    numAlgorithms = length(algorithms);
    
    % Define inertial reference vectors (normalized)
    sun_vector_i = [0.6; 0.8; 0.1]; % [rad] Solar vector
    sun_vector_norm = sun_vector_i/norm(sun_vector_i);
    
    mag_vector_i = [10342; 1823; 18186]; % [nT] Magnetic field
    mag_vector_norm = mag_vector_i/norm(mag_vector_i);
    
    refData = [sun_vector_norm, mag_vector_norm]';
    
    % Define performance metrics
    metrics = {'Time', ...
               'MAE_yaw', 'MAE_pitch', 'MAE_roll', 'MAE_rotation', ...
               'MSE_yaw', 'MSE_pitch', 'MSE_roll', ...
               'RMSE_yaw', 'RMSE_pitch', 'RMSE_roll', ...
               'MaxError_yaw', 'MaxError_pitch', 'MaxError_roll', ...
               'STD_yaw', 'STD_pitch', 'STD_roll'};
    
    % Initialize results structure for each algorithm
    results = struct();
    for algIdx = 1:numAlgorithms
        algorithmName = algorithms{algIdx};
        results.(algorithmName) = zeros(length(metrics), length(dt_values));
    end
    
    % Loop over different time step values
    for dtIdx = 1:length(dt_values)
        dt = dt_values(dtIdx); % Current time step value
        fprintf('Analizando para dt = %d...\n', dt);
    
        % Generate time vector
        time_vec = 0:dt:max_time;
        samp_freq = 1 / dt;
        length_time_vec = length(time_vec);
    
        % Define sensor variances
        mag_var = (0.012 * sqrt(samp_freq))^2;
        sun_var = 0.00349^2;
        gyro_var = (0.004 * (pi/180) * sqrt(samp_freq))^2; % gyro noise density: 0.011 [degrees/(s*sqrt(Hz))]

        % Downsample Euler angles and rates to match dt
        indx = round(1 : dt/0.1 : (max_time/0.1 +1));
        eulerAngles = true_eulerAngles(:,indx);
        eulerRates = true_eulerRates(:, indx);

        % Simulate gyroscope measurements
        [gyro_meas] = gyroMeasure(eulerAngles(2,:), eulerAngles(3,:), eulerRates, 0);
    
        % Create Satellite object with or without noise
        if strcmp(noise, 'Noise-free')
            satellite = Satellite(sun_vector_i, mag_vector_i, 0, 0);
        else
            satellite = Satellite(sun_vector_i, mag_vector_i, sun_var, mag_var);
        end

        % Generate quaternions and direction cosine matrices (DCMs)
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
    
        % Loop over Monte Carlo iterations
        for iter = 1:numIterations
            fprintf('  Iteración %d de %d...\n', iter, numIterations);
    
            % Generate body-frame measurements based on scenario
            if strcmp(scenario, 'Nominal operations') || strcmp(scenario, 'Reorientation operations')
                [bodyData] = satellite.Intertial2BodyFrame(quaternions);
            elseif strcmp(scenario, 'Failure operations')
                [bodyData] = satellite.Intertial2BodyFrame_failure(quaternions);
            end
    
            % Loop over selected algorithms
            for algIdx = 1:numAlgorithms
                algorithmName = algorithms{algIdx};
    
                % Execute the selected algorithm
                switch algorithmName
                    case 'TRIAD'
                        algorithmInstance = Triad_Algorithm(algorithmConfig.TRIAD.anchorVector);
                        algResults = algorithmInstance.run(bodyData, refData, A_ref);
    
                    case 'QUEST'
                        algorithmInstance = Quest_Algorithm(algorithmConfig.QUEST.sensorWeights);
                        algResults = algorithmInstance.run(bodyData, refData, A_ref);
    
                    case 'REQUEST'
                        REQUESTparameters.weights = algorithmConfig.REQUEST.sensorWeights;
                        REQUESTparameters.fadingMemory = algorithmConfig.REQUEST.fadingMemoryFactor;
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

                % Store computation time
                timeComputation(iter,algIdx) = mean(algResults.time(1,:));
    
                % Compute error metrics for this iteration
                iterationMetrics = calculateErrors(eulerAngles * 180/pi, algResults.eulerAngles, algResults.errorRotation);
    
                if iter == numIterations
                    % Plot error results
                   plotResults(time_vec, true_time_vec, iterationMetrics, true_eulerAngles* 180/pi, algResults.eulerAngles, algorithmName, 'dt', dt);
                end
    
                % Store error metrics for this iteration
                MAE(:, iter, algIdx) = [iterationMetrics.MAE.yaw; iterationMetrics.MAE.pitch; iterationMetrics.MAE.roll];
                MSE(:, iter, algIdx) = [iterationMetrics.MSE.yaw; iterationMetrics.MSE.pitch; iterationMetrics.MSE.roll];
                RMSE(:, iter, algIdx) = [iterationMetrics.RMSE.yaw; iterationMetrics.RMSE.pitch; iterationMetrics.RMSE.roll];
                MaxError(:, iter, algIdx) = [iterationMetrics.maxError.yaw; iterationMetrics.maxError.pitch; iterationMetrics.maxError.roll];
                MAE_rot_angle(iter, algIdx) = iterationMetrics.MAE_rot_angle;
    
                % Store errors for statistical analysis
                errorsYaw(iter, :, algIdx) = iterationMetrics.error.yaw;
                errorsPitch(iter, :, algIdx) = iterationMetrics.error.pitch;
                errorsRoll(iter, :, algIdx) = iterationMetrics.error.roll;
            end
        end
    
        % Compute average error metrics across iterations
        for algIdx = 1:numAlgorithms
            MAE_avg = mean(MAE(:, :, algIdx), 2);
            MSE_avg = mean(MSE(:, :, algIdx), 2);
            RMSE_avg = mean(RMSE(:, :, algIdx), 2);
            MaxError_avg = mean(MaxError(:, :, algIdx), 2);
            MAE_rot_avg = mean(MAE_rot_angle(:, algIdx));
            
            % Compute standard deviation
            MeanSTD = [mean(std(errorsYaw(:, :, algIdx), 0, 1)); ...
                       mean(std(errorsPitch(:, :, algIdx), 0, 1)); ...
                       mean(std(errorsRoll(:, :, algIdx), 0, 1))];
            
            % Compute mean computation time
            time_avg = mean(timeComputation(:,algIdx));
    
            % Store metrics in results structure
            results.(algorithms{algIdx})(:, dtIdx) = [
                time_avg; ...
                MAE_avg; MAE_rot_avg; ...
                MSE_avg; ...
                RMSE_avg; ...
                MaxError_avg; ...
                MeanSTD];
        end
    end
    
    % Convert results to tables for each algorithm
    for algIdx = 1:numAlgorithms
        algorithmName = algorithms{algIdx};
        resultsTable = array2table(results.(algorithmName), ...
            'RowNames', metrics, ...
            'VariableNames', strcat('dt_', string(dt_values)));
        fprintf('Results table: %s\n', algorithmName);
        disp(resultsTable);
        results.(algorithmName) = resultsTable;
    end

    % Save results to an Excel file
    saveToExcel(results, 'Results_timeStep.xlsx')
    fprintf('Análisis completado.\n');

end

