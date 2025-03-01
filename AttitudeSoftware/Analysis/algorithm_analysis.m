function resultsTable = algorithm_analysis(dt,max_time, scenario, noise, numIterations, algorithms_analysis, algorithmConfig)

    % ALGORITHM_ANALYSIS: Evaluates attitude determination algorithms through simulation.
    %
    % INPUTS:
    % - dt: Time step (scalar)
    % - max_time: Total simulation time (scalar)
    % - scenario: String defining the simulation scenario ('Nominal operations', 'Failure operations', 'Reorientation operations')
    % - noise: String defining noise condition ('Noise-free' or 'Noisy')
    % - numIterations: Number of Monte Carlo iterations (scalar)
    % - algorithms_analysis: Cell array containing algorithm names to be analyzed
    % - algorithmConfig: Struct containing configuration parameters for each algorithm
    %
    % OUTPUT:
    % - resultsTable: Table summarizing the error metrics for each algorithm

    % Generate time vector
    time_vec = 0:dt:max_time;
    samp_freq = 1 / dt;
    length_time_vec = length(time_vec);
    
    % Sensor variances
    mag_var = (0.012 * sqrt(samp_freq))^2;  % Magnetometer variance
    sun_var = 0.00349^2;                    % Sun sensor variance
    gyro_var = (0.004 * sqrt(samp_freq))^2; % Gyroscope variance
    
    % Select the appropriate dataset based on the scenario
    if strcmp(scenario, 'Nominal operations') || strcmp(scenario, 'Failure operations')
        file_path = 'data/Table1.xlsx';  % Ruta al archivo Excel
    elseif strcmp(scenario, 'Reorientation operations')
        file_path = 'data/Table2.xlsx';  % Ruta al archivo Excel
    end

    % Extract Euler angles and rates from dataset
    [eulerAngles, eulerRates] = extract_ypr_matrix(file_path, dt, max_time);
    % Simulate gyroscope measurements
    [gyro_meas] = gyroMeasure(eulerAngles(2,:), eulerAngles(3,:), eulerRates, 0);
    
    % Define inertial reference vectors (normalized)
    sun_vector_i = [0.6; 0.8; 0.1]; % [rad] Vector solar
    sun_vector_norm = sun_vector_i/norm(sun_vector_i);
    
    mag_vector_i = [10342; 1823; 18186]; % [nT] Vector magnético
    mag_vector_norm = mag_vector_i/norm(mag_vector_i);
    
    refData = [sun_vector_norm, mag_vector_norm]';
    
    % Create Satellite object with or without noise
    if strcmp(noise, 'Noise-free')
        satellite = Satellite(sun_vector_i, mag_vector_i, 0, 0);
    else
        satellite = Satellite(sun_vector_i, mag_vector_i, sun_var, mag_var);
    end

    % Generate quaternions and direction cosine matrices (DCMs)
    quaternions = satellite.getQuaternion(eulerAngles(1,:), eulerAngles(2,:), eulerAngles(3,:));
    A_ref = satellite.getDCM(eulerAngles(1,:), eulerAngles(2,:), eulerAngles(3,:));
    
    % Identify standard and fusion-based algorithms
    fusionAlgorithms = {'Fusion E-T', 'Fusion E-Q'};
    algorithms = algorithms_analysis(~ismember(algorithms_analysis, fusionAlgorithms));  % Contains all others
    algorithms_2 = algorithms_analysis(ismember(algorithms_analysis, fusionAlgorithms)); % Contains only fusion algorithms
    
    numAlgorithms = length(algorithms) + length(algorithms_2);
    
    % Define performance metrics
    metrics = {'Time', ...
               'MAE_yaw', 'MAE_pitch', 'MAE_roll', 'MAE_rotation', ...
               'MSE_yaw', 'MSE_pitch', 'MSE_roll', ...
               'RMSE_yaw', 'RMSE_pitch', 'RMSE_roll', ...
               'MaxError_yaw', 'MaxError_pitch', 'MaxError_roll', ...
               'Mean_STD_yaw', 'Mean_STD_pitch', 'Mean_STD_roll'};

    % Initialize storage for simulation results
    eulerAngles_Triad = zeros(3,length_time_vec,numIterations);
    eulerAngles_Quest = zeros(3,length_time_vec,numIterations);
    eulerAngles_EKF = zeros(3,length_time_vec,numIterations);
    
    % Initialize error metric matrices
    MAE = zeros(3, numIterations, numAlgorithms); % 3 rows (yaw, pitch, roll)
    MSE = zeros(3, numIterations, numAlgorithms);
    RMSE = zeros(3, numIterations, numAlgorithms);
    MaxError = zeros(3, numIterations, numAlgorithms);
    MAE_rot_angle = zeros(numIterations, numAlgorithms);

    % Error tracking over time
    errorsYaw = zeros(numIterations, length_time_vec, numAlgorithms);
    errorsPitch = zeros(numIterations, length_time_vec, numAlgorithms);
    errorsRoll = zeros(numIterations, length_time_vec, numAlgorithms);
    
    % Computation time storage
    timeComputation = zeros(numIterations, numAlgorithms);
    
    % Inicializar acumuladores para métricas
    resultsMatrix = zeros(length(metrics), numAlgorithms);
    
    % Loop over Monte Carlo iterations
    for iter = 1:numIterations
        fprintf('  Iteration %d of %d...\n', iter, numIterations);
    
        % Generate body-frame measurements based on scenario
        if strcmp(scenario, 'Nominal operations') || strcmp(scenario, 'Reorientation operations')
            [bodyData] = satellite.Intertial2BodyFrame(quaternions);
        elseif strcmp(scenario, 'Failure operations')
            [bodyData] = satellite.Intertial2BodyFrame_failure(quaternions);
        end


        % Loop over selected algorithms
        for algIdx = 1:length(algorithms)
            algorithmName = algorithms{algIdx};
    
            % Execute the selected algorithm
            switch algorithmName
                case 'TRIAD'
                    algorithmInstance = Triad_Algorithm(algorithmConfig.TRIAD.anchorVector); % Parámetro del algoritmo TRIAD
                    algResults = algorithmInstance.run(bodyData, refData, A_ref);
                    eulerAngles_Triad(:,:,iter) = algResults.eulerAngles * pi/180;
    
                case 'QUEST'
                    algorithmInstance = Quest_Algorithm(algorithmConfig.QUEST.sensorWeights); % Pesos para QUEST
                    algResults = algorithmInstance.run(bodyData, refData, A_ref);
                    eulerAngles_Quest(:,:,iter) = algResults.eulerAngles * pi/180;
    
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
                    eulerAngles_EKF(:,:,iter) = algResults.eulerAngles * pi/180;
    
                case 'OptREQUEST'
                    OptREQUESTparameters.weights = algorithmConfig.OptREQUEST.sensorWeights;
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
    
            if iter == numIterations
                % Plot results for the last iteration
                plotResults(time_vec, time_vec, iterationMetrics, eulerAngles* 180/pi, algResults.eulerAngles, algorithmName, 'dt', dt);
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
    
    % Save STD for fusion algorithms
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
    % Loop over Monte Carlo iterations
    for iter = 1:numIterations
        fprintf('  Iteration %d of %d...\n', iter, numIterations);

        % Bucle sobre algoritmos
        for algIdx = 1:length(algorithms_2)
    
            algorithmName = algorithms_2{algIdx};
    
            % Loop over selected algorithms
            switch algorithmName
                case 'Fusion E-T'
                    algorithmInstance = Merged_Algorithm(NaN);
                    algResults = algorithmInstance.runTriad(eulerAngles_Triad(:,:,iter), eulerAngles_EKF(:,:,iter), stdTriad, stdEKF, A_ref);
                    
                    % Store computation time
                    timeComputation(iter,realNum_alg+algIdx) = sum(algResults.time(1,:)) + mean(timeComputation(:,algIdxTriad)) + mean(timeComputation(:,algIdxEKF));
                
                case 'Fusion E-Q'
                    algorithmInstance = Merged_Algorithm(NaN);
                    algResults = algorithmInstance.runQuest(eulerAngles_Quest(:,:,iter), eulerAngles_EKF(:,:,iter), stdQuest, stdEKF, A_ref);
                    
                    % Store computation time
                    timeComputation(iter,realNum_alg+algIdx) = sum(algResults.time(1,:)) + mean(timeComputation(:,algIdxQuest)) + mean(timeComputation(:,algIdxEKF));
                
                otherwise
                    error('Algoritmo no soportado: %s', algorithmName);
            end
        
            % Compute error metrics for this iteration
            iterationMetrics = calculateErrors(eulerAngles*180/pi, algResults.eulerAngles, algResults.errorRotation);
            
            if iter == numIterations
                % Plot results for the last iteration
                plotResults(time_vec, time_vec, iterationMetrics, eulerAngles* 180/pi, algResults.eulerAngles, algorithmName, 'dt', dt);
            end
            % Store error metrics for this iteration
            MAE(:, iter, realNum_alg+algIdx) = [iterationMetrics.MAE.yaw; iterationMetrics.MAE.pitch; iterationMetrics.MAE.roll];
            MSE(:, iter, realNum_alg+algIdx) = [iterationMetrics.MSE.yaw; iterationMetrics.MSE.pitch; iterationMetrics.MSE.roll];
            RMSE(:, iter, realNum_alg+algIdx) = [iterationMetrics.RMSE.yaw; iterationMetrics.RMSE.pitch; iterationMetrics.RMSE.roll];
            MaxError(:, iter, realNum_alg+algIdx) = [iterationMetrics.maxError.yaw; iterationMetrics.maxError.pitch; iterationMetrics.maxError.roll];
            MAE_rot_angle(iter, realNum_alg+algIdx) = iterationMetrics.MAE_rot_angle;
        
            % Store errors for statistical analysis
            errorsYaw(iter, :, realNum_alg+algIdx) = iterationMetrics.error.yaw;
            errorsPitch(iter, :, realNum_alg+algIdx) = iterationMetrics.error.pitch;
            errorsRoll(iter, :, realNum_alg+algIdx) = iterationMetrics.error.roll;
        end
    end
    
    % Compute mean and standard deviation metrics
    for algIdx = 1:numAlgorithms
        MAE_avg = mean(MAE(:, :, algIdx), 2);
        MSE_avg = mean(MSE(:, :, algIdx), 2);
        RMSE_avg = mean(RMSE(:, :, algIdx), 2);
        MaxError_avg = mean(MaxError(:, :, algIdx), 2);
        MAE_rot_avg = mean(MAE_rot_angle(:, algIdx));
    
        % Compute standard deviation over time
        MeanSTD = [mean(std(errorsYaw(:, :, algIdx), 0, 1)); ...
                   mean(std(errorsPitch(:, :, algIdx), 0, 1)); ...
                   mean(std(errorsRoll(:, :, algIdx), 0, 1))];
        
        % Compute mean computation time
        time_avg = mean(timeComputation(:,algIdx));
    
        % Store results
        resultsMatrix(:, algIdx) = [
            time_avg; ...
            MAE_avg; MAE_rot_avg; ...
            MSE_avg; ...
            RMSE_avg; ...
            MaxError_avg; ...
            MeanSTD
        ];
    end
    
    % Convert results matrix to table
    resultsTable = array2table(resultsMatrix, ...
        'RowNames', metrics, ...
        'VariableNames', [algorithms, algorithms_2]);
    results = struct();
    results.Algorithms = resultsTable;

    % Save results in an Excel
    saveToExcel(results, 'Results_algorithmAnalysis.xlsx');
    
    % Display results
    disp('Results table: ');
    disp(resultsTable);
   
    fprintf('Análisis completado.\n');
end

