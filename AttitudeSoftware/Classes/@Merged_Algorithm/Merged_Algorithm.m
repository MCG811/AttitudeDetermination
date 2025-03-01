classdef Merged_Algorithm < AttitudeAlgorithm

    % MERGED_ALGORITHM: Implements a fusion-based attitude determination algorithm.
    %
    % This class extends the AttitudeAlgorithm base class and fuses results from different 
    % attitude estimation methods (e.g., EKF and QUEST or EKF and TRIAD) using weighted averaging.
    %
    % METHODS:
    % - Merged_Algorithm: Constructor to initialize the algorithm
    % - runQuest: Executes fusion between QUEST and EKF
    % - runTriad: Executes fusion between TRIAD and EKF
    methods
        function obj = Merged_Algorithm(parameters)
            % CONSTRUCTOR: Initializes an instance of the Merged Algorithm.
            %
            % INPUT:
            % - parameters: Struct containing configuration settings
            %
            % OUTPUT:
            % - obj: An instance of the Merged_Algorithm class
            obj@AttitudeAlgorithm('MergedAlg', parameters);

        end
        
        function results = runQuest(obj, eulerAnglesQuest, eulerAnglesEKF, stdQuest, stdEKF, A_ref)
            
            % RUNQUEST: Fuses QUEST and EKF results for attitude estimation.
            %
            % INPUTS:
            % - eulerAnglesQuest: Euler angles estimated using QUEST (3xT matrix)
            % - eulerAnglesEKF: Euler angles estimated using EKF (3xT matrix)
            % - stdQuest: Standard deviation of QUEST estimation errors (3xT matrix)
            % - stdEKF: Standard deviation of EKF estimation errors (3xT matrix)
            % - A_ref: Ground truth direction cosine matrix (DCM)
            %
            % OUTPUT:
            % - results: Struct containing:
            %   - quaternion: Estimated quaternion at each time step
            %   - eulerAngles: Estimated Euler angles at each time step
            %   - errorRotation: Attitude error at each time step
            %   - time: Computation time per iteration

            fprintf('    Calculating MergedAlg QUEST... \n');
            
            % Replace NaN or zero standard deviations with 1 (to avoid division errors)
            stdQuest(isnan(stdQuest) | stdQuest == 0) = 1;
            stdEKF(isnan(stdEKF) | stdEKF == 0) = 1;

            % Initialize results struct
            numSamples = length(stdQuest);
            results = struct();
            results.quaternion = zeros(4, numSamples);
            results.eulerAngles = zeros(3, numSamples);
            results.errorRotation = zeros(1, numSamples);
            results.time = zeros(1, numSamples);
            
            for i = 1:numSamples
                % Compute weighted fusion of attitude estimates
                tStart = tic;
                yaw = (stdQuest(1,i)^2 * eulerAnglesEKF(1,i))/(stdEKF(1,i)^2 + stdQuest(1,i)^2) + (stdEKF(1,i)^2 * eulerAnglesQuest(1,i))/(stdEKF(1,i)^2 + stdQuest(1,i)^2);
                pitch = (stdQuest(2,i)^2 * eulerAnglesEKF(2,i))/(stdEKF(2,i)^2 + stdQuest(2,i)^2) + (stdEKF(2,i)^2 * eulerAnglesQuest(2,i))/(stdEKF(2,i)^2 + stdQuest(2,i)^2);
                roll = (stdQuest(3,i)^2 * eulerAnglesEKF(3,i))/(stdEKF(3,i)^2 + stdQuest(3,i)^2) + (stdEKF(3,i)^2 * eulerAnglesQuest(3,i))/(stdEKF(3,i)^2 + stdQuest(3,i)^2);
                
                % Convert fused angles to direction cosine matrix (DCM)
                A_estimated = angle2dcm(yaw, pitch, roll); % refrence attitude in DCM
                results.time(1,i) = toc(tStart);
                
                % Compute attitude parameters and errors
                results = attitudeParameters(A_estimated, A_ref, results, i);
            end
            fprintf('    Process completed MergedAlg TRIAD\n');
        end

        function results = runTriad(obj, eulerAnglesTriad, eulerAnglesEKF, stdTriad, stdEKF, A_ref)
            
            % RUNTRIAD: Fuses TRIAD and EKF results for attitude estimation.
            %
            % INPUTS:
            % - eulerAnglesTriad: Euler angles estimated using TRIAD (3xT matrix)
            % - eulerAnglesEKF: Euler angles estimated using EKF (3xT matrix)
            % - stdTriad: Standard deviation of TRIAD estimation errors (3xT matrix)
            % - stdEKF: Standard deviation of EKF estimation errors (3xT matrix)
            % - A_ref: Ground truth direction cosine matrix (DCM)
            %
            % OUTPUT:
            % - results: Struct containing:
            %   - quaternion: Estimated quaternion at each time step
            %   - eulerAngles: Estimated Euler angles at each time step
            %   - errorRotation: Attitude error at each time step
            %   - time: Computation time per iteration
            fprintf('    Calculating MergedAlg QUEST... \n');

            % Replace NaN or zero standard deviations with 1 (to avoid division errors)
            stdTriad(isnan(stdTriad) | stdTriad == 0) = 1;
            stdEKF(isnan(stdEKF) | stdEKF == 0) = 1;

            % Initialize results struct
            numSamples = length(stdTriad);

            results = struct();
            results.quaternion = zeros(4, numSamples);
            results.eulerAngles = zeros(3, numSamples);
            results.errorRotation = zeros(1, numSamples);
            results.time = zeros(1, numSamples);
            
            for i = 1:numSamples
                % Compute weighted fusion of attitude estimates
                tStart = tic;
                yaw = (stdTriad(1,i)^2 * eulerAnglesEKF(1,i))/(stdEKF(1,i)^2 + stdTriad(1,i)^2) + (stdEKF(1,i)^2 * eulerAnglesTriad(1,i))/(stdEKF(1,i)^2 + stdTriad(1,i)^2);
                pitch = (stdTriad(2,i)^2 * eulerAnglesEKF(2,i))/(stdEKF(2,i)^2 + stdTriad(2,i)^2) + (stdEKF(2,i)^2 * eulerAnglesTriad(2,i))/(stdEKF(2,i)^2 + stdTriad(2,i)^2);
                roll = (stdTriad(3,i)^2 * eulerAnglesEKF(3,i))/(stdEKF(3,i)^2 + stdTriad(3,i)^2) + (stdEKF(3,i)^2 * eulerAnglesTriad(3,i))/(stdEKF(3,i)^2 + stdTriad(3,i)^2);
                
                % Convert fused angles to direction cosine matrix (DCM)
                A_estimated = angle2dcm(yaw, pitch, roll); % refrence attitude in DCM
                results.time(1,i) = toc(tStart);

                % Compute attitude parameters and errors
                results = attitudeParameters(A_estimated, A_ref, results, i);
            end
            fprintf('    Process completed MergedAlg TRIAD\n');
        end
    end
end

