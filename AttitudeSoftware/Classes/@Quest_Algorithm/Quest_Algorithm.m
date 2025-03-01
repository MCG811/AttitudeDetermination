classdef Quest_Algorithm < AttitudeAlgorithm

    % QUEST_ALGORITHM: Implements the QUEST (Quaternion Estimator) algorithm for attitude determination.
    %
    % This class extends the AttitudeAlgorithm base class and applies the QUEST method
    % to estimate the attitude using body-frame measurements and reference vectors.
    %
    % METHODS:
    % - Quest_Algorithm: Constructor to initialize the algorithm
    % - run: Executes the QUEST algorithm

    methods
        function obj = Quest_Algorithm(parameters)

            % CONSTRUCTOR: Initializes an instance of the QUEST algorithm.
            %
            % INPUT:
            % - parameters: Struct containing algorithm configuration settings, such as:
            %   - sensorWeights: Weights assigned to sensor measurements
            %
            % OUTPUT:
            % - obj: An instance of the Quest_Algorithm class

            obj@AttitudeAlgorithm('QUEST', parameters);
        end
        
        function results = run(obj, bodyData, refData, A_ref)

            % RUN: Executes the QUEST algorithm for attitude determination.
            %
            % INPUTS:
            % - bodyData: Measured vectors in the body frame (Nx3xT matrix)
            % - refData: Reference vectors in the inertial frame (Nx3 matrix)
            % - A_ref: Ground truth direction cosine matrix (DCM)
            %
            % OUTPUT:
            % - results: Struct containing:
            %   - quaternion: Estimated quaternion at each time step
            %   - eulerAngles: Estimated Euler angles at each time step
            %   - errorRotation: Attitude error at each time step
            %   - time: Computation time per iteration

            fprintf('    Calculating QUEST... \n');
            
            % Initialize results struct
            numSamples = size(bodyData, 3); % Number of time steps
            results = struct();
            results.quaternion = zeros(4, numSamples);
            results.eulerAngles = zeros(3, numSamples);
            results.errorRotation = zeros(1, numSamples);
            results.time = zeros(1, numSamples);
            
            for i = 1:numSamples
                % Compute the estimated DCM using QUEST
                tStart = tic;
                A_estimated = QUEST(bodyData(:, :, i), refData, obj.Parameters);
                results.time(1,i) = toc(tStart);

                % Compute attitude parameters and errors
                results = attitudeParameters(A_estimated, A_ref, results, i);
            end
            fprintf('    Process completed QUEST\n');
        end
    end
end


