classdef Request_Algorithm < AttitudeAlgorithm

    % REQUEST_ALGORITHM: Implements the Recursive Quaternion Estimator (REQUEST) algorithm for attitude determination.
    %
    % This class extends the AttitudeAlgorithm base class and applies the REQUEST method
    % to estimate the attitude using body-frame measurements, gyroscope data, and reference vectors.
    %
    % METHODS:
    % - Request_Algorithm: Constructor to initialize the algorithm
    % - run: Executes the REQUEST algorithm

    methods
        function obj = Request_Algorithm(parameters)

            % CONSTRUCTOR: Initializes an instance of the REQUEST algorithm.
            %
            % INPUT:
            % - parameters: Struct containing algorithm configuration settings, such as:
            %   - weights: Weights assigned to sensor measurements
            %   - fadingMemory: Fading memory factor for recursive estimation
            %
            % OUTPUT:
            % - obj: An instance of the Request_Algorithm class

            obj@AttitudeAlgorithm('REQUEST', parameters);
        end
        
        function results = run(obj, bodyData, refData, A_ref, gyro_meas, dt)

            % RUN: Executes the REQUEST algorithm for attitude determination.
            %
            % INPUTS:
            % - bodyData: Measured vectors in the body frame (Nx3xT matrix)
            % - refData: Reference vectors in the inertial frame (Nx3 matrix)
            % - A_ref: Ground truth direction cosine matrix (DCM)
            % - gyro_meas: Gyroscope measurements (3xT matrix)
            % - dt: Time step (scalar)
            %
            % OUTPUT:
            % - results: Struct containing:
            %   - quaternion: Estimated quaternion at each time step
            %   - eulerAngles: Estimated Euler angles at each time step
            %   - errorRotation: Attitude error at each time step
            %   - time: Computation time per iteration

            fprintf('    Calculating REQUEST... \n');
            
            % Initialize the state matrix K for recursive updates
            K = eye(4);

            % Initialize results struct
            numSamples = size(bodyData, 3); % Number of time steps
            results = struct();
            results.quaternion = zeros(4, numSamples);
            results.eulerAngles = zeros(3, numSamples);
            results.errorRotation = zeros(1, numSamples);
            results.time = zeros(1, numSamples);
            
            for i = 1:numSamples
                % Compute the estimated DCM using REQUEST
                tStart = tic;
                [A_estimated, K] = REQUEST(K, bodyData(:, :, i), refData, i, obj.Parameters.weights, obj.Parameters.fadingMemory, gyro_meas(:,i), dt);
                results.time(1,i) = toc(tStart);

                % Compute attitude parameters and errors
                results = attitudeParameters(A_estimated, A_ref, results, i);
            end

            fprintf('    Process completed REQUEST\n');
        end
    end
end

