classdef OptRequest_Algorithm < AttitudeAlgorithm

    % OPTREQUEST_ALGORITHM: Implements the Optimized REQUEST algorithm for attitude determination.
    %
    % This class extends the AttitudeAlgorithm base class to implement an optimized 
    % version of the Recursive Quaternion Estimator (REQUEST) algorithm.
    %
    % METHODS:
    % - OptRequest_Algorithm: Constructor to initialize the algorithm
    % - run: Executes the standard Optimized REQUEST algorithm

    methods
        function obj = OptRequest_Algorithm(parameters)

            % CONSTRUCTOR: Initializes an instance of the Optimized REQUEST algorithm.
            %
            % INPUT:
            % - parameters: Struct containing algorithm configuration settings, including:
            %   - weights: Sensor weights for the estimation process
            %   - magvar: Magnetometer variance
            %   - sunvar: Sun sensor variance
            %   - gyrovar: Gyroscope variance
            %
            % OUTPUT:
            % - obj: An instance of the OptRequest_Algorithm class

            obj@AttitudeAlgorithm('OptREQUEST', parameters);
        end
        
        function results = run(obj, bodyData, refData, A_ref, gyro_meas, dt)

            % RUN: Executes the standard Optimized REQUEST algorithm for attitude estimation.
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

            fprintf('    Calculating OptREQUEST... \n');

            % Initialize loop state
            loop = struct();
            loop.B = zeros(3, 3);

            % Initialize results struct
            numSamples = size(bodyData, 3);  % Number of time steps
            results = struct();
            results.quaternion = zeros(4, numSamples);
            results.eulerAngles = zeros(3, numSamples);
            results.errorRotation = zeros(1, numSamples);
            results.time = zeros(1, numSamples);
            
            for i = 1:numSamples
                % Compute the estimated DCM using Optimized REQUEST
                tStart = tic;
                [A_estimated, loop] = OptREQUEST(obj.Parameters, bodyData(:,:,i), refData, loop, i, dt, gyro_meas(:,i));
                results.time(1,i) = toc(tStart);

                % Compute attitude parameters and errors
                results = attitudeParameters(A_estimated, A_ref, results, i);
            end

            fprintf('    Process completed OptREQUEST\n');
        end
    end
end
