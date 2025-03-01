classdef EKF_Algorithm < AttitudeAlgorithm

    % EKF_ALGORITHM: Implements the Extended Kalman Filter (EKF) for attitude determination.
    %
    % This class extends the AttitudeAlgorithm base class to implement the EKF method
    % for attitude estimation using gyroscope and reference vector measurements.
    %
    % METHODS:
    % - EKF_Algorithm: Constructor to initialize the EKF algorithm
    % - run: Executes the EKF algorithm to estimate attitude over time
    
    methods
        function obj = EKF_Algorithm(parameters)
            % CONSTRUCTOR: Initializes an instance of the EKF algorithm.
            %
            % INPUT:
            % - parameters: Struct containing EKF configuration settings, such as:
            %   - R: Measurement noise covariance matrix
            %   - Quat: Initial quaternion state estimate
            %
            % OUTPUT:
            % - obj: An instance of the EKF_Algorithm class
            obj@AttitudeAlgorithm('EKF', parameters);
        end
        
        function results = run(obj, bodyData, refData, A_ref, gyro_meas, dt)
            % RUN: Executes the EKF algorithm for attitude determination.
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
            %   - eulerAngles: Estimated Euler angles (Yaw, Pitch, Roll) in degrees
            %   - errorRotation: Rotation error in arcseconds
            %   - time: Computation time per iteration

            fprintf('    Calculating EKF... \n');

            % Initialize results struct
            numSamples = size(bodyData, 3); % Number of time steps
            results = struct();
            results.quaternion = zeros(4, numSamples);
            results.eulerAngles = zeros(3, numSamples);
            results.errorRotation = zeros(1, numSamples);
            results.time = zeros(1, numSamples);
            
            for i = 1:numSamples
                if i~=1
                    % Prepare measurement input for EKF
                    bodyRef = [bodyData(1, :, i) bodyData(2, :, i)]';
                    % Start time measurement
                    tStart = tic;
                    % Perform EKF update step
                    [sys_state, error_covariance] = EKF(obj.Parameters.R, past_state, past_error_cov, gyro_meas(:,i), dt, bodyRef, refData');
                    % Store computation time
                    results.time(1,i) = toc(tStart);
                else
                    % Initialize filter state on the first iteration
                    tStart = tic;
                    sys_state = obj.Parameters.Quat; % Initial quaternion estimate
                    error_covariance = 1e-6*eye(4); % Small initial error covariance
                    results.time(1,i) = toc(tStart);
                end

                % Convert quaternion to DCM
                A_estimated = quat2dcm(sys_state');
                % Compute attitude parameters and errors
                results = attitudeParameters(A_estimated, A_ref, results, i);

                % Update previous state and error covariance for next iteration
                past_state = sys_state;
                past_error_cov = error_covariance;
            end
            fprintf('    Process completed EKF\n');
        end
    end
end
