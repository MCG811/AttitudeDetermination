classdef Triad_Algorithm < AttitudeAlgorithm

    % TRIAD_ALGORITHM: Implements the TRIAD (Tri-Axial Attitude Determination) algorithm.
    %
    % This class extends the AttitudeAlgorithm base class and applies the TRIAD method
    % to estimate the attitude using body-frame measurements and reference vectors.
    %
    % METHODS:
    % - Triad_Algorithm: Constructor to initialize the algorithm
    % - run: Executes the TRIAD algorithm

    methods
        function obj = Triad_Algorithm(parameters)
            % CONSTRUCTOR: Initializes an instance of the TRIAD algorithm.
            %
            % INPUT:
            % - parameters: Struct containing algorithm configuration settings, such as:
            %   - anchorVector: Specifies which reference vector to use as the primary anchor
            %
            % OUTPUT:
            % - obj: An instance of the Triad_Algorithm class

            obj@AttitudeAlgorithm('TRIAD', parameters);
        end
        
        function results = run(obj, bodyData, refData, A_ref)
            % RUN: Executes the TRIAD algorithm for attitude determination.
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
            
            fprintf('    Calculating TRIAD... \n');
            
            % Initialize results struct
            numSamples = size(bodyData, 3); % Number of time steps
            results = struct();
            results.quaternion = zeros(4, numSamples);
            results.eulerAngles = zeros(3, numSamples);
            results.errorRotation = zeros(1, numSamples);
            results.time = zeros(1, numSamples);
            
            for i = 1:numSamples
                % Compute the estimated DCM using TRIAD
                tStart = tic;
                A_estimated = TRIAD(bodyData(:, :, i), refData, obj.Parameters);
                results.time(1,i) = toc(tStart);

                % Compute attitude parameters and errors
                results = attitudeParameters(A_estimated, A_ref, results, i);
            end
            fprintf('    Process completed TRIAD\n');
        end
    end
end

