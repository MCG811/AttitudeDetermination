classdef OptTriad_Algorithm < AttitudeAlgorithm

    % OPTTRIAD_ALGORITHM: Implements an optimized TRIAD-based attitude determination algorithm.
    %
    % This class extends the AttitudeAlgorithm base class and applies an optimized 
    % version of the TRIAD (Tri-Axial Attitude Determination) method by fusing results
    % from sun and magnetometer sensor estimates.
    %
    % METHODS:
    % - OptTriad_Algorithm: Constructor to initialize the algorithm
    % - run: Executes the Optimized TRIAD algorithm

    methods
        function obj = OptTriad_Algorithm(parameters)

            % CONSTRUCTOR: Initializes an instance of the Optimized TRIAD algorithm.
            %
            % INPUT:
            % - parameters: Struct containing algorithm configuration settings, including:
            %   - sunvar: Sun sensor variance
            %   - magvar: Magnetometer variance
            %
            % OUTPUT:
            % - obj: An instance of the OptTriad_Algorithm class

            obj@AttitudeAlgorithm('OptTRIAD', parameters);
        end
        
        function results = run(obj, bodyData, refData, A_ref)

            % RUN: Executes the Optimized TRIAD algorithm for attitude determination.
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

            fprintf('    Calculating OptTRIAD... \n');
            
            % Initialize results struct
            numSamples = size(bodyData, 3);
            results = struct();
            results.quaternion = zeros(4, numSamples);
            results.eulerAngles = zeros(3, numSamples);
            results.errorRotation = zeros(1, numSamples);
            results.time = zeros(1, numSamples);
            
            for i = 1:numSamples
                % Compute the estimated DCM using Optimized TRIAD
                tStart = tic;

                % Compute separate TRIAD estimates using sun and magnetometer vectors
                A_estimated_sun = TRIAD(bodyData(:, :, i), refData, 1);
                A_estimated_mag = TRIAD(bodyData(:, :, i), refData, 2);

                % Weighted fusion of the two TRIAD estimates
                part_1 = obj.Parameters.sunvar * A_estimated_sun / (obj.Parameters.sunvar + obj.Parameters.magvar);
                part_2 = obj.Parameters.magvar * A_estimated_mag / (obj.Parameters.sunvar + obj.Parameters.magvar);
                
                % Compute the final attitude matrix
                A = part_1 + part_2;
                A_estimated = 0.5 * (A + (A^-1)');

                % Store computation time
                results.time(1,i) = toc(tStart);
                
                % Compute attitude parameters and errors
                results = attitudeParameters(A_estimated, A_ref, results, i);
            end
            fprintf('    Process completed OptTRIAD\n');
        end
    end
end


