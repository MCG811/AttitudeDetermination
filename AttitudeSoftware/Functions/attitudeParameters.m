function [results] = attitudeParameters(A_estimated, A_ref, results, i)

    % ATTITUDEPARAMETERS: Computes attitude estimation metrics.
    %
    % This function evaluates an estimated direction cosine matrix (DCM) by checking its
    % orthogonality, determinant, and computing related attitude parameters such as 
    % quaternions, Euler angles, and rotation error.
    %
    % INPUTS:
    % - A_estimated: Estimated Direction Cosine Matrix (DCM) (3x3)
    % - A_ref: Ground truth reference DCM at different time steps (3x3xT)
    % - results: Struct storing results of attitude estimation
    % - i: Current time step index
    %
    % OUTPUT:
    % - results: Updated struct with computed attitude parameters:
    %   - quaternion: Estimated quaternion
    %   - eulerAngles: Estimated Euler angles (Yaw, Pitch, Roll) in degrees
    %   - errorRotation: Rotation error in arcseconds

    %% Verify orthogonality
    % The estimated matrix should satisfy A * A' = I (identity matrix).
    orthogonality_error = norm(A_estimated * A_estimated' - eye(3), 'fro');
    if orthogonality_error > 1e-6
        warning('La matriz A_estimated no es perfectamente ortogonal.');
    end
    
    %% Verify determinant
    % A valid rotation matrix must have a determinant of exactly 1.
    determinant_error = abs(det(A_estimated) - 1);
    if determinant_error > 1e-6
        warning('La matriz A_estimated tiene un determinante inválido.');
    end

    %% Convert DCM to quaternion representation
    % Convert the estimated rotation matrix (DCM) to a quaternion and store it.
    results.quaternion(:, i) = dcm2quat(A_estimated)';

    %% Convert DCM to Euler angles (Yaw, Pitch, Roll)
    % Convert the estimated rotation matrix (DCM) to Euler angles in the ZYX convention.
    % Convert the angles from radians to degrees.
    [yaw, pitch, roll] = dcm2angle(A_estimated, 'ZYX');
    results.eulerAngles(:, i) = [yaw, pitch, roll] * 180 / pi;

    %% Ensure continuity in Euler angles
    % Avoid jumps greater than 180° to maintain angle continuity.
    if i > 1
        for k = 1:3
            delta = results.eulerAngles(k, i) - results.eulerAngles(k, i-1);
            if delta > 180
                results.eulerAngles(k, i) = results.eulerAngles(k, i) - 360;
            elseif delta < -180
                results.eulerAngles(k, i) = results.eulerAngles(k, i) + 360;
            end
        end
    end

    %% Ensure quaternion continuity
    % Adjust quaternion signs to maintain consistent direction across iterations.
    if i > 1
        dot_product = dot(results.quaternion(:, i-1), results.quaternion(:, i));
        if dot_product < 0
            results.quaternion(:, i) = -results.quaternion(:, i);
        end
    end

    %% Compute rotation error
    % Compute the angular difference between A_estimated and A_ref in arcseconds.
    % This uses the trace of the relative rotation matrix to extract the misalignment angle.
    Mult = A_estimated * A_ref(:, :, i)'; % Relative rotation matrix
    trace_value = trace(Mult); % Compute trace
    cos_theta = max(-1, min(1, 0.5 * (trace_value - 1))); % Clamp to avoid numerical errors
    results.errorRotation(i) = (acos(cos_theta) * 180 / pi) * 3600; % Convert to arcseconds
end


