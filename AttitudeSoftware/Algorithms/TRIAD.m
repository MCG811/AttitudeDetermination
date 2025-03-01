function [A_estimated] = TRIAD(body_meas, inertial_values, anchor_vector)
    % TRIAD: Estimates the attitude matrix using the TRIAD method.
    % 
    % INPUTS:
    % - body_meas: Measured vectors in the body frame (2x3 matrix).
    % - inertial_values: Corresponding reference vectors in the inertial frame (2x3 matrix).
    % - anchor_vector: Specifies which vector to use as the primary reference (1 or 2).
    %
    % OUTPUT:
    % - A_estimated: Estimated attitude matrix (3x3 matrix).

    % Extract vectors from body and inertial reference frames
    s1_b = body_meas(1, :)';  % First measured body-frame vector
    s2_b = body_meas(2, :)';  % Second measured body-frame vector
    s1_r = inertial_values(1, :)'; % First reference inertial vector
    s2_r = inertial_values(2, :)'; % Second reference inertial vector
    
    % Normalize vectors to ensure unit magnitude
    s1_b = s1_b / norm(s1_b);
    s2_b = s2_b / norm(s2_b);
    s1_r = s1_r / norm(s1_r);
    s2_r = s2_r / norm(s2_r);
    
    % Select the base vector for TRIAD
    if anchor_vector == 1
        t1_b = s1_b; % Use first body-frame vector as the primary reference
        t1_r = s1_r; % Use first inertial reference vector
    else
        t1_b = s2_b; % Use second body-frame vector as the primary reference
        t1_r = s2_r; % Use second inertial reference vector
    end
    
    % Compute the TRIAD basis in the body frame
    t2_b = cross(s1_b, s2_b) / norm(cross(s1_b, s2_b)); % Perpendicular to both measured vectors
    t3_b = cross(t1_b, t2_b); % Orthogonal third basis vector
    b_T = [t1_b t2_b t3_b]; % Construct the TRIAD frame in the body frame
    
    % Compute the TRIAD basis in the inertial frame
    t2_r = cross(s1_r, s2_r) / norm(cross(s1_r, s2_r)); % Perpendicular to both reference vectors
    t3_r = cross(t1_r, t2_r); % Orthogonal third basis vector
    r_T = [t1_r t2_r t3_r];  % Construct the TRIAD frame in the inertial frame
    
    % Compute the estimated attitude matrix (DCM)
    A_estimated = b_T * r_T';
end
