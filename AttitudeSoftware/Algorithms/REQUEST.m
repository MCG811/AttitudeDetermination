function [A_estimated, K] = REQUEST(past_K, b_i, r_i, i, w_i, fading_memory, gyro_meas, dt)
    % REQUEST: Recursive Quaternion Estimator (REQUEST) algorithm for satellite attitude determination.
    %
    % INPUTS:
    %   - past_K: Previous K matrix (from the last time step)
    %   - w_i: Weights associated with each observation (2x1 vector)
    %   - fading_memory: Fading memory factor (scalar)
    %   - gyro_meas: Gyroscope measurements [w1, w2, w3] (1x3 vector)
    %   - b_i: Measured vectors in the body frame (2x3 matrix)
    %   - r_i: Corresponding reference vectors in the inertial frame (2x3 matrix)
    %   - dt: Time step (scalar)
    %   - i: Iteration index (scalar)
    %
    % OUTPUTS:
    %   - A_estimated: Estimated attitude as a Direction Cosine Matrix (DCM)
    %   - K: Updated K matrix for the next iteration

    % Handle cases where no valid body-frame measurement is available
    if norm(b_i(1,:)) == 0
        w_i = [0.01, 0.99];  % Adjust weight distribution
        fading_memory = 0.99; % Increase fading memory to rely more on past states
    end

    % Compute the sum of observation weights
    sum_w = sum(w_i);

    % Extract gyroscope measurements
    w1 = gyro_meas(1);
    w2 = gyro_meas(2);
    w3 = gyro_meas(3);

    % Construct the B matrix for quaternion propagation using gyroscope measurements
    B = (1 / 2) * [0, -w1, -w2, -w3;
                   w1,  0,  w3, -w2;
                   w2, -w3,  0,  w1;
                   w3,  w2, -w1,  0];

    % Compute the state transition matrix based on the time step
    trans = eye(4) + dt * B; % First-order approximation

    % Compute the weighted observation matrix (B_total)
    B_total = zeros(3, 3); % Initialize B_total matrix
    for j = 1:size(b_i, 1)
        B_total = B_total + w_i(j) * (b_i(j, :)' * r_i(j, :));
    end

    % Compute sigma (trace of B_total), symmetric matrix S, and vector Z
    sigma = trace(B_total);               % Sum of diagonal elements (trace)
    S = B_total + B_total';               % Symmetric part of B_total
    Z = [B_total(2, 3) - B_total(3, 2);   % Vector of anti-symmetric elements
         B_total(3, 1) - B_total(1, 3);
         B_total(1, 2) - B_total(2, 1)];

    % Construct the single observation-based K matrix
    single_obs_K = [sigma, Z';            % Extended K matrix for individual observation
                    Z, S - sigma * eye(3)];

    % Initialize or propagate the K matrix
    if i == 1
        K_1 = 0; % If first iteration, set initial K_1 to zero
    else
        % Apply fading memory and propagate K from the previous step
        K_1 = fading_memory * trans * past_K * trans';
    end

    % Compute the updated K matrix, considering both fading memory and new observations
    K = K_1 + sum_w * single_obs_K;

    % Compute eigenvalues and eigenvectors of the updated K matrix
    [eigvec, eigval] = eig(K);            % Compute eigen decomposition
    [~, max_idx] = max(diag(eigval));     % Identify the index of the largest eigenvalue

    % Extract the quaternion corresponding to the largest eigenvalue
    q = eigvec(:, max_idx);

    % Ensure quaternion sign convention is consistent
    if q(1) < 0
        q = -q; % Flip sign to maintain quaternion positivity
    end
    
    % Convert quaternion to Direction Cosine Matrix (DCM)
    A_estimated = quat2dcm(q');
end


