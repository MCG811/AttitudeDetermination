function [A_estimated, loop] = OptREQUEST(parameter, body, ref, loop, k, dt, gyro_meas)
    % OptREQUEST: Optimized REQUEST algorithm for satellite attitude determination.
    %
    % INPUTS:
    % - parameter: Struct containing weights, sensor variances, and gyro variance.
    % - body: Body-frame measured vectors (Nx3 matrix).
    % - ref: Reference-frame vectors (Nx3 matrix).
    % - loop: Struct holding the previous state of the filter.
    % - k: Current iteration index.
    % - dt: Sampling time step.
    % - gyro_meas: Gyroscope measurements [w1, w2, w3] in rad/s.
    %
    % OUTPUTS:
    % - A_estimated: Estimated attitude as a DCM matrix.
    % - loop: Updated filter state for the next iteration.

    % Compute the sum of observation weights
    sum_weights = sum(parameter.weights);

    %% Compute the dK matrix (Measurement Information Matrix)
    B = zeros(3, 3);
    for i = 1:length(parameter.weights)
        B = B + parameter.weights(i) * (body(i, :)' * ref(i, :)); % Weighted sum of dyadic products
    end

    % Compute the elements of the dK matrix
    z = [B(2, 3) - B(3, 2); B(3, 1) - B(1, 3); B(1, 2) - B(2, 1)];
    sigma = trace(B);
    S = B + B';

    % Construct the dK matrix
    dK_n1 = [sigma, z'; z, S - sigma * eye(3)] / sum_weights;

    %% Define Measurement Noise Covariance Matrix (V)
    % Compute noise variances based on sensor properties
    db = [sqrt(parameter.sunvar), sqrt(parameter.sunvar), sqrt(parameter.sunvar); 
          sqrt(parameter.magvar), sqrt(parameter.magvar), sqrt(parameter.magvar)];
    
    Bb = zeros(3, 3);
    for i = 1:length(parameter.weights)
        Bb = Bb + parameter.weights(i) * (db(i, :)' * ref(i, :));
    end

    % Compute elements of the noise covariance matrix
    zb = [Bb(2, 3) - Bb(3, 2); Bb(3, 1) - Bb(1, 3); Bb(1, 2) - Bb(2, 1)];
    sigmab = trace(Bb);
    Sb = Bb + Bb';

    % Normalize the noise matrix
    V_n1 = [sigmab, zb'; zb, Sb - sigmab * eye(3)] / sum_weights;
    V_n1 = V_n1 / norm(V_n1);
    R_n1 = V_n1 * V_n1';  % Measurement noise covariance matrix

    %% Define Process Noise Covariance Matrix (W)
    % Compute process noise matrix related to gyroscope variance
    Be = [0, -parameter.gyrovar, parameter.gyrovar; 
          parameter.gyrovar, 0, -parameter.gyrovar; 
          -parameter.gyrovar, parameter.gyrovar, 0];
    Be = Be * B;

    % Compute elements of process noise covariance matrix
    ze = [Be(3, 2) - Be(2, 3); Be(1, 3) - Be(3, 1); Be(2, 1) - Be(1, 2)];
    sigmae = trace(Be);
    Se = Be + Be';

    % Normalize the process noise matrix
    W_n = [sigmae, ze'; ze, Se - sigmae * eye(3)] * dt;
    W_n = W_n / norm(W_n);
    Q_n = W_n * W_n';  % Process noise covariance matrix

    %% Time Update: State Propagation
    % Compute the gyroscopic angular velocity matrix
    omega = 0.5 * [0, -gyro_meas(1), -gyro_meas(2), -gyro_meas(3);
                   gyro_meas(1), 0, gyro_meas(3), -gyro_meas(2);
                   gyro_meas(2), -gyro_meas(3), 0, gyro_meas(1);
                   gyro_meas(3), gyro_meas(2), -gyro_meas(1), 0];

    % Compute the state transition matrix using a Taylor series approximation
    omega_dt = omega * dt;
    omega_dt2 = omega_dt * omega_dt;
    phi = eye(4) + omega_dt + 0.5 * omega_dt2;  % First two terms of Taylor series expansion
    
    %% Kalman Filter Update Step
    if k == 1
        % If first iteration, initialize state and covariance matrices
        K_n1_n1 = dK_n1 / norm(dK_n1);
        P_n1_n1 = R_n1;
        m_n1 = sum_weights;
    else
        % Propagate the state covariance forward in time
        K_n1_n = loop.phi * loop.K * loop.phi';
        P_n1_n = loop.phi * loop.P * loop.phi' + loop.Q;

        % Compute the measurement update factor (rho)
        if norm(body(1,:)) == 0
            rho_n1 = 0;  % If no valid measurement, set update weight to zero
        else
            rho_n1 = loop.m^2 * trace(P_n1_n) / (loop.m^2 * trace(P_n1_n) + sum_weights^2 * trace(R_n1));
        end

        % Compute the updated weight (m)
        m_n1 = (1 - rho_n1) * loop.m + rho_n1 * sum_weights;

        % Update the Kalman gain matrix
        K_n1_n1 = (1 - rho_n1) * (loop.m / m_n1) * K_n1_n + rho_n1 * (sum_weights / m_n1) * dK_n1;
        K_n1_n1 = K_n1_n1 / norm(K_n1_n1);

        % Update the covariance matrix
        P_n1_n1 = ((1 - rho_n1) * (loop.m / m_n1))^2 * P_n1_n + (rho_n1 * sum_weights / m_n1)^2 * R_n1;
    end

    %% Update the filter state for the next iteration
    loop.K = K_n1_n1;
    loop.P = P_n1_n1;
    loop.phi = phi;
    loop.m = m_n1;
    loop.Q = Q_n;
    loop.B = B;

    %% Extract Quaternion from the K Matrix
    [eigvec, eigval] = eig(K_n1_n1);  % Compute eigenvalues and eigenvectors
    [~, max_idx] = max(diag(eigval));  % Find the largest eigenvalue
    q = eigvec(:, max_idx); % Extract corresponding eigenvector (quaternion)
    q = q / norm(q);  % Normalize the quaternionn

    %% Convert Quaternion to Direction Cosine Matrix (DCM)
    A_estimated = quat2dcm(q');  % Convert quaternion to DCM for output
end





