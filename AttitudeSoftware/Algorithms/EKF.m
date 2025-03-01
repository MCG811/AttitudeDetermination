function [sys_state, error_covariance] = EKF(R, past_state, past_error_cov, gyro_meas, dt, actual_meas, reference_meas)
    % EKF - Extended Kalman Filter for attitude determination
    %
    % INPUTS:
    % R              - Measurement noise covariance matrix
    % past_state     - Previous system state (quaternions, 4x1)
    % past_error_cov - Previous error covariance matrix (4x4)
    % gyro_meas      - Gyroscope measurements (3x1)
    % dt             - Time step (scalar)
    % actual_meas    - Current measurements in the body frame (6x1: concatenation of two 3x1 vectors)
    % reference_meas - Reference vectors in the inertial frame (3x2)
    %
    % OUTPUTS:
    % sys_state      - Updated system state (quaternions, 4x1)
    % error_covariance - Updated error covariance matrix (4x4)

    % Process noise covariance matrix (adjustable based on application)
    Q = diag(norm(gyro_meas)*ones(1,4));

    %% PREDICTION STEP
    % Construct matrix B for quaternion prediction based on gyroscope dynamics
    w1 = gyro_meas(1);
    w2 = gyro_meas(2);
    w3 = gyro_meas(3);
    B = 0.5 * [  0, -w1, -w2, -w3; 
                 w1,   0,  w3, -w2;
                 w2, -w3,   0,  w1;
                 w3,  w2, -w1,   0];

    % Predict the next state using quaternion kinematics
    predicted_state = past_state + B * past_state * dt;
    predicted_state = quatnormalize(predicted_state')'; % Normalize quaternion

    %
    % J = df(x,u)/dx = [ dq_dot0/dq0 dq_dot0/dq1 dq_dot0/dq2 dq_dot0/dq3;
    %                    dq_dot1/dq0 dq_dot1/dq1 dq_dot1/dq2 dq_dot1/dq3;
    %                    dq_dot2/dq0 dq_dot2/dq1 dq_dot2/dq2 dq_dot2/dq3;
    %                    dq_dot3/dq0 dq_dot3/dq1 dq_dot3/dq2 dq_dot3/dq3 ]
    %
    %   = (1/2)*[0 -w1 -w2 -w3; = B
    %            w1  0  w3 -w2;
    %            w2 -w3 0   w1;
    %            w3 w2 -w1  0 ]
    
    % Compute the state transition Jacobian matrix (F)
    F = eye(4) + dt * B;

    % Predict the error covariance matrix
    predicted_covariance = F * past_error_cov + past_error_cov * F' + Q;

    %% MEASUREMENT ESTIMATION
    % Convert predicted quaternion to a Direction Cosine Matrix (DCM)
    A_predicted = quat2dcm(predicted_state');


    % Estimate the expected measurements based on the predicted state
    h_estimated = [A_predicted * reference_meas(:, 1);  % Projection of the first reference vector
                    A_predicted * reference_meas(:, 2)]; % Projection of the second reference vector

    %% UPDATE STEP
    % Compute partial derivatives of the DCM with respect to the quaternions
    part_dev_Aref_q0 = 2 * [predicted_state(1),  predicted_state(4), -predicted_state(3);
                            -predicted_state(4), predicted_state(1),  predicted_state(2);
                             predicted_state(3), -predicted_state(2), predicted_state(1)];
    part_dev_Aref_q1 = 2 * [predicted_state(2),  predicted_state(3),  predicted_state(4);
                             predicted_state(3), -predicted_state(2),  predicted_state(1);
                             predicted_state(4), -predicted_state(1), -predicted_state(2)];
    part_dev_Aref_q2 = 2 * [-predicted_state(3), predicted_state(2), -predicted_state(1);
                             predicted_state(2), predicted_state(3),  predicted_state(4);
                             predicted_state(1), predicted_state(4), -predicted_state(3)];
    part_dev_Aref_q3 = 2 * [-predicted_state(4), predicted_state(1),  predicted_state(2);
                            -predicted_state(1), -predicted_state(4), predicted_state(3);
                             predicted_state(2),  predicted_state(3), predicted_state(4)];

    % Construct the measurement Jacobian matrix (H)
    H = [part_dev_Aref_q0 * reference_meas(:, 1), part_dev_Aref_q1 * reference_meas(:, 1), part_dev_Aref_q2 * reference_meas(:, 1), part_dev_Aref_q3 * reference_meas(:, 1);
         part_dev_Aref_q0 * reference_meas(:, 2), part_dev_Aref_q1 * reference_meas(:, 2), part_dev_Aref_q2 * reference_meas(:, 2), part_dev_Aref_q3 * reference_meas(:, 2)];

    % Compute the Kalman gain (K)
    K = predicted_covariance * H' / (H * predicted_covariance * H' + R);

    % Compute the measurement innovation (difference between actual and expected measurements)
    if norm(actual_meas(1:3, 1)) == 0
        innovation = h_estimated - h_estimated;  % Zero innovation in case of invalid measurements
    else
        innovation = actual_meas - h_estimated; % Otherwise, compute the difference
    end

    % Update the state estimate using the Kalman gain
    updated_state = predicted_state + K * innovation;
    updated_state = quatnormalize(updated_state')'; % Normalize updated quaternion

    % Update error covariance matrix
    updated_covariance = predicted_covariance - K * H * predicted_covariance;

    %% OUTPUTS
    sys_state = updated_state;          % Return updated system state
    error_covariance = updated_covariance; % Return updated error covariance matrix
end

