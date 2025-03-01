function errorMetrics = calculateErrors(ref_euler_deg, est_euler_deg, error_rot_angle)
    % CALCULATEERRORS: Computes error metrics between reference and estimated Euler angles.
    %
    % This function calculates various error metrics such as MAE, MSE, RMSE, and maximum error
    % for yaw, pitch, and roll angles, as well as the rotation angle error in arcseconds.
    %
    % INPUTS:
    % - ref_euler_deg: Reference Euler angles (3xN matrix) [degrees]
    % - est_euler_deg: Estimated Euler angles (3xN matrix) [degrees]
    % - error_rot_angle: Rotation angle error at each time step (1xN vector) [arcseconds]
    %
    % OUTPUT:
    % - errorMetrics: Struct containing computed error metrics, including:
    %   - error: Angular differences between reference and estimated angles [arcseconds]
    %   - MAE: Mean Absolute Error for yaw, pitch, and roll [arcseconds]
    %   - MSE: Mean Squared Error for yaw, pitch, and roll [arcseconds^2]
    %   - RMSE: Root Mean Squared Error for yaw, pitch, and roll [arcseconds]
    %   - maxError: Maximum absolute error for yaw, pitch, roll, and rotation angle [arcseconds]
    %   - MAE_rot_angle: Mean Absolute Error of rotation angle [arcseconds]

    %% Compute angular differences and convert to arcseconds
    % The difference between reference and estimated angles is multiplied by 3600
    % to convert degrees to arcseconds.
    angular_differences.yaw = (ref_euler_deg(1,:) - est_euler_deg(1,:)) * 3600; % [arcsec]
    angular_differences.pitch = (ref_euler_deg(2,:) - est_euler_deg(2,:)) * 3600; % [arcsec]
    angular_differences.roll = (ref_euler_deg(3,:) - est_euler_deg(3,:)) * 3600; % [arcsec]

    %% Compute Mean Absolute Error (MAE)
    % The mean of absolute angular differences for each component.
    MAE.yaw = mean(abs(angular_differences.yaw), 2); % [arcsec]
    MAE.pitch = mean(abs(angular_differences.pitch), 2); % [arcsec]
    MAE.roll = mean(abs(angular_differences.roll), 2); % [arcsec]

    %% Compute Mean Squared Error (MSE)
    % The mean of squared angular differences for each component.
    MSE.yaw = mean(angular_differences.yaw .^2, 2); % [arcsec^2]
    MSE.pitch = mean(angular_differences.pitch .^2, 2); % [arcsec^2]
    MSE.roll = mean(angular_differences.roll .^2, 2); % [arcsec^2]

    %% Compute Root Mean Squared Error (RMSE)
    % The square root of the Mean Squared Error (MSE).
    RMSE.yaw = sqrt(MSE.yaw); % [arcsec]
    RMSE.pitch = sqrt(MSE.pitch); % [arcsec]
    RMSE.roll = sqrt(MSE.roll); % [arcsec]

    %% Compute Maximum Error (Max Error)
    % The maximum absolute angular difference for each component.
    maxError.yaw = max(abs(angular_differences.yaw), [], 2); % [arcsec]
    maxError.pitch = max(abs(angular_differences.pitch), [], 2); % [arcsec]
    maxError.roll = max(abs(angular_differences.roll), [], 2); % [arcsec]
    maxError.rot_angle = max(abs(error_rot_angle), [], 2); % [arcsec]

    %% Compute Mean Absolute Rotation Angle Error
    % The mean absolute value of the rotation angle error.
    MAE_rot_angle = mean(abs(error_rot_angle)); % [arcsec]

    %% Store results in a structured output
    errorMetrics = struct( ...
        'error', angular_differences, ... % Angular differences [arcsec]
        'MAE', MAE, ... % Mean Absolute Error per component [arcsec]
        'maxError', maxError, ... % Maximum absolute error per component [arcsec]
        'MSE', MSE, ... % Mean Squared Error [arcsec^2]
        'RMSE', RMSE, ... % Root Mean Squared Error [arcsec]
        'rot_angle', error_rot_angle, ... % Rotation angle error [arcsec]
        'MAE_rot_angle', MAE_rot_angle ... % Mean Absolute Error of rotation angle [arcsec]
        );
end


