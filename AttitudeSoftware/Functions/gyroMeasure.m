function [gyro_meas] = gyroMeasure(pitch, roll, angle_rates, gyro_var)

    % GYROMEASURE: Simulates gyroscope measurements with noise.
    %
    % This function computes the angular velocity in the body frame using pitch and roll angles.
    % It also adds Gaussian noise to simulate real-world gyroscope readings.
    %
    % INPUTS:
    % - pitch: Vector of pitch angles [radians].
    % - roll: Vector of roll angles [radians].
    % - angle_rates: 3xN matrix of angular rates in the inertial frame [radians/sec].
    % - gyro_var: Standard deviation of gyroscope noise.
    %
    % OUTPUT:
    % - gyro_meas: 3xN matrix of simulated gyroscope measurements [radians/sec].
    
    %% Initialize gyroscope measurement matrix
    gyro_meas = zeros(3,length(pitch));

    % Noise limits for bounding random noise values
    lim_inf = -2; % Límite inferior
    lim_sup = 2; % Límite superior

    %% Compute gyroscope measurements with noise
    for i = 1:length(pitch)

        % Transformation matrix from inertial to body frame
        C = [1, 0, -sin(pitch(i));
             0, cos(roll(i)), cos(pitch(i))*sin(roll(i));
             0, -sin(roll(i)), cos(pitch(i))*cos(roll(i))];
    
        % Compute angular velocity in body frame
        angular_velocity = C*angle_rates(:,i); % w_x, w_y and w_z in rad/s

        % Generate bounded Gaussian noise
        rand_vec = max(min(randn(3, 1), lim_sup), lim_inf);

        % Add noise to angular velocity
        gyro_meas(:,i) = angular_velocity + (gyro_var)*rand_vec;
        
    end
end

