function [ypr_matrix, ypr_rates] = extract_ypr_matrix(file_path, time_step,max_duration)
    % EXTRACT_YPR_MATRIX: Extracts sampled yaw, pitch, and roll angles from an Excel file.
    %
    % This function reads time-series attitude data (yaw, pitch, roll) from an Excel file,
    % resamples the data at a specified time step, and converts angles from degrees to radians.
    %
    % INPUTS:
    % - file_path: Path to the Excel file containing the attitude data.
    % - time_step: Desired sampling time step [seconds].
    % - max_duration: Maximum duration for which the data should be extracted [seconds].
    %
    % OUTPUTS:
    % - ypr_matrix: 3xN matrix containing yaw, pitch, and roll angles [radians].
    % - ypr_rates: 3xN matrix containing yaw, pitch, and roll angular rates [radians/sec].

    %% Load data from the Excel file
    data = readtable(file_path);

    % Extract relevant columns
    time = data.("Time");        % Time values
    roll = data.("Roll");        % Roll angles [degrees]
    pitch = data.("Pitch");      % Pitch angles [degrees]
    yaw = data.("Yaw");          % Yaw angles [degrees]
    dRoll = data.('dRoll_dt');   % Roll rate [degrees/sec]
    dPitch = data.('dPitch_dt'); % Pitch rate [degrees/sec]
    dYaw = data.('dYaw_dt');     % Yaw rate [degrees/sec]
    
    %% Compute number of required samples
    % Determine how many samples fit within max_duration
    num_samples = floor(max_duration / time_step);
    
    % Compute the sampling interval (number of rows to skip)
    sampling_interval = round(time_step / (time(2) - time(1)));
    
    % Ensure that we do not exceed the available data length
    max_possible_indices = floor(height(data) / sampling_interval);
    actual_samples = min(num_samples, max_possible_indices);

    % Generate sample indices with appropriate spacing
    indices = 1:sampling_interval:(actual_samples*sampling_interval +1);
    
    %% Extract sampled data
    sampled_yaw = yaw(indices);       % Yaw angles
    sampled_pitch = pitch(indices);   % Pitch angles
    sampled_roll = roll(indices);     % Roll angles
    sampled_dyaw = dYaw(indices);     % Yaw rate
    sampled_dpitch = dPitch(indices); % Pitch rate
    sampled_droll = dRoll(indices);   % Roll rate

    %% Convert angles and rates from degrees to radians
    ypr_matrix = [sampled_yaw'; sampled_pitch'; sampled_roll'] * pi/180; % Convert angles
    ypr_rates = [sampled_dyaw'; sampled_dpitch'; sampled_droll'] * pi/180; % Convert rates
end

