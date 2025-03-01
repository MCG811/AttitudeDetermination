function [maxTime] = getMaxTime()
    % ---------------- MAXIMUM SIMULATION TIME ----------------
    valid = false; % Flag for valid input
    while ~valid
        maxTime = centeredTextInput('Select maximum simulation time (1 - 120) [s]:', '120');
        maxTime = str2double(maxTime);  % Convert to number
    
        % Check if input is a number and within the allowed range
        if isnan(maxTime) || maxTime < 1 || maxTime > 120
            % Display error message and wait for user to close it
            h = errordlg('Invalid input. Please enter a number between 1 and 120.', 'Input Error');
            uiwait(h);  % Wait until the user closes the error message
        else
            valid = true; % Input is valid, exit loop
        end
    end
end

