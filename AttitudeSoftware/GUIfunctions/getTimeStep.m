function [dtValue] = getTimeStep()
    valid = false; % Flag for valid input
    
    while ~valid
        % Ask the user for input
        dtValue = centeredTextInput('Select time step value [s]:', '0.1');
        dtValue = str2num(dtValue); % Convert string to numeric array
    
        % Define the valid range of values
        validDtRange = 0.1:0.1:2;
    
        % Check if input is empty, contains NaN, or has invalid numbers
        if isempty(dtValue) || any(isnan(dtValue)) || any(~ismember(dtValue, validDtRange))
            % Display error message and wait until user closes it
            h = errordlg('Invalid input. Please enter value from 0.1 to 2 in steps of 0.1.', 'Input Error');
            uiwait(h);  % Wait for user to close the error dialog
        else
            valid = true; % Input is valid, exit loop
        end
    end
end

