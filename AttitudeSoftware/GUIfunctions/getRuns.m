function [runs] = getRuns()
    valid = false; % Flag for valid input
    
    while ~valid
        % Ask the user for input
        runs = centeredTextInput('Select number of runs [-]:', '1');
        runs = str2num(runs); % Convert string to numeric array
    
        % Check if input is empty, contains NaN, or has invalid numbers
        if isempty(runs) || any(isnan(runs)) 
            % Display error message and wait until user closes it
            h = errordlg('Invalid input. Please enter a value', 'Input Error');
            uiwait(h);  % Wait for user to close the error dialog
        else
            valid = true; % Input is valid, exit loop
        end
    end
end

