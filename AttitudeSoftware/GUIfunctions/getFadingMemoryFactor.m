% =========================================================
% FUNCTION TO GET FADING MEMORY FACTOR (0 to 1)
% =========================================================
function fadingMemoryFactor = getFadingMemoryFactor()
    valid = false;
    while ~valid
        % Get user input
        inputStr = centeredTextInput('Enter fading memory factor for REQUEST (0 to 1):', '0.001');
        fadingMemoryFactor = str2double(inputStr); % Convert to number

        % Validate input
        if isnan(fadingMemoryFactor) || fadingMemoryFactor < 0 || fadingMemoryFactor > 1
            h = errordlg('Invalid input. Enter a value between 0 and 1.', 'Input Error');
            uiwait(h);
        else
            valid = true;
        end
    end
end
