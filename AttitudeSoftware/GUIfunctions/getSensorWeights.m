% =========================================================
% FUNCTION TO GET SENSOR WEIGHTS (Sun Sensor & Magnetometer)
% =========================================================
function sensorWeights = getSensorWeights(algorithmName)
    valid = false;
    while ~valid
        % Get user input
        inputStr = centeredTextInput(sprintf('Enter sensor weights for %s [Sun Sensor, Magnetometer] (0 to 1):', algorithmName), '0.5 0.5');
        sensorWeights = str2num(inputStr); % Convert to numeric array

        % Validate input
        if isempty(sensorWeights) || length(sensorWeights) ~= 2 || any(sensorWeights < 0) || any(sensorWeights > 1)
            h = errordlg('Invalid input. Enter two values between 0 and 1 (e.g., "0.6 0.4").', 'Input Error');
            uiwait(h);
        else
            valid = true;
        end
    end
end