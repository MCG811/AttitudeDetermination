clc; clear;

% Get the directory where the script is located
rootDir = fileparts(mfilename('fullpath'));

% Add necessary folders to the MATLAB path for accessing functions and classes
addpath(genpath(fullfile(rootDir, 'GUIfunctions')));
addpath(genpath(fullfile(rootDir, 'Algorithms')));
addpath(genpath(fullfile(rootDir, 'Functions')));
addpath(genpath(fullfile(rootDir, 'Classes')));
addpath(genpath(fullfile(rootDir, 'Analysis')));


% ---------------- ANALYSIS SELECTION ----------------
% Define analysis options
analysisOptions = {'Time Step', 'Fading Memory', 'Algorithms'};
% Display menu for user to select analysis type
analysisIndex = centeredMenu('What do you want to analyze? (Select ONE)', analysisOptions);
selectedAnalysis = analysisOptions{analysisIndex};

% ---------------- CONDITIONAL INPUTS ----------------
% Initialize variables for different analysis types
dtValue = [];
selectedAlgorithms = {};
fadingMemoryValues = 'N/A';

% List of available attitude determination algorithms
algorithmOptions = {'TRIAD', 'QUEST', 'REQUEST', 'EKF', 'OptREQUEST', 'OptTRIAD', 'Fusion E-T', 'Fusion E-Q'};

% Handling different types of analysis
if strcmp(selectedAnalysis, 'Time Step')
    
    % ---------- Time Step Selection ----------
    valid = false; % Flag for valid input
    
    while ~valid
        % Ask the user for input
        dtValue = centeredTextInput('Select time step values [s]:', '0.1 0.2 0.5');
        dtValue = str2num(dtValue); % Convert string input to numeric array
        validDtRange = 0.1:0.1:2; % Define valid range of values
    
        % Validate input
        if isempty(dtValue) || any(isnan(dtValue)) || any(~ismember(dtValue, validDtRange))
            % Display error message and wait until user closes it
            h = errordlg('Invalid input. Please enter values from 0.1 to 2 in steps of 0.1.', 'Input Error');
            uiwait(h);  % Wait for user to close the error dialog
        else
            valid = true; % Input is valid, exit loop
        end
    end

    % Get maximum simulation time
    [maxTime] = getMaxTime();

    % ---------- Algorithm Selection ----------
    selectedAlgorithms = centeredMultiSelect('Select algorithms to use:', {'TRIAD', 'QUEST', 'REQUEST', 'EKF'});
    if ismember('Fusion E-T', selectedAlgorithms)
        selectedAlgorithms = unique([selectedAlgorithms, {'EKF', 'TRIAD'}]); % Add EKF & TRIAD
    end
    if ismember('Fusion E-Q', selectedAlgorithms)
        selectedAlgorithms = unique([selectedAlgorithms, {'EKF', 'QUEST'}]); % Add EKF & QUEST
    end

    % Configure selected algorithm
    algorithmConfig = configureAlgorithms(selectedAlgorithms);

    % Get additional scenario and noise settings
    [scenario] = getScenario();
    [noise] = getNoise();

elseif strcmp(selectedAnalysis, 'Fading Memory')
    % ---------- Fading Memory Selection ----------

    valid = false; % Flag for valid input
    while ~valid
        % Ask the user for input
        fadingMemoryValues = centeredTextInput('Select fading memory values for REQUEST:', '0.001 0.01 0.1');
        fadingMemoryValues = str2num(fadingMemoryValues); % Convert string to numeric array
    
        % Validate input
        if isempty(fadingMemoryValues) || any(isnan(fadingMemoryValues)) || any(fadingMemoryValues < 0.001 | fadingMemoryValues > 1)
            % Display error message and wait until user closes it
            h = errordlg('Invalid input. Please enter values between 0.001 and 1 (e.g., 0.001 0.01 0.1).', 'Input Error');
            uiwait(h);  % Wait for user to close the error dialog
        else
            valid = true; % Input is valid, exit loop
        end
    end

    % Define algorithm settings for REQUEST
    selectedAlgorithms = {'REQUEST'};
    algorithmConfig.REQUEST.fadingMemoryFactor = fadingMemoryValues;
    algorithmConfig.REQUEST.sensorWeights = getSensorWeights('REQUEST');

    % Get additional settings
    [dtValue] = getTimeStep();
    [maxTime] = getMaxTime();

    [scenario] = getScenario();
    [noise] = getNoise();
    

elseif strcmp(selectedAnalysis, 'Algorithms')

    % ---------- Algorithm Selection ----------
    selectedAlgorithms = centeredMultiSelect('Select algorithms to use:', algorithmOptions);
    
    % Ensure dependencies are included
    if ismember('Fusion E-T', selectedAlgorithms)
        selectedAlgorithms = unique([selectedAlgorithms, {'EKF', 'TRIAD'}]); % Add EKF & TRIAD
    end
    if ismember('Fusion E-Q', selectedAlgorithms)
        selectedAlgorithms = unique([selectedAlgorithms, {'EKF', 'QUEST'}]); % Add EKF & QUEST
    end

    % Configure selected algorithms
    algorithmConfig = configureAlgorithms(selectedAlgorithms);

    % Get additional settings
    [dtValue] = getTimeStep();
    [maxTime] = getMaxTime();

    [scenario] = getScenario();
    [noise] = getNoise();
end

% Get the number of simulation runs
[runs] = getRuns();

% ---------------- DISPLAY USER SELECTION ----------------
disp('-------------------- USER SELECTION --------------------');

fprintf('- Analysis: %s\n', selectedAnalysis);

dtString = strjoin(arrayfun(@(x) sprintf('%.1f', x), dtValue, 'UniformOutput', false), ', ');
fprintf('- Time Step Values: %s s\n', dtString);
fprintf('- Max Time: %.2f s\n', maxTime);

fprintf('- Selected Algorithms: \n')

% Display selected algorithms and their configurations
for i = 1:length(selectedAlgorithms)
    algorithm = selectedAlgorithms{i};
    fprintf('\t- Name: %s\n', algorithm)
    switch algorithm
        case 'TRIAD'
            fprintf('\t\t- Anchor Vector: %s\n', algorithmConfig.TRIAD.anchorVector)
            if strcmp(algorithmConfig.TRIAD.anchorVector,'Sun Vector')
                algorithmConfig.TRIAD.anchorVector = 1;
            else
                algorithmConfig.TRIAD.anchorVector = 2;
            end
    
        case 'QUEST'
            dtString = strjoin(arrayfun(@(x) sprintf('%.3f', x), algorithmConfig.QUEST.sensorWeights, 'UniformOutput', false), ', ');
            fprintf('\t\t- Sensor Weights [Sun, Magnetometer]: %s\n', dtString);
    
        case 'REQUEST'
            dtString = strjoin(arrayfun(@(x) sprintf('%.3f', x), algorithmConfig.REQUEST.fadingMemoryFactor, 'UniformOutput', false), ', ');
            fprintf('\t\t- Fading memory factor: %s\n', dtString)
            dtString = strjoin(arrayfun(@(x) sprintf('%.3f', x), algorithmConfig.REQUEST.sensorWeights, 'UniformOutput', false), ', ');
            fprintf('\t\t- Sensor Weights [Sun, Magnetometer]: %s\n', dtString);
    
        case 'OptREQUEST'
            dtString = strjoin(arrayfun(@(x) sprintf('%.3f', x), algorithmConfig.OptREQUEST.sensorWeights, 'UniformOutput', false), ', ');
            fprintf('\t\t- Sensor Weights [Sun, Magnetometer]: %s\n', dtString);
    end
end

fprintf('- Scenario: %s\n', scenario);

fprintf('- Noise: %s\n', noise);

fprintf('- Number of runs: %1f \n', runs);

disp('-------------------------------------------------------------------');

% ---------------- RUN SIMULATION FUNCTION ----------------
% Call the appropriate analysis function based on selected analysis type
if strcmp(selectedAnalysis, 'Algorithms')
    results = algorithm_analysis(dtValue,maxTime, scenario, noise, runs, selectedAlgorithms, algorithmConfig);
elseif strcmp(selectedAnalysis, 'Time Step')
    results = timeStep_analysis(dtValue, maxTime, scenario, noise, runs, selectedAlgorithms, algorithmConfig);
elseif strcmp(selectedAnalysis, 'Fading Memory')
    results = fadingMemory_analysis(dtValue, maxTime, scenario, noise, runs, selectedAlgorithms, algorithmConfig);
end
