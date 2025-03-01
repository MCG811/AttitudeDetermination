function algorithmConfig = configureAlgorithms(selectedAlgorithms)
    % Initialize configuration structure
    algorithmConfig = struct();

    % Loop through each selected algorithm
    for i = 1:length(selectedAlgorithms)
        algorithm = selectedAlgorithms{i};

        switch algorithm
            case 'TRIAD'
                % Choose anchor vector
                anchorOptions = {'Sun Vector', 'Magnetic Field Vector'};
                anchorIndex = centeredMenu('Select anchor vector for TRIAD:', anchorOptions);
                algorithmConfig.TRIAD.anchorVector = anchorOptions{anchorIndex};

            case 'QUEST'
                % Choose sensor weights (0 to 1)
                sensorWeights = getSensorWeights('QUEST');
                algorithmConfig.QUEST.sensorWeights = sensorWeights;

            case 'REQUEST'
                % Choose fading memory factor
                fadingMemoryFactor = getFadingMemoryFactor();
                sensorWeights = getSensorWeights('REQUEST');
                algorithmConfig.REQUEST.fadingMemoryFactor = fadingMemoryFactor;
                algorithmConfig.REQUEST.sensorWeights = sensorWeights;

            case 'OptREQUEST'
                % Choose sensor weights
                sensorWeights = getSensorWeights('OptREQUEST');
                algorithmConfig.OptREQUEST.sensorWeights = sensorWeights;
        end
    end
end
