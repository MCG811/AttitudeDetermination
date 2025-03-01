function [scenario] = getScenario()
    % ---------------- SCENARIO SELECTION ----------------
    scenarioOptions = {'Nominal operations', 'Reorientation operations', 'Failure operations'};
    scenarioIndex = centeredMenu('Select the scenario:', scenarioOptions);
    scenario = scenarioOptions{scenarioIndex};

end

