function saveToExcel(results, fileName)
    % saveToExcel Saves multiple algorithm results to an Excel file in the "Figures" folder inside "Software"

    % Inputs:
    %   results  - A struct where each field is an AlgorithmName containing a table
    %   fileName - String, name of the Excel file (e.g., 'Results.xlsx')

    % Get the path of the "Functions" folder (where this script is located)
    functionFolderPath = fileparts(mfilename('fullpath'));

    % Move up one level to the "Software" folder
    softwareFolderPath = fileparts(functionFolderPath);

    % Define the "Figures" folder path inside "Software"
    folderPath = fullfile(softwareFolderPath, 'ExcelResults');

    % Ensure the folder exists, create if not
    if ~exist(folderPath, 'dir')
        mkdir(folderPath);
    end

    % Full path to save the Excel file
    fullFilePath = fullfile(folderPath, fileName);

    % Get the algorithm names (fields in the struct)
    algorithmNames = fieldnames(results);

    % Loop through each algorithm
    for i = 1:length(algorithmNames)
        algorithmName = algorithmNames{i}; % Get the algorithm name
        dataTable = results.(algorithmName); % Extract the table
        
        % Write the table to an Excel sheet named after the algorithm
        writetable(dataTable, fullFilePath, 'Sheet', algorithmName, 'WriteRowNames', true);
    end

    % Confirmation message
    fprintf('Data saved to Excel file: %s\n', fullFilePath);
end
