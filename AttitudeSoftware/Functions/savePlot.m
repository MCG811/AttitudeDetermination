function savePlot(figHandle, algorithmName, dt)
    % saveCustomPlot Saves a plot as a PNG file with a custom name in a specific folder

    % Inputs:
    %   figHandle    - Handle to the figure to save (use gcf for the current figure)
    %   algorithmName - String, name of the algorithm (e.g., 'Triad', 'EKF')
    %   dt           - Time step value to include in the file name (numeric)

    % Get the path of the "Functions" folder (where this script is located)
    functionFolderPath = fileparts(mfilename('fullpath'));

    % Move up one level to the "Software" folder
    softwareFolderPath = fileparts(functionFolderPath);

    % Define the "Figures" folder path inside "Software"
    folderPath = fullfile(softwareFolderPath, 'Figures');

    % Format the file name
    dtStr = strrep(num2str(dt), '.', '_'); % Replace dot with underscore
    fileName = sprintf('%s-%s.png', algorithmName, dtStr);

    % Ensure the folder exists, create if not
    if ~exist(folderPath, 'dir')
        mkdir(folderPath);
    end

    % Full path to save the file
    fullFilePath = fullfile(folderPath, fileName);

    % Ajustar el tamaño de la figura al de una pantalla estándar
    screenSize = get(0, 'ScreenSize'); % Tamaño de la pantalla en píxeles
    set(figHandle, 'Units', 'pixels', 'Position', screenSize);

    % Save the figure as PNG
    print(figHandle, fullFilePath, '-dpng', '-r300'); % 300 DPI for high quality
end
