% =====================================================================
% ---------------- FUNCTION TO CREATE CENTERED MENU (SINGLE SELECTION) ----------------
% =====================================================================
function choice = centeredMenu(prompt, options)
    % Create a figure window
    fig = figure('Name', prompt, ...
                 'NumberTitle', 'off', ...
                 'Position', [500 400 300 250], ...  % Increased height for spacing
                 'Resize', 'off', ...
                 'WindowStyle', 'modal');

    movegui(fig, 'center');  % Center the figure on the screen

    % Display prompt
    uicontrol('Style', 'text', ...
              'String', prompt, ...
              'Position', [20 180 260 40], ...
              'FontSize', 10, ...
              'FontWeight', 'bold');

    % Create buttons dynamically
    numOptions = length(options);
    btnHeight = 30;
    btnWidth = 250;
    spacing = 5;
    startY = 150;  % Adjusted Y position for better spacing

    for i = 1:numOptions
        uicontrol('Style', 'pushbutton', ...
                  'String', options{i}, ...
                  'Position', [25, startY - (i - 1) * (btnHeight + spacing), btnWidth, btnHeight], ...
                  'FontSize', 10, ...
                  'Callback', @(src, event) buttonCallback(i));
    end

    % Extra spacing before "Close" button
    closeButtonY = startY - numOptions * (btnHeight + spacing) - 20;

    % Styled "Close" button with red border and white text
    uicontrol('Style', 'pushbutton', ...
              'String', 'Close', ...
              'Position', [25, closeButtonY, btnWidth, btnHeight], ...
              'FontSize', 10, ...
              'ForegroundColor', 'red', ...   % Red border
              'BackgroundColor', 'white', ... % White background
              'FontWeight', 'bold', ...
              'Callback', @closeButtonCallback);

    % Wait for user input
    uiwait(fig);

    % Button callback function
    function buttonCallback(selection)
        choice = selection;
        uiresume(fig);
        close(fig);
    end

    % Close button callback function
    function closeButtonCallback(~, ~)
        disp('User closed the menu. Stopping execution.');
        close(fig);
        error('Execution stopped by user.');
    end
end

