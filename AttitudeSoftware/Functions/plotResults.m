function plotResults(vec_time, time_vec_plot, errorMetrics, true_euler, est_euler, algorithm, parameter_name, parameter_value)
    % Generates various plots to evaluate the performance of the algorithm.

    %% Font size settings for better visualization
    titleFontSize = 20; % Title font size
    axisLabelFontSize = 18; % Axis label font size
    tickFontSize = 16; % Tick label font size
    legendFontSize = 12; % Legend font size
    
    %% ---- PLOT 1: Estimated vs. True Yaw ----
    fig = figure;
    subplot(3, 2, 1); % Create a subplot (3 rows, 2 columns, position 1)
    plot(vec_time, est_euler(1, :), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Estimated');
    hold on;
    plot(time_vec_plot, true_euler(1, :), 'r', 'DisplayName', 'Reference');
    ylabel('Yaw [deg]', 'FontSize', axisLabelFontSize);
    % Add a title based on the parameter being analyzed
    if strcmp(parameter_name, 'fading_memory')
        title(['True vs Estimated Euler Angles (' algorithm ' \mu = ' num2str(parameter_value) ')'], 'FontSize', titleFontSize);
    elseif strcmp(parameter_name, 'dt')
        title(['True vs Estimated Euler Angles (' algorithm ' dt = ' num2str(parameter_value) 's)'], 'FontSize', titleFontSize);
    end
    legend ('Location','southeast', 'NumColumns', 2, 'FontSize', legendFontSize);
    set(gca, 'FontSize', tickFontSize);
    
    %% ---- PLOT 2: Estimated vs. True Pitch ----
    subplot(3, 2, 3);
    plot(vec_time, est_euler(2, :), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Estimated');
    hold on
    plot(time_vec_plot, true_euler(2, :), 'r', 'DisplayName', 'Reference');
    ylabel('Pitch [deg]', 'FontSize', axisLabelFontSize);
    set(gca, 'FontSize', tickFontSize);

    %% ---- PLOT 3: Estimated vs. True Roll ----
    subplot(3, 2, 5);
    plot(vec_time, est_euler(3, :), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Estimated');
    hold on
    plot(time_vec_plot, true_euler(3, :), 'r', 'DisplayName', 'Reference');
    xlabel('Time [s]', 'FontSize', axisLabelFontSize);
    ylabel('Roll [deg]', 'FontSize', axisLabelFontSize);
    set(gca, 'FontSize', tickFontSize);
    

    %% ---- ERROR PLOTS ----
   % ---- PLOT 4: Error Yaw ----
    subplot(3, 2, 2);
    plot(vec_time, errorMetrics.error.yaw, 'b', 'DisplayName', 'Angular difference');
    hold on;
    yline(errorMetrics.MAE.yaw, 'r', 'LineWidth', 2, 'DisplayName', 'Mean absolute error');
    [max_val, max_idx] = max(abs(errorMetrics.error.yaw));
    yline(errorMetrics.maxError.yaw, 'Color', [0 0.5 0], 'LineWidth', 1.5, 'LineStyle', '--', 'DisplayName', 'Max. error');
    plot(vec_time(max_idx), max_val, '*', 'MarkerSize', 10, 'Color', [0 0.5 0], 'DisplayName', 'Maximum');
    text(vec_time(max_idx), max_val, sprintf('%.2f', max_val), 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', 'FontSize', tickFontSize, 'Color', [0 0.5 0]);
    ylabel('Yaw [arcsec]', 'FontSize', axisLabelFontSize);
    if strcmp(parameter_name, 'fading_memory')
        title(['True vs Estimated Euler Angles (' algorithm ' \mu = ' num2str(parameter_value) ')'], 'FontSize', titleFontSize);
    elseif strcmp(parameter_name, 'dt')
        title(['True vs Estimated Euler Angles (' algorithm ' dt = ' num2str(parameter_value) 's)'], 'FontSize', titleFontSize);
    end
    legend ('Location','southeast', 'NumColumns', 2, 'FontSize', legendFontSize);
    set(gca, 'FontSize', tickFontSize);
    
   % ---- PLOT 4: Error Pitch ----
    subplot(3, 2, 4);
    plot(vec_time, errorMetrics.error.pitch, 'b', 'DisplayName', 'Angular difference');
    hold on;
    yline(errorMetrics.MAE.pitch, 'r', 'LineWidth', 2, 'DisplayName', 'Mean absolute error');
    [max_val, max_idx] = max(abs(errorMetrics.error.pitch));
    yline(errorMetrics.maxError.pitch, 'Color', [0 0.5 0], 'LineWidth', 1.5, 'LineStyle', '--', 'DisplayName', 'Max. error');
    plot(vec_time(max_idx), max_val, '*', 'MarkerSize', 10, 'Color', [0 0.5 0], 'DisplayName', 'Maximum');
    text(vec_time(max_idx), max_val, sprintf('%.2f', max_val), 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', 'FontSize', tickFontSize, 'Color', [0 0.5 0]);
    ylabel('Pitch [arcsec]', 'FontSize', axisLabelFontSize);
    set(gca, 'FontSize', tickFontSize);
    
    % ---- PLOT 4: Error Roll ----
    subplot(3, 2, 6);
    plot(vec_time, errorMetrics.error.roll, 'b', 'DisplayName', 'Angular difference');
    hold on;
    yline(errorMetrics.MAE.roll, 'r', 'LineWidth', 2, 'DisplayName', 'Mean absolute error');
    [max_val, max_idx] = max(abs(errorMetrics.error.roll));
    yline(errorMetrics.maxError.roll, 'Color', [0 0.5 0], 'LineWidth', 1.5, 'LineStyle', '--', 'DisplayName', 'Max. error');
    plot(vec_time(max_idx), max_val, '*', 'MarkerSize', 10, 'Color', [0 0.5 0], 'DisplayName', 'Maximum');
    text(vec_time(max_idx), max_val, sprintf('%.2f', max_val), 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', 'FontSize', tickFontSize, 'Color', [0 0.5 0]);
    ylabel('Roll [arcsec]', 'FontSize', axisLabelFontSize);
    xlabel('Time [s]', 'FontSize', axisLabelFontSize);
    set(gca, 'FontSize', tickFontSize);

    savePlot(fig, algorithm, parameter_value)

    %% Save the plot
    savePlot(fig, algorithm, parameter_value)

    % Plot: Estimated VS. True Quaternion components
%     figure;
%     labels = {'q0', 'q1', 'q2', 'q3'};
%     for i = 1:4
%         subplot(4, 1, i);
%         plot(vec_time, true_quaternion(i, :), 'r', 'DisplayName', 'Reference');
%         hold on;
%         plot(vec_time, est_quaternion(i, :), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Estimated');
%         ylabel(labels{i});
%         legend;
%         if i == 1
%             title(['Quaternion components (' algorithm ')']);
%         end
%     end
%     xlabel('Time [s]');

%     % Plot: Angular difference during time [arcsec]
%     figure;
%     
%     % Yaw
%     subplot(4, 1, 1);
%     plot(vec_time, errorMetrics.error.yaw, 'b', 'DisplayName', 'Angular difference');
%     hold on;
%     yline(errorMetrics.MAE.yaw, 'r', 'LineWidth', 2, 'DisplayName', 'Mean absolute error'); % Línea roja para el MAE
%     [max_val, max_idx] = max(abs(errorMetrics.error.yaw));
%     yline(errorMetrics.maxError.yaw, 'Color', [0 0.5 0], 'LineWidth', 1.5, 'LineStyle', '--', 'DisplayName', 'Max. error'); % Línea verde oscura
%     plot(vec_time(max_idx), max_val, '*', 'MarkerSize', 10, 'Color', [0 0.5 0], 'DisplayName', 'Maximum'); % Asterisco verde oscuro
%     text(vec_time(max_idx), max_val, sprintf('%.2f', max_val), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 10, 'Color', [0 0.5 0]); % Etiqueta verde oscura
%     ylabel('Yaw angle [arcsec]');
%     title(['Attitude estimation error over time (' algorithm ')']);
%     legend;
%     
%     % Pitch
%     subplot(4, 1, 2);
%     plot(vec_time, errorMetrics.error.pitch, 'b', 'DisplayName', 'Angular difference');
%     hold on;
%     yline(errorMetrics.MAE.pitch, 'r', 'LineWidth', 2, 'DisplayName', 'Mean absolute error'); % Línea roja para el MAE
%     [max_val, max_idx] = max(abs(errorMetrics.error.pitch));
%     yline(errorMetrics.maxError.pitch, 'Color', [0 0.5 0], 'LineWidth', 1.5, 'LineStyle', '--', 'DisplayName', 'Max. error'); % Línea verde oscura
%     plot(vec_time(max_idx), max_val, '*', 'MarkerSize', 10, 'Color', [0 0.5 0], 'DisplayName', 'Maximum'); % Asterisco verde oscuro
%     text(vec_time(max_idx), max_val, sprintf('%.2f', max_val), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 10, 'Color', [0 0.5 0]); % Etiqueta verde oscura
%     ylabel('Pitch angle [arcsec]');
%     legend;
%     
%     % Roll
%     subplot(4, 1, 3);
%     plot(vec_time, errorMetrics.error.roll, 'b', 'DisplayName', 'Angular difference');
%     hold on;
%     yline(errorMetrics.MAE.roll, 'r', 'LineWidth', 2, 'DisplayName', 'Mean absolute error'); % Línea roja para el MAE
%     [max_val, max_idx] = max(abs(errorMetrics.error.roll));
%     yline(errorMetrics.maxError.roll, 'Color', [0 0.5 0], 'LineWidth', 1.5, 'LineStyle', '--', 'DisplayName', 'Max. error'); % Línea verde oscura
%     plot(vec_time(max_idx), max_val, '*', 'MarkerSize', 10, 'Color', [0 0.5 0], 'DisplayName', 'Maximum'); % Asterisco verde oscuro
%     text(vec_time(max_idx), max_val, sprintf('%.2f', max_val), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 10, 'Color', [0 0.5 0]); % Etiqueta verde oscura
%     ylabel('Roll [arcsec]');
%     legend;
%     
%     % Principal angle
%     subplot(4, 1, 4);
%     plot(vec_time, errorMetrics.rot_angle, 'b', 'DisplayName', 'Angular difference');
%     hold on;
%     yline(errorMetrics.MAE_rot_angle, 'r', 'LineWidth', 2, 'DisplayName', 'Mean absolute error'); % Línea roja para el MAE
%     [max_val, max_idx] = max(abs(errorMetrics.rot_angle));
%     yline(max_val, 'Color', [0 0.5 0], 'LineWidth', 1.5, 'LineStyle', '--', 'DisplayName', 'Max. error'); % Línea verde oscura
%     plot(vec_time(max_idx), max_val, '*', 'MarkerSize', 10, 'Color', [0 0.5 0], 'DisplayName', 'Maximum'); % Asterisco verde oscuro
%     text(vec_time(max_idx), max_val, sprintf('%.2f', max_val), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 10, 'Color', [0 0.5 0]); % Etiqueta verde oscura
%     ylabel('Principal angle [arcsec]');
%     xlabel('Time [s]');
%     legend;

%     % Plot: Execution time for each iteration
%     figure;
%     plot(vec_time / dt, time_algorithm, '-', 'Color', 'r', 'DisplayName', 'Execution time');
%     yline(mean_time_algorithm, 'k', sprintf('%.2f', mean_time_algorithm), 'DisplayName', 'Mean value');
%     xlabel('Iteration number');
%     ylabel('Time [s]');
%     title(['Execution time for each iteration (' algorithm ')']);

end
