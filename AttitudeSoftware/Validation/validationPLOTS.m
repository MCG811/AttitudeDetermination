%% PlotResults Function Validation Script
% Este script valida la funcionalidad y definición correcta de la función plotResults

% Limpieza del entorno
disp('Initializing PlotResults Function Validation Script...');
clear; clc;

%% Configuración inicial
% Simular datos de entrada
numSamples = 100;
vec_time = linspace(0, 10, numSamples); % Vector de tiempo [s]
dt = vec_time(2) - vec_time(1); % Paso de tiempo [s]
time_algorithm = rand(1, numSamples) * 0.01; % Tiempos de ejecución individuales [s]
mean_time_algorithm = mean(time_algorithm); % Tiempo medio [s]

% Generar datos simulados para métricas de error
errorMetrics.error = randn(3, numSamples) * 10; % Diferencias angulares [arcsec]
errorMetrics.MAE = mean(abs(errorMetrics.error), 2); % MAE [arcsec]
errorMetrics.maxError = max(abs(errorMetrics.error), [], 2); % Error máximo [arcsec]
errorMetrics.MAE_rot_angle = mean(abs(randn(1, numSamples) * 5)); % MAE del ángulo de rotación [arcsec]
errorMetrics.errorRotation = randn(1, numSamples) * 5; % Errores de rotación [arcsec]

% Generar datos simulados para cuaterniones y ángulos de Euler
true_euler_arcsec = [linspace(0, 3600, numSamples); linspace(0, 1800, numSamples); linspace(0, 900, numSamples)];
est_euler_arcsec = true_euler_arcsec + randn(3, numSamples) * 50; % Con errores simulados

true_quaternion = [ones(1, numSamples); zeros(3, numSamples)];
est_quaternion = true_quaternion + randn(4, numSamples) * 0.01; % Con errores simulados

algorithm = 'TRIAD'; % Nombre del algoritmo

%% Llamada a la función plotResults
plotResults(vec_time, dt, time_algorithm, mean_time_algorithm, errorMetrics, true_euler_arcsec, est_euler_arcsec, true_quaternion, est_quaternion, algorithm);

%% Validar visualmente
% Este script genera varios gráficos para verificar visualmente los resultados:
% - Comparación de componentes cuaternión estimados vs. reales
% - Comparación de ángulos de Euler estimados vs. reales
% - Errores angulares en el tiempo
% - Tiempos de ejecución por iteración

disp('Validación completada. Revisa los gráficos generados para confirmar el funcionamiento.');