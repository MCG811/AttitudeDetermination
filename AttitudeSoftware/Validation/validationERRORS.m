%% CalculateErrors Function Validation Script
% Este script valida la funcionalidad y definición correcta de la función calculateErrors

% Limpieza del entorno
disp('Initializing CalculateErrors Function Validation Script...');
clear; clc;

%% Configuración inicial
% Ángulos de Euler de referencia (en grados)
ref_euler_deg = [10; 5; 2]; % [Yaw; Pitch; Roll]

% Ángulos de Euler estimados (en grados)
est_euler_deg = [10.1; 4.8; 2.2]; % Introduciendo pequeños errores

% Errores de rotación (en arcsec)
numSamples = 100;
error_rot_angle = randn(1, numSamples) * 5; % Simulación de errores de rotación

%% Llamada a la función calculateErrors
errorMetrics = calculateErrors(ref_euler_deg, est_euler_deg, error_rot_angle);

%% Validar resultados
% Verificar que las métricas están definidas
assert(isfield(errorMetrics, 'error'), 'Error: No se encontró la métrica de diferencias angulares.');
assert(isfield(errorMetrics, 'MAE'), 'Error: No se encontró la métrica MAE.');
assert(isfield(errorMetrics, 'maxError'), 'Error: No se encontró la métrica de error máximo.');
assert(isfield(errorMetrics, 'MSE'), 'Error: No se encontró la métrica MSE.');
assert(isfield(errorMetrics, 'RMSE'), 'Error: No se encontró la métrica RMSE.');
assert(isfield(errorMetrics, 'MAE_rot_angle'), 'Error: No se encontró la métrica MAE del ángulo de rotación.');

% Imprimir métricas calculadas
disp('Métricas de error calculadas:');
disp(errorMetrics);

%% Validación de resultados esperados
% Verificar que las métricas están dentro de rangos esperados
expected_error = (ref_euler_deg - est_euler_deg) * 3600; % Diferencias esperadas
assert(norm(errorMetrics.error - expected_error) < 1e-6, 'Error: Diferencias angulares calculadas incorrectamente.');

% Validar MAE
expected_MAE = mean(abs(expected_error), 2);
assert(norm(errorMetrics.MAE - expected_MAE) < 1e-6, 'Error: MAE calculado incorrectamente.');

% Validar Max Error
expected_max_error = max(abs(expected_error), [], 2);
assert(norm(errorMetrics.maxError - expected_max_error) < 1e-6, 'Error: Error máximo calculado incorrectamente.');

% Validar MSE
expected_MSE = mean(expected_error.^2, 2);
assert(norm(errorMetrics.MSE - expected_MSE) < 1e-6, 'Error: MSE calculado incorrectamente.');

% Validar RMSE
expected_RMSE = sqrt(expected_MSE);
assert(norm(errorMetrics.RMSE - expected_RMSE) < 1e-6, 'Error: RMSE calculado incorrectamente.');

% Validar MAE del ángulo de rotación
expected_MAE_rot_angle = mean(abs(error_rot_angle));
assert(abs(errorMetrics.MAE_rot_angle - expected_MAE_rot_angle) < 1e-6, 'Error: MAE del ángulo de rotación calculado incorrectamente.');

%% Finalización del script
disp('Validación completada. Todos los cálculos son correctos.');
