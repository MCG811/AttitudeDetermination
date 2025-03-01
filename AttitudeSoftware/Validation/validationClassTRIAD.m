%% TRIAD Algorithm Class Validation Script
% Este script valida la funcionalidad y definición correcta de la clase Triad_Algorithm

% Limpieza del entorno
clear; clc;
disp('Initializing TRIAD Algorithm Class Validation Script...');

%% Generar datos simulados
% Vectores en el marco inercial
inertial_vector1 = [1; 0; 0]; % Vector de referencia 1
inertial_vector2 = [0; 1; 0]; % Vector de referencia 2

% Normalización de los vectores inerciales
inertial_vector1 = inertial_vector1 / norm(inertial_vector1);
inertial_vector2 = inertial_vector2 / norm(inertial_vector2);

% Definir una orientación conocida (matriz de rotación)
angle = pi / 4; % Rotación de 45 grados
rotation_axis = [0; 0; 1]; % Eje de rotación (z)
rotation_axis = rotation_axis / norm(rotation_axis); % Normalizar el eje
A_true = axang2rotm([rotation_axis' angle]); % Matriz de rotación

% Transformar los vectores al marco del cuerpo
body_vector1 = A_true * inertial_vector1;
body_vector2 = A_true * inertial_vector2;

% Crear datos simulados para el cuerpo e inerciales
bodyData = cat(3, [body_vector1, body_vector2]', [body_vector1, body_vector2]'); % Tamaño: 2x3xN
refData = [inertial_vector1 inertial_vector2]'; % Tamaño: 2x3   

% Datos de referencia (matrices de rotación reales)
numSamples = 2;
A_ref = repmat(A_true, [1, 1, numSamples]);

%% Inicializar la clase Triad_Algorithm
anchor_vector = 1; % Usar el primer vector como base
triadAlgorithm = Triad_Algorithm(anchor_vector);

%% Ejecutar el algoritmo TRIAD
results = triadAlgorithm.run(bodyData, refData, A_ref);

%% Verificar resultados
% Verificar que los resultados estén definidos
assert(isfield(results, 'quaternion'), 'Error: No se encontraron cuaterniones en los resultados.');
assert(isfield(results, 'eulerAngles'), 'Error: No se encontraron ángulos de Euler en los resultados.');
assert(isfield(results, 'errorRotation'), 'Error: No se encontraron errores de rotación en los resultados.');

% Imprimir resultados clave
disp('Resultados de la clase Triad_Algorithm:');
disp('Primeros ángulos de Euler estimados (grados):');
disp(results.eulerAngles(:, 1));
disp('Primeros cuaterniones estimados:');
disp(results.quaternion(:, 1));

% Verificar error de rotación promedio
meanErrorRotation = mean(results.errorRotation);
fprintf('Error de rotación promedio [arcsec]: %.6f\n', meanErrorRotation);

% Validar que el error promedio sea pequeño
tolerance = 1e-6;
if meanErrorRotation < tolerance
    disp('La clase Triad_Algorithm funciona correctamente: El error está dentro del rango esperado.');
else
    disp('La clase Triad_Algorithm NO funciona correctamente: El error es mayor al rango esperado.');
end

%% Visualización
% Graficar el error de rotación
timeVector = 1:numSamples;
figure;
plot(timeVector, results.errorRotation, 'o', 'LineWidth', 2);
xlabel('Muestras');
ylabel('Error de rotación [arcsec]');
title('Error de rotación estimado vs. verdadero');
grid on;

% Graficar los ángulos de Euler estimados
figure;
plot(timeVector, results.eulerAngles(1, :), 'r', 'LineWidth', 2, 'DisplayName', 'Yaw Estimado');
hold on;
plot(timeVector, results.eulerAngles(2, :), 'g', 'LineWidth', 2, 'DisplayName', 'Pitch Estimado');
plot(timeVector, results.eulerAngles(3, :), 'b', 'LineWidth', 2, 'DisplayName', 'Roll Estimado');
legend;
xlabel('Muestras');
ylabel('Ángulos de Euler [deg]');
title('Ángulos de Euler estimados');
grid on;

%% Finalización del script
disp('Validación completada.');
