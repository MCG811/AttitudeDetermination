%% TRIAD Algorithm Validation Script
% Este script valida el funcionamiento del algoritmo TRIAD

% Limpieza del entorno
clear; clc;
disp('Initializing TRIAD Algorithm Validation Script...');

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

%% Ejecutar el algoritmo TRIAD
body_meas = [body_vector1'; body_vector2'];
inertial_values = [inertial_vector1'; inertial_vector2'];
anchor_vector = 1; % Usar el primer vector como base

% Llamar a la función TRIADMethod
A_estimated = TRIAD(body_meas, inertial_values, anchor_vector);

%% Verificar resultados
% Comparar la matriz estimada con la matriz verdadera
fprintf('Matriz verdadera (A_true):\n');
disp(A_true);

fprintf('Matriz estimada (A_estimated):\n');
disp(A_estimated);

% Error absoluto entre las matrices
error_matrix = abs(A_true - A_estimated);
error_norm = norm(error_matrix, 'fro'); % Norma de Frobenius del error

fprintf('Norma del error entre A_true y A_estimated: %.6f\n', error_norm);

% Validar que el error es suficientemente pequeño
tolerance = 1e-6;
if error_norm < tolerance
    disp('El algoritmo TRIAD funciona correctamente: El error está dentro del rango esperado.');
else
    disp('El algoritmo TRIAD NO funciona correctamente: El error es mayor al rango esperado.');
end

%% Visualización
% Graficar los vectores en el marco inercial y del cuerpo
figure;
quiver3(0, 0, 0, inertial_vector1(1), inertial_vector1(2), inertial_vector1(3), 'r', 'LineWidth', 2, 'DisplayName', 'Inertial Vector 1');
hold on;
quiver3(0, 0, 0, inertial_vector2(1), inertial_vector2(2), inertial_vector2(3), 'g', 'LineWidth', 2, 'DisplayName', 'Inertial Vector 2');
quiver3(0, 0, 0, body_vector1(1), body_vector1(2), body_vector1(3), 'r--', 'LineWidth', 2, 'DisplayName', 'Body Vector 1');
quiver3(0, 0, 0, body_vector2(1), body_vector2(2), body_vector2(3), 'g--', 'LineWidth', 2, 'DisplayName', 'Body Vector 2');
grid on;
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Vectores en Marcos Inercial y del Cuerpo');
legend;

%% Finalización del script
disp('Validación completada.');