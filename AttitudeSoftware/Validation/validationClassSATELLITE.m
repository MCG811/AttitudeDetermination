%% Satellite Class Validation Script
% Este script valida la funcionalidad y definición correcta de la clase Satellite

% Limpieza del entorno
clear; clc;
disp('Initializing Satellite Class Validation Script...');

%% Configuración inicial
% Definir los ángulos de Euler (en radianes)
eulerAngles.yaw = deg2rad(30); % Rotación en yaw
eulerAngles.pitch = deg2rad(20); % Rotación en pitch
eulerAngles.roll = deg2rad(10); % Rotación en roll

% Definir vectores inerciales
sunSensor = [1; 0; 0]; % Vector solar
magnetometerSensor = [0; 1; 0]; % Vector magnético

%% Crear el objeto Satellite
satellite = Satellite(eulerAngles, sunSensor, magnetometerSensor);

%% Validar métodos de la clase
% Obtener cuaterniones
quaternion = satellite.getQuaternion();
assert(length(quaternion) == 4, 'Error: El cuaternión no tiene 4 componentes.');

% Obtener matriz de cosenos de dirección (DCM)
DCM = satellite.getDCM();
assert(all(size(DCM) == [3, 3]), 'Error: La matriz DCM no es de 3x3.');

% Validar transformación de vectores
numSamples = 10;
attitudeQuat = repmat(quaternion, 1, numSamples); % Repetir cuaterniones para simulación
[sunBody, magBody] = satellite.Intertial2BodyFrame(attitudeQuat);
assert(size(sunBody, 2) == numSamples, 'Error: El número de muestras en sunBody no es correcto.');
assert(size(magBody, 2) == numSamples, 'Error: El número de muestras en magBody no es correcto.');

% Obtener matriz de rotación desde los ángulos de Euler
DCM_from_euler = angle2dcm(eulerAngles.yaw, eulerAngles.pitch, eulerAngles.roll, 'ZYX');

% Transformar vectores inerciales usando la matriz de rotación
sun_transformed_expected = DCM_from_euler * sunSensor;
mag_transformed_expected = DCM_from_euler * magnetometerSensor;

%% Imprimir resultados
% Mostrar los valores clave generados
disp('Resultados de la clase Satellite:');
disp('Cuaternión estimado:');
disp(quaternion);
disp('Matriz DCM:');
disp(DCM);
disp('Primer vector transformado (sunBody):');
disp(sunBody(:, 1));

% Comparar errores
sun_error = norm(sunBody(:, 1) - sun_transformed_expected);
mag_error = norm(magBody(:, 1) - mag_transformed_expected);

fprintf('Error en el vector solar transformado: %.6f\n', sun_error);
fprintf('Error en el vector magnético transformado: %.6f\n', mag_error);

% Verificar si los errores son pequeños
tolerance = 1e-6;
if sun_error < tolerance && mag_error < tolerance
    disp('Los vectores transformados coinciden con las expectativas según los ángulos definidos.');
else
    disp('Error significativo en los vectores transformados.');
end

%% Visualización
% Graficar vectores originales y transformados
figure;
quiver3(0, 0, 0, sunSensor(1), sunSensor(2), sunSensor(3), 'r', 'LineWidth', 2, 'DisplayName', 'Sun Sensor (Inertial)');
hold on;
quiver3(0, 0, 0, sunBody(1, 1), sunBody(2, 1), sunBody(3, 1), 'r--', 'LineWidth', 2, 'DisplayName', 'Sun Sensor (Body)');
quiver3(0, 0, 0, magnetometerSensor(1), magnetometerSensor(2), magnetometerSensor(3), 'g', 'LineWidth', 2, 'DisplayName', 'Magnetometer (Inertial)');
quiver3(0, 0, 0, magBody(1, 1), magBody(2, 1), magBody(3, 1), 'g--', 'LineWidth', 2, 'DisplayName', 'Magnetometer (Body)');
grid on;
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Vectores inerciales y transformados');
legend;

% Graficar los vectores originales y transformados
figure;
quiver3(0, 0, 0, sunSensor(1), sunSensor(2), sunSensor(3), 'r', 'LineWidth', 2, 'DisplayName', 'Sun Sensor (Inertial)');
hold on;
quiver3(0, 0, 0, sunBody(1, 1), sunBody(2, 1), sunBody(3, 1), 'r--', 'LineWidth', 2, 'DisplayName', 'Sun Sensor (Body - Actual)');
quiver3(0, 0, 0, sun_transformed_expected(1), sun_transformed_expected(2), sun_transformed_expected(3), 'k:', 'LineWidth', 2, 'DisplayName', 'Sun Sensor (Body - Expected)');
quiver3(0, 0, 0, magnetometerSensor(1), magnetometerSensor(2), magnetometerSensor(3), 'g', 'LineWidth', 2, 'DisplayName', 'Magnetometer (Inertial)');
quiver3(0, 0, 0, magBody(1, 1), magBody(2, 1), magBody(3, 1), 'g--', 'LineWidth', 2, 'DisplayName', 'Magnetometer (Body - Actual)');
quiver3(0, 0, 0, mag_transformed_expected(1), mag_transformed_expected(2), mag_transformed_expected(3), 'k:', 'LineWidth', 2, 'DisplayName', 'Magnetometer (Body - Expected)');
grid on;
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Vectores inerciales y transformados');
legend;

%% Finalización del script
disp('Validación completada.');
