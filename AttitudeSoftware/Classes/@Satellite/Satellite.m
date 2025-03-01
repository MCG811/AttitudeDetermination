classdef Satellite

    % SATELLITE: Simulates satellite attitude dynamics and sensor measurements.
    %
    % This class models a satellite's attitude determination system, converting 
    % inertial reference vectors into body-frame measurements using quaternion-based
    % transformations.
    %
    % PROPERTIES:
    % - SunSensor: Inertial frame sun vector
    % - MagnetometerSensor: Inertial frame magnetic field vector
    % - SunSensor_var: Variance of sun sensor measurements
    % - MagnetometerSensor_var: Variance of magnetometer measurements
    %
    % METHODS:
    % - Satellite: Constructor to initialize the satellite model
    % - getQuaternion: Converts Euler angles (yaw, pitch, roll) to quaternions
    % - getDCM: Converts Euler angles (yaw, pitch, roll) to a direction cosine matrix (DCM)
    % - Intertial2BodyFrame: Transforms inertial reference vectors to the body frame
    % - Intertial2BodyFrame_failure: Simulates sensor failure in transformation

    properties
        SunSensor             % Sun vector in the inertial frame
        MagnetometerSensor    % Magnetic field vector in the inertial frame
        SunSensor_var         % Sun sensor noise variance
        MagnetometerSensor_var % Magnetometer sensor noise variance
    end
    methods
        function obj = Satellite(sunSensor, magnetometer, sun_var, mag_var)

            % CONSTRUCTOR: Initializes the Satellite model.
            %
            % INPUTS:
            % - sunSensor: Sun vector in the inertial frame (3x1)
            % - magnetometer: Magnetic field vector in the inertial frame (3x1)
            % - sun_var: Variance of sun sensor noise
            % - mag_var: Variance of magnetometer noise
            %
            % OUTPUT:
            % - obj: Satellite object with initialized properties

            obj.SunSensor = sunSensor; %/norm(sunSensor);
            obj.MagnetometerSensor = magnetometer; %/norm(magnetometer);
            obj.SunSensor_var = sun_var;
            obj.MagnetometerSensor_var = mag_var;
        end

        function quaternion = getQuaternion(~, yaw, pitch, roll)
            % GETQUATERNION: Converts Euler angles to quaternions.
            %
            % INPUTS:
            % - yaw, pitch, roll: Euler angles in radians
            %
            % OUTPUT:
            % - quaternion: Corresponding quaternion representation (4x1)

            quaternion = angle2quat(yaw, pitch, roll, 'ZYX')';
        end

        function DCM = getDCM(~, yaw, pitch, roll)
            % GETDCM: Converts Euler angles to a Direction Cosine Matrix (DCM).
            %
            % INPUTS:
            % - yaw, pitch, roll: Euler angles in radians
            %
            % OUTPUT:
            % - DCM: Direction Cosine Matrix (3x3)

            DCM = angle2dcm(yaw, pitch, roll);
        end

        function [bodyData] = Intertial2BodyFrame(obj, attitudeQuat)
            % INERTIAL2BODYFRAME: Transforms inertial reference vectors to the body frame.
            %
            % INPUT:
            % - attitudeQuat: Quaternions representing the attitude at different time steps (4xT)
            %
            % OUTPUT:
            % - bodyData: Sensor measurements in the body frame (2x3xT)

            numSamples = size(attitudeQuat, 2);
            bodyData = zeros(2, 3, numSamples);

            % Define noise limits
            lim_inf = -2;
            lim_sup = 2;
           
            for t = 1:numSamples
                % Convert quaternion to DCM (rotation matrix)
                A = quat2dcm(attitudeQuat(:, t)');

                % Generate random noise vector within defined limits
                rand_vec = max(min(randn(3, 1), lim_sup), lim_inf);

                % Compute noisy sun sensor measurement
                noise_sun = obj.SunSensor_var*rand_vec;
                SunBody = A * obj.SunSensor + noise_sun;
                bodyData(1, :, t) = SunBody/norm(SunBody)';

                % Compute noisy magnetometer measurement
                noise_mag = obj.MagnetometerSensor_var*rand_vec;
                mag_vec = obj.MagnetometerSensor + noise_mag;
                MagBody = A * mag_vec; 
                bodyData(2, :, t) = MagBody/norm(MagBody)';
            end
        end

        function [bodyData] = Intertial2BodyFrame_failure(obj, attitudeQuat)
            % INERTIAL2BODYFRAME_FAILURE: Simulates sensor failure by modifying sun sensor measurements.
            %
            % INPUT:
            % - attitudeQuat: Quaternions representing the attitude at different time steps (4xT)
            %
            % OUTPUT:
            % - bodyData: Sensor measurements in the body frame (2x3xT)
            %
            %   NOTE: This function simulates a failure where the sun sensor outputs zero
            %   measurements during the middle third of the simulation time.

            numSamples = size(attitudeQuat, 2);
            bodyData = zeros(2, 3, numSamples);

            % Define noise limits
            lim_inf = -2;
            lim_sup = 2;
           
            for t = 1:numSamples
                % Convert quaternion to DCM (rotation matrix)
                A = quat2dcm(attitudeQuat(:, t)');

                % Generate random noise vector within defined limits
                rand_vec = max(min(randn(3, 1), lim_sup), lim_inf);

                % Simulate sun sensor failure during middle third of simulation
                if t >= numSamples/3 && t<= 2*numSamples/3
                    bodyData(1, :, t) = [0 0 0];
                else
                    noise_sun = obj.SunSensor_var*rand_vec;
                    SunBody = A * obj.SunSensor + noise_sun;
                    bodyData(1, :, t) = SunBody/norm(SunBody)';
                end

                % Compute noisy magnetometer measurement
                noise_mag = obj.MagnetometerSensor_var*rand_vec;
                mag_vec = obj.MagnetometerSensor + noise_mag;
                MagBody = A * mag_vec; 
                bodyData(2, :, t) = MagBody/norm(MagBody)';
            end
        end
    end
end


