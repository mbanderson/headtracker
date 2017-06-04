classdef HeadDynamicsModel
    %HEADDYNAMICSMODEL Simulates head rotation according to true dynamics.
    %   This class uses pre-defined angular velocity measurements to
    %   integrate the head orientation quaternion. At each timestep, the
    %   model stores the true angular velocity and orientation quaternion
    %   in the head frame.
    %
    
    properties
        dt; % s
        ts; % s
        ws; % deg/s
        qs; % quaternions
        
        % Sensors
        accel;
        gyro;
        mag;
        % Simulated measurements, body frame
        accel_meas; % m/s^2
        gyro_meas; % deg/s
        mag_meas; % Gauss
        
        env_model; % Reference gravity, mag field vectors
    end
    
    methods
        function obj = HeadDynamicsModel()
            obj.env_model = Environment();
            
            % Generate pre-defined angular velocity values
            obj.dt = 0.1;
            t0 = obj.dt;
            tf = 100;
            obj.ts = t0:obj.dt:tf;
            axis = 2; % single axis w only
            
            w0 = [0,0,0];
            obj.ws = vertcat(w0, AxisAngVals(axis,obj.ts) * 180/pi);
            
            % Integrate quaternions over time history
            q0 = [1,0,0,0]';
            obj.qs = zeros(numel(obj.ts)+2,4); obj.qs(1,:) = q0;
            for i = 1:numel(obj.ts)+1
                obj.qs(i+1,:) = QuatIntegration(obj.qs(i,:)',obj.ws(i,:)',obj.dt);
            end
            
            % Generate head-frame ("sensor inside the head") measurements
            % Noise parameters based on imuAnalysis.m
            accel_mu = zeros(3,1); 
            accel_sigma = 1.0e-04*[0.1042, 0, 0.0004;
                                0, 0.1010, -0.0005;
                                0.0004, -0.0005, 0.2463];
            obj.accel = Accelerometer(obj, accel_mu, accel_sigma);
            
            % Noise parameters based on imuAnalysis.m
            gyro_mu = zeros(3,1); 
            gyro_sigma = [0.0073, 0, 0;
                          0, 0.0091, 0;
                          0, 0, 0.0075];
            bias_scale = 1/50; % scale static bias down to simulate imperfect calibration
            gyro_bias = bias_scale*[-0.0388, 0.1440, -0.1383];
            obj.gyro = Gyroscope(obj, gyro_mu, gyro_sigma, gyro_bias);
            
            % Noise parameters based on imuAnalysis.m
            mag_mu = zeros(3,1); 
            mag_sigma = 1.0e-06*[0.4017, -0.0353, 0.0255;
                                -0.0353, 0.3963, -0.0061;
                                 0.0255, -0.0061, 0.3319];            
            obj.mag = Magnetometer(obj, mag_mu, mag_sigma);
            
            noisy = false; % FLAG: Enable for noise
            obj.accel_meas = obj.accel.measurements(noisy);
            obj.gyro_meas = obj.gyro.measurements(noisy);
            obj.mag_meas = obj.mag.measurements(noisy);
        end 
    end
    
end

