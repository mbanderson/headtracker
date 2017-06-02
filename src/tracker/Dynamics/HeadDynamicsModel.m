classdef HeadDynamicsModel
    %HEADDYNAMICSMODEL Simulates head rotation according to true dynamics.
    %   This class uses pre-defined angular velocity measurements to
    %   integrate the head orientation quaternion. At each timestep, the
    %   model stores the true angular velocity and orientation quaternion
    %   in the head frame.
    %
    
    properties
        ts; % s
        ws; % deg/s
        qs; % quaternions
        
        % Sensors
        accel;
        gyro;
        mag;
        % Simulated measurements
        accel_meas; % m/s^2
        gyro_meas; % deg/s
        mag_meas; % Gauss
        
        env_model; % Reference gravity, mag field vectors
    end
    
    methods
        function obj = HeadDynamicsModel()
            obj.env_model = Environment();
            
            % Generate pre-defined angular velocity values
            t0 = 0;
            dt = 0.1;
            tf = 100;
            obj.ts = t0:dt:tf;
            axis = 1; % w_x only
            
            obj.ws = AxisAngVals(axis,obj.ts) * 180/pi;
            
            % Integrate quaternions over time history
            q0 = [1,0,0,0]';
            obj.qs = zeros(numel(obj.ts),4); obj.qs(1,:) = q0;
            for i = 2:numel(obj.ts)
                obj.qs(i,:) = QuatIntegration(obj.qs(i-1,:)',obj.ws(i-1,:)',dt);
            end
            
            % Generate head-frame ("sensor inside the head") measurements
            accel_mu = zeros(3,1); accel_sigma = 0.1^3*eye(3);
            obj.accel = Accelerometer(obj, accel_mu, accel_sigma);
            
            % FLAG: Set Gyro static bias
            gyro_mu = zeros(3,1); gyro_sigma = 0.1^3*eye(3);
            obj.gyro = Gyroscope(obj, gyro_mu, gyro_sigma);
            
            mag_mu = zeros(3,1); mag_sigma = 0.1^3*eye(3);
            obj.mag = Magnetometer(obj, mag_mu, mag_sigma);
            
            noisy = false; % FLAG: Enable for noise
            obj.accel_meas = obj.accel.measurements(noisy);
            obj.gyro_meas = obj.gyro.measurements(noisy);
            obj.mag_meas = obj.mag.measurements(noisy);
        end 
    end
    
end

