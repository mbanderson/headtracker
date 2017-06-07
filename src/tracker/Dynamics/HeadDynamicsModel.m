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
        ws; % rad/s
        wdots; % rad/s
        qs; % quaternions
        
        % Sensors
        accel;
        gyro;
        mag;
        % Simulated measurements, body frame
        accel_meas; % m/s^2
        gyro_meas; % rad/s
        mag_meas; % Gauss
        
        accel_meas_offset; % m/s^2
        % accel_offset = [0.04, 0.04, -0.04]; % m
        
        ws_cons % rad/s
        wdots_cons % rad/s^2
        accel_meas_cons %m/s^2
        
        avg_time_filter % s
        avg_time_cons % s
        
        env_model; % Reference gravity, mag field vectors
    end
    
    methods
        function obj = HeadDynamicsModel(tf,noisy)
            obj.env_model = Environment();
            
            % Generate pre-defined angular velocity values
            obj.dt = 0.01;
            t0 = obj.dt;
            obj.ts = t0:obj.dt:tf;
            
            % Get w, wdot values
            axes = [2]; % 1: x-axis, 2: y-axis, 3: z-axis (or combinations)
            [wvals,wdotvals,handles] = AxisAngVals(axes,obj.ts);
            
            % Add in initial conditions
            w0 = [0,0,0];
            wdot0 = zeros(1,3);
            for i = 1:numel(handles)
                wdot_ftn = handles{i};
                wdot0(i) = wdot_ftn(0);
            end
                        
            obj.ws = vertcat(w0, wvals);
            obj.wdots = vertcat(wdot0, wdotvals);
            
            R_mu = zeros(4,1);
            R_sigma = 0.1*obj.dt^2*eye(4);
            
            % Integrate quaternions over time history
            q0 = [1,0,0,0]';
            obj.qs = zeros(numel(obj.ts)+2,4); obj.qs(1,:) = q0;
            for i = 1:numel(obj.ts)+1
                obj.qs(i+1,:) = QuatIntegration(obj.qs(i,:)',obj.ws(i,:)',obj.dt) + mvnrnd(R_mu,R_sigma)';
                obj.qs(i+1,:) = obj.qs(i+1,:) / norm(obj.qs(i+1,:));
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
            gyro_sigma = pi/180*[0.0073, 0, 0;
                                 0, 0.0091, 0;
                                 0, 0, 0.0075];
            gyro_bias = pi/180*[-0.0388, 0.1440, -0.1383];
            obj.gyro = Gyroscope(obj, gyro_mu, gyro_sigma, gyro_bias);
            
            % Noise parameters based on imuAnalysis.m
            mag_mu = zeros(3,1); 
            mag_sigma = 1.0e-06*[0.4017, -0.0353, 0.0255;
                                -0.0353, 0.3963, -0.0061;
                                 0.0255, -0.0061, 0.3319];            
            obj.mag = Magnetometer(obj, mag_mu, mag_sigma);
            
            obj.accel_meas = obj.accel.measurements(noisy);
            obj.gyro_meas = obj.gyro.measurements(noisy);
            obj.mag_meas = obj.mag.measurements(noisy);
            
            [obj.ws_cons, obj.wdots_cons, obj.accel_meas_cons, ...
                obj.accel_meas_offset, obj.avg_time_cons] = ...
                    constellationMeasurement(obj, noisy);
        end 
    end
    
end

