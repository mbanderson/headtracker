classdef HeadDynamicsModel
    %HEADDYNAMICSMODEL Simulates head rotation according to true dynamics.
    %   This class uses pre-defined angular velocity measurements to
    %   integrate the head orientation quaternion. At each timestep, the
    %   model stores the true angular velocity and orientation quaternion
    %   in the head frame.
    %
    
    properties
        ts;
        ws;
        qs;
        
        accel_meas;
        gyro_meas;
        mag_meas;
        
        env_model;
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
            
            obj.ws = AxisAngVals(axis,obj.ts);
            
            % Integrate quaternions over time history
            q0 = [1,0,0,0]';
            obj.qs = zeros(numel(obj.ts),4); obj.qs(1,:) = q0;
            for i = 2:numel(obj.ts)
                obj.qs(i,:) = QuatIntegration(obj.qs(i-1,:)',obj.ws(i-1,:)',dt);
            end
            
            % Generate head-frame ("sensor inside the head") measurements
            
        end 
    end
    
end

