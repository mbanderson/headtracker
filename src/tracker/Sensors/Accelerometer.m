classdef Accelerometer < Sensor
    %ACCELEROMETER Models center of head accelerometer measurements.
    %   Initializes accelerometer object to sample measurements from the
    %   center of head simulator.
    %
    
    properties
    end
    
    methods
        function obj = Accelerometer(model, mu, sigma)
            %   Initialize Accelerometer sensor model.
            %   INPUTS:
            %       model - (DynamicsModel) models center of neck motion
            %       mu - (3x1 vector) Gaussian noise mean
            %       sigma - (3x3 matrix) Gaussian noise covariance
            %
            
            narginchk(3,3);
            type = 'accel';
            obj@Sensor(type, model, mu, sigma);
        end
    end
    
end
