classdef Gyroscope < Sensor
    %GYROSCOPE Models center of head gyroscope measurements.
    %   Initializes gyroscope object to sample measurements from the
    %   center of head simulator.
    %
    %   INPUTS:
    %       model - (HeadDynamicsModel) models center of neck motion
    %       mu - (3x1 vector) Gaussian noise mean
    %       sigma - (3x3 matrix) Gaussian noise covariance
    %       bias - (3x1 vector) static gyroscope bias
    %
    
    properties
        static_bias;
    end
    
    methods
        function obj = Gyroscope(model, mu, sigma, bias)
            %   Initialize Gyroscope sensor model.
            %   INPUTS:
            %       model - (HeadDynamicsModel) models center of neck motion
            %       mu - (3x1 vector) Gaussian noise mean
            %       sigma - (3x3 matrix) Gaussian noise covariance
            %       bias - (3x1 vector) static gyroscope bias
            %
   
            narginchk(3,4);
            type = 'gyro';
            obj@Sensor(type, model, mu, sigma);
            
            if nargin < 4
                bias = 0;
            end 
            obj.static_bias = bias;
        end
        function ys = measurements(obj,noisy)
           %    Generate gyroscope measurements, with optional static bias
           %    INPUTS:
           %        noisy - (bool) True to add static bias to measurements
           %
            
           ys = measurements@Sensor(obj,noisy);
           if noisy
               for i = 1:size(ys,1)
                   ys(i,:) = ys(i,:) + obj.static_bias;
               end
           end
        end
    end
    
end
