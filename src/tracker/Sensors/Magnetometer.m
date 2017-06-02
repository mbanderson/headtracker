classdef Magnetometer < Sensor
    %MAGNETOMETER Models center of head magnetometer measurements.
    %   Initializes magnetometer object to sample measurements from the
    %   center of head simulator.
    %
    
    properties
    end
    
    methods
        function obj = Magnetometer(model, mu, sigma)
            %   Initialize Magnetometer sensor model.
            %   INPUTS:
            %       model - (HeadDynamicsModel) models center of neck motion
            %       mu - (3x1 vector) Gaussian noise mean
            %       sigma - (3x3 matrix) Gaussian noise covariance
            %
            
            narginchk(3,3);
            type = 'mag';
            obj@Sensor(type, model, mu, sigma);
        end
    end
    
end
