classdef Environment < handle
    %ENVIRONMENT Represents environment external to head-tracking system.
    %   The head-tracking system relies on knowledge of the gravity vector
    %   and magnetic field. This class defines these parameters for the
    %   world frame in North-East-Down (NED) coordinates.
    %
    
    properties
        grav_vec;
        field_vec;
        
        % IGRF datestr
        date;
    end
    
    properties (Constant)
        % Magnetic Model Parameters
        % Stanford University (lat,long,alt)
        lat = 37.4275; % deg N
        long = 122.1697; % deg W
        alt = 0; % km above Earth surface
        frame = 'geodetic' 
        
        % Gravity Model Parameters
        g = 9.80665; % m/s^2
    end
    
    methods
        function obj = Environment(varargin)
            obj.date = floor(now);
            obj.updateFieldVector();
            obj.updateGravityVector();
        end
    end
        
    methods (Access = private)
        function updateGravityVector(obj)
            % Returns gravity vector in m/s^2, NED frame
            obj.grav_vec = [0, 0, -obj.g]';
        end
        function updateFieldVector(obj)            
            % Returns magnetic field in Gauss, NED frame
            %{
            % nT_vec = igrf(obj.date, obj.lat, obj.long, ...
                                 obj.alt, obj.frame)'; % nT
            % obj.field_vec = Environment.Tesla2Gauss(nT_vec / 10^9); % G
            %}
            
            % For now, we follow Sabatelli et al. and use a simple
            % reference vector to simplify the measurement Jacobian.
            obj.field_vec = [0,1,0]';
        end
    end
        
    methods (Static)
        function G = Tesla2Gauss(T)
            G = T * 10000;
        end
        function T = Gauss2Tesla(G)
            T = G / 10000;
        end
    end
end