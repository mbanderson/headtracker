classdef Quaternion
    %QUATERNION Helper class for quaternion representations.
    %   Provides helper functions for vector rotations, etc.
    %
    
    properties
        q;
    end
    
    methods
        function obj = Quaternion(q)
            narginchk(0,1);
            if nargin < 1
               q = [1,0,0,0]';
            end
            obj.q = q;
        end
    end
    
    methods (Static)
        function y = vec2world(x,q)
            % Rotate (3x1 vector) by quaternion from body to world.
            % INPUTS:
            %   x - (3x1) input vector
            %   q - (4x1) quaternion by which to rotate vector
            % OUTPUTS:
            %   y - (3x1) rotated vector
            %
            R = Quaternion.Rbw(q);
            y = R*x; 
        end
        function y = vec2body(x,q)
            % Rotate (3x1 vector) by quaternion from world to body.
            % INPUTS:
            %   x - (3x1) input vector
            %   q - (4x1) quaternion by which to rotate vector
            % OUTPUTS:
            %   y - (3x1) rotated vector
            %
            R = Quaternion.Rwb(q);
            y = R*x;
        end
            
        function R = Rbw(q)
            % Produce rotation matrix from body to world
            %   INPUTS:
            %       q - (4x1) quaternion representing rotation
            %   OUTPUTS:
            %       R - (3x3) body-to-world rotation matrix
            %
            R = Rwb(q)';
        end
        
        function R = Rwb(q)
            % Produce rotation matrix from world to body
            R = [q(1)^2+q(2)^2-q(3)^2-q(4)^2, 2*q(2)*q(3)+2*q(1)*q(4), 2*q(2)*q(4)-2*q(1)*q(3);
                 2*q(2)*q(3)-2*q(1)*q(4), q(1)^2-q(2)^2+q(3)^2-q(4)^2, 2*q(3)*q(4)+2*q(1)*q(2);
                 2*q(2)*q(4)+2*q(1)*q(3), 2*q(3)*q(4)-2*q(1)*q(2), q(1)^2-q(2)^2-q(3)^2+q(4)^2];
        end
    end
    
end
