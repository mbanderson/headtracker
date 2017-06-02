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
        function y = vec_rot(x,q)
            % Rotate (3x1 vector) by quaternion.
            % INPUTS:
            %   x - (3x1) input vector
            %   q - (4x1) quaternion by which to rotate vector
            % OUTPUTS:
            %   y - (3x1) rotated vector
            %
            R = [1-2*q(3)^2-2*q(4)^2, 2*(q(2)*q(3)+q(1)*q(4)), 2*(q(2)*q(4)-q(1)*q(3));
                 2*(q(2)*q(3)-q(1)*q(4)), 1-2*q(2)^2-2*q(4)^2, 2*(q(3)*q(4)+q(1)*q(2));
                 2*(q(2)*q(4)+q(1)*q(3)), 2*(q(3)*q(4)-q(1)*q(2)), 1-2*q(2)^2-2*q(3)^2];
            y = R*x; 
        end
    end
    
end
