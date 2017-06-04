classdef Sensor
    %SENSOR Models head-mounted sensors measurements.
    %   The head-tracking model dynamics are defined with respect to the
    %   center of rotation (i.e. center of head). This class defines sensor
    %   measurements as if they were sampled at the center of head ("inside
    %   the head measurements"). These values can feed into subsequent
    %   calculations for head-mounted sensor values.
    %
    
    properties
        % Sensor type
        type;
        
        % Gaussian noise parameters
        mu;
        sigma;
        
        % Center of head dynamics simulator
        % Used to query current head orientation
        head_model; % true dynamics (for real measurements)
        
    end
    properties (Constant)
        sensor_types = {'accel', 'gyro', 'mag'};
    end
    
    methods
        function obj = Sensor(type, model, mu, sigma)
            %   Initialize sensor model.
            %   INPUTS:
            %       type - (str) sensor type: 'accel', 'gyro', or 'mag'
            %       model - (HeadDynamicsModel) center of head motion
            %       mu - (3x1 vector) Gaussian noise mean
            %       sigma - (3x3 matrix) Gaussian noise covariance
            %
            
            narginchk(4,4);
            if any(strcmp(type,obj.sensor_types))
                obj.type = type;
            else
                warning('Unrecognized type. Default sensor type assigned.');
                obj.type = obj.sensor_types{1};
            end
            
            obj.head_model = model;
            obj.mu = mu;
            obj.sigma = sigma;
        end
        function ys = measurements(obj,noisy)
            % Sample measurements in center of head frame.
            % INPUTS:
            %   noisy - (bool) add noise to measurement
            % OUTPUTS:
            %   ys - (Nx3) measurement array for given sensor type
            %
            narginchk(1,2);
            if nargin < 2
                noisy = true;
            end
            
            % Acceleration model
            head_quats = obj.head_model.qs;
            n_meas = size(head_quats,1)-1;
            ys = zeros(n_meas,3);
            if strcmp(obj.type,obj.sensor_types{1})
                % No linear acceleration at center of head, only consider
                % gravity vector
                grav_vec = obj.head_model.env_model.grav_vec;
                for i = 1:n_meas
                    ys(i,:) = Quaternion.Rwb(head_quats(i+1,:)')*grav_vec;
                    if noisy
                        ys(i,:) = ys(i,:) + obj.sensor_noise();
                    end
                end
                
            % Gyroscope model
            elseif strcmp(obj.type,obj.sensor_types{2})
                for i = 1:n_meas
                    ys(i,:) = obj.head_model.ws(i,:);
                    if noisy
                        ys(i,:) = ys(i,:) + obj.sensor_noise();
                    end
                end
                                
            % Magnetometer model
            elseif strcmp(obj.type,obj.sensor_types{3})
                field_vec = obj.head_model.env_model.field_vec;
                for i = 1:n_meas
                    ys(i,:) = Quaternion.Rwb(head_quats(i+1,:)')*field_vec;
                    if noisy
                        ys(i,:) = ys(i,:) + obj.sensor_noise();
                    end
                end    
            else
                error('Measurements requested for invalid sensor type.');
            end
        end
    end
    
    methods (Access = private)
        function vt = sensor_noise(obj)
            % Sample Gaussian noise values for measurement model
            vt = mvnrnd(obj.mu,obj.sigma);
        end
    end
    
end
