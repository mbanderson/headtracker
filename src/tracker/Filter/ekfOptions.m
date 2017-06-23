classdef ekfOptions
    %EKFOPTIONS Stores basic EKF options for simulation runs.
    %   This class acts as a glorified struct; it initializes to the
    %   default simulation settings, which can then be modified on a
    %   case-by-case basis.
    %
    
    properties
        % Sensor configuration
        gyro_type;
        accel_type;
        
        % Basic sim settings
        tf;
        noisy;
        
        % Output flags
        plotresults;
        gifname;
        quatname;
    end
    
    methods
        function obj = ekfOptions()
           % gyro_type - (int) Flag to indicate angular rate measurement type
           %    1: gyroscope measurements
           %    2: constellation angular rate measurements
           %
           % accel_type - (int) Flag to indicate accelerometer measurement types
           %  to use in filtering:
           %    1: center-of-head accelerometer measurements
           %    2: naive/uncompensated head-mounted measurements
           %    3: compensated head-mounted measurements
           %    4: constellation accelerometer measurements
           %
           %    tf - (float) final simulation time
           %    noisy - (bool) turn noise on
           %    plotresults - (bool) plot output
           %    gifname - (str)(optional) gif file name (no extension)
           %    quatname - (str)(optional) base name for quaternion 
           %                                    files (no extension)
           %
            
           obj.gyro_type = 1;
           obj.accel_type = 1;
           
           obj.tf = 15;
           obj.noisy = true;
           
           obj.plotresults = false;
           obj.gifname = '';
           obj.quatname = '';
        end
        function obj = changeGyroType(obj, type)
           gyroTypes = [1,2];
           assert(any(type==gyroTypes),'Invalid gyroscope type.');
           obj.gyro_type = type;
        end
        function obj = changeAccelType(obj, type)
            accelTypes = [1,2,3,4];
            assert(any(type==accelTypes),'Invalid accelerometer type.');
            obj.accel_type = type;
        end
    end
    
end
