%% Acceleration / Angular Velocity Source Analysis
% Performs filter runs with different angular rate and acceleration model
% sources.
%
close all; clear all; clc;
%% Define parameters
%       gyro_type:
%       1: gyroscope measurements
%       2: constellation angular rate measurements
%
%       accel_type:
%       1: center-of-head accelerometer measurements
%       2: naive/uncompensated head-mounted measurements
%       3: compensated head-mounted measurements
%       4: constellation accelerometer measurements

tf = 30;
noisy = true;

% Traditional gyroscope measurements
% Center, Naive, Compensated
teststrs = {'Center of Head','Naive','Compensated','Constellation'};
for i = 1:4
    if i == 4
        gyro = 2; accel = 4;
    else
        gyro = 1; accel = i;
    end
    
    [rms,errs] = ekf(gyro,accel,tf,noisy);
    resultstr = sprintf('(phi,th,psi)=(%.3f, %.3f, %.3f) [deg/s]',rms(2),rms(3),rms(4));
    fprintf(strcat(teststrs{i}, ': ', resultstr, '\n'));
end
