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
noisy = false;

% Traditional gyroscope measurements
% Center, Naive, Compensated
teststrs = {'Center of Head','Naive','Compensated','Constellation'};

h1 = figure(); h2 = figure(); h3 = figure();
figs = {h1,h2,h3};
colorstrs = {'b:','r-','g--','k-.'};
widths = {.2,.2,.2,2.5};
for i = 1:4
    if i == 4
        gyro = 2; accel = 4;
    else
        gyro = 1; accel = i;
    end
    
    [rms,errs,model] = ekf(gyro,accel,tf,noisy);
    ts = horzcat(0,model.ts);
    
    resultstr = sprintf('(phi,th,psi)=(%.3f, %.3f, %.3f) [deg/s]',rms(2),rms(3),rms(4));
    fprintf(strcat(teststrs{i}, ': ', resultstr, '\n'));
    
    figure(figs{1}); hold on;
    plot(ts(1:5:end),errs(1:5:end-1,2),colorstrs{i},'LineWidth',widths{i});
    
    figure(figs{2}); hold on;
    plot(ts(1:5:end),errs(1:5:end-1,3),colorstrs{i},'LineWidth',widths{i});
    
    figure(figs{3}); hold on;
    plot(ts(1:5:end),errs(1:5:end-1,4),colorstrs{i},'LineWidth',widths{i});
end
for i = 1:numel(figs)
    if i == 1
        titlestr = '\phi';
    elseif i == 2
        titlestr = '\theta';
    else
        titlestr = '\psi';
    end
    title(titlestr);
    
    figure(figs{i});
    legend('Center of Head','Naive','Compensated','Constellation');
    xlabel('Time (s)');
    ylabel('Error (deg)');
end
