%% Final Paper Plots
% Performs filter runs and plots results.
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

options = ekfOptions();
options.tf = 15;
options.noisy = true;
options.plotresults = true;

%% Single Sim
options.gyro_type = 1; options.accel_type = 3;
[rms,errs,model] = ekf(options);

%% Error Analysis
% Traditional gyroscope measurements
% Center, Naive, Compensated
teststrs = {'Center of Head','Naive','Compensated','Constellation'};

h1 = figure(); h2 = figure(); h3 = figure();
figs = {h1,h2,h3};
colorstrs = {'b:','r-','g--','k-.'};
widths = {.2,.2,.2,2.5};

all_rms = cell(numel(colorstrs),1);
all_errs = cell(numel(colorstrs),1);
all_models = cell(numel(colorstrs),1);
all_resultstrs = cell(numel(colorstrs),1);
for i = 1:4
    if i == 4
        options.gyro_type = 2; options.accel_type = 4;
    else
        options.gyro_type = 1; options.accel_type = i;
    end
    
    [rms,errs,model] = ekf(options);
    fprintf('Gyro Type: %d, Accel Type: %d\n',options.gyro_type,options.accel_type);
    fprintf('Average filter loop time (ms): %.5f\n',model.avg_time_total * 1000);
    
    all_rms{i} = rms;
    all_errs{i} = errs;
    all_models{i} = model;
    
    ts = horzcat(0,model.ts);
    
    resultstr = sprintf('(phi,th,psi)=(%.3f, %.3f, %.3f) [deg/s]',rms(2),rms(3),rms(4));
    all_resultstrs{i} = strcat(teststrs{i}, ': ', resultstr, '\n');
end

%% Plot Results
for i = 1:numel(figs)
    h = figure(figs{i});
    if i == 1
        titlestr = 'Euler Angle Error: \phi';
    elseif i == 2
        titlestr = 'Euler Angle Error: \theta';
    else
        titlestr = 'Euler Angle Error: \psi';
    end
    title(titlestr);

    % Plot each sim run for the given phi, theta, or psi
    hold on;
    for j = 1:numel(all_errs)
        plot(ts(1:5:end),all_errs{j}(1:5:end-1,i+1),colorstrs{j},'LineWidth',widths{j});
    end
    ylim([-15,15]); 
    legend('Center of Head','Naive','Compensated','Constellation');
    xlabel('Time (s)');
    ylabel('Error (deg)');
    
    print(h,sprintf('eulerangleerr%d',i),'-dpng');
end
