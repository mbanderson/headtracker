%% IMU Data Analysis
% Michael Anderson
% 5/23/2017
% Analyzes recorded IMU data collected by recorder.py
%
%% Data Read
fname = 'C:\Users\Michael\Desktop\recorder\saved\recorder_052317205217_longdwell.txt';
A = dlmread(fname);

T_COL = 1;
MX_COL = 2;
MY_COL = 3;
MZ_COL = 4;
AX_COL = 5;
AY_COL = 6;
AZ_COL = 7;
GX_COL = 8;
GY_COL = 9;
GZ_COL = 10;

%% Gyro Time-Frequency Plots
ts = A(:,T_COL) / 1000; % ms to seconds

figure(); plot(ts,A(:,GX_COL));
title('Gyroscope X-Axis (deg/s)');
ylabel('deg/s');
xlabel('Time (s)');
axis tight;

figure(); plot(ts,A(:,GY_COL));
title('Gyroscope Y-Axis (deg/s)');
ylabel('deg/s');
xlabel('Time (s)');
axis tight;

figure(); plot(ts,A(:,GZ_COL));
title('Gyroscope Z-Axis (deg/s)');
ylabel('deg/s');
xlabel('Time (s)');
axis tight;

%% Gyro Static Bias
bias = @(col) mean(A(:,col));
gx_bias = bias(GX_COL);
gy_bias = bias(GY_COL);
gz_bias = bias(GZ_COL);

% Remove static bias from readings
rem_bias = @(col,bias) A(:,col) - bias;
gx_nobias = rem_bias(GX_COL,gx_bias);
gy_nobias = rem_bias(GY_COL,gy_bias);
gz_nobias = rem_bias(GZ_COL,gz_bias);

%% Perform Allan Variance Analysis
% MPU-6050 Datasheet:
% https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
% Rate Noise Spectral Density: 0.005 deg/s/sqrt(Hz)
%
rnsd = 0.005;
expected_arw = 1/60*fft(rnsd); % deg/sqrt(h)

%%
gyro_data = {gx_nobias, gy_nobias, gz_nobias};
[allan_devs, taus] = deal(cell(numel(gyro_data),1));

for i = 1:numel(gyro_data)
    params = struct('freq', gyro_data{i}, 'time', ts);
    [allan_dev,~,~,tau] = allan_overlap(params);
    allan_devs{i} = allan_devs;
    taus{i} = tau;
    fprintf('Iteration %d complete\n',i);
end

%% Save Results
save('gyro_allan_dev.mat', 'allan_devs', 'taus');
%% Load Results

%% Extract Noise Parameters

adev_tau = 1;
[arws, biasinstabs] = deal(zeros(numel(gyro_data),1));

for i = 1:numel(gyro_data);
    allan_dev = allan_devs{i}; tau = taus{i};

    adev = allan_dev(tau == adev_tau);
    arws(i) = adev^2;
    biasinstabs(i) = min(allan_dev); % deg/s
end









