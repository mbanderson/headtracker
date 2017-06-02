%% IMU Data Analysis
% Analyzes recorded IMU data collected by recorder.py
%
% Results:
% Accerometer Static Bias:
%    -0.0504    0.0067    0.9726
% 
% Gyroscope Static Bias:
%    -0.0388    0.1440   -0.1383
% 
% Accelerometer covariance:
%    1.0e-04 *
% 
%     0.1042    0.0000    0.0004
%     0.0000    0.1010   -0.0005
%     0.0004   -0.0005    0.2463
% 
% Gyroscope covariance:
%     0.0073    0.0000   -0.0000
%     0.0000    0.0091   -0.0000
%    -0.0000   -0.0000    0.0075
% 
% Magnetometer covariance:
%    1.0e-06 *
% 
%     0.4017   -0.0353    0.0255
%    -0.0353    0.3963   -0.0061
%     0.0255   -0.0061    0.3319

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

%% Static Bias
bias = @(col) mean(A(:,col));
ax_bias = bias(AX_COL);
ay_bias = bias(AY_COL);
az_bias = bias(AZ_COL);
fprintf('Accerometer Static Bias:\n'); disp([ax_bias,ay_bias,az_bias]);

gx_bias = bias(GX_COL);
gy_bias = bias(GY_COL);
gz_bias = bias(GZ_COL);
fprintf('Gyroscope Static Bias:\n'); disp([gx_bias,gy_bias,gz_bias]);

%% Covariance Analysis
acov = cov(A(:,[AX_COL,AY_COL,AZ_COL]));
fprintf('Accelerometer covariance:\n'); disp(acov);
gcov = cov(A(:,[GX_COL, GY_COL, GZ_COL]));
fprintf('Gyroscope covariance:\n'); disp(gcov);
mcov = cov(A(:,[MX_COL, MY_COL, MZ_COL]));
fprintf('Magnetometer covariance:\n'); disp(mcov);

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

%% Remove Static Bias from Gyro
rem_bias = @(col,bias) A(:,col) - bias;
gx_nobias = rem_bias(GX_COL,gx_bias);
gy_nobias = rem_bias(GY_COL,gy_bias);
gz_nobias = rem_bias(GZ_COL,gz_bias);

%% Wait to Proceed
user = input('Continue to Allan Variance? [y/n]: ','s');
if ~strcmp(user,'y')
    return
end

%% Perform Allan Variance Analysis
% MPU-6050 Datasheet:
% https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
% Rate Noise Spectral Density: 0.005 deg/s/sqrt(Hz)
%
rnsd = 0.005;
expected_arw = 1/60*fft(rnsd); % deg/sqrt(h)
fprintf('Expected Gyro ARW (deg/sqrt(h)): %.8f\n',expected_arw);

%% Allan Variance
gyro_data = {gx_nobias, gy_nobias, gz_nobias};
[allan_devs, taus] = deal(cell(numel(gyro_data),1));

for i = 1:numel(gyro_data)
    params = struct('freq', gyro_data{i}, 'time', ts);
    [allan_dev,~,~,tau] = allan(params);
    allan_devs{i} = allan_devs;
    taus{i} = tau;
    fprintf('Axis %d complete\n',i);
end

%% Save Results
save('gyro_allan_dev.mat', 'allan_devs', 'taus');

%% Extract Noise Parameters
adev_tau = 1;
[arws, biasinstabs] = deal(zeros(numel(gyro_data),1));

for i = 1:numel(gyro_data);
    allan_dev = allan_devs{i}; tau = taus{i};

    adev = allan_dev(tau == adev_tau);
    arws(i) = adev^2;
    biasinstabs(i) = min(allan_dev); % deg/s
end
