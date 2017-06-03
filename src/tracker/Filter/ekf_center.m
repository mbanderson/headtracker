%% Single-Stage EKF with Center of Head Measurements
%
%% Simulate Head Movement and Measurements
Model = HeadDynamicsModel();

%% Helper Functions
% Expected measurement models
accel_expected = @(q) Quaternion.Rwb(q)*Model.env_model.grav_vec;
mag_expected = @(q) Quaternion.Rwb(q)*Model.env_model.field_vec;
% Measurement Jacobians
accel_jacob = @(q) norm(Model.env_model.grav_vec)*[2*q(3), 2*q(4), 2*q(1),2*q(2);
                                                  -2*q(2),-2*q(1), 2*q(4),2*q(3);
                                                   0     ,-4*q(2),-4*q(3),0];
mag_jacob = @(q) magnetometer_jacob(Model.env_model.field_vec,q);                                     

% accel_expected = @(q) Quaternion.Rbw(q)*Model.env_model.grav_vec;
% mag_expected = @(q) Quaternion.Rbw(q)*Model.env_model.field_vec;
% accel_jacob = @(q) 2*[-q(3), q(4), -q(1), q(2);
%                        q(2), q(1),  q(4), q(3);
%                        q(1), -q(2), -q(3), q(4)];
% mag_jacob = @(q) 2*[q(4), q(3), q(2), q(1);
%                     q(1), -q(2), -q(3), -q(4);
%                     -q(2), -q(1), q(4), q(3)];

%% Simulator Variables
%Q_mu = zeros(4,1);
%Q_sigma = 0.1*Model.dt^2*eye(4);
Q_mu = zeros(6,1);
Q_sigma = 0.1*Model.dt^2*eye(6);

R_mu = zeros(4,1);
R_sigma = 0.1*Model.dt^2*eye(4);

%% EKF
% Initial state estimate
mu = [1,0,0,0]';
sigma = 50000*eye(4);

mu_hist = zeros(numel(Model.ts),4); mu_hist(1,:) = mu;
sigma_hist = cell(numel(Model.ts),1); sigma_hist{1} = sigma;

for i = 1:numel(Model.ts)-1
   w = Model.ws(i,:)';
 
   % True State (no process noise)
   q = Model.qs(i,:)'; % Current q
   q = Model.qs(i+1,:)'; % "State Update" to next q
   % Sensor Measurements in True State (with Noise)
   gyro_meas = Model.gyro_meas(i,:)';   
   accel_meas = Model.accel_meas(i,:)';
   mag_meas = Model.mag_meas(i,:)';
   
   y = vertcat(accel_meas,mag_meas);

   % Update accel Jacobian
   H1 = accel_jacob(mu);
   H2 = mag_jacob(mu);
   C = vertcat(H1,H2);
   
   % EKF Predict
   % Propagate estimated quaternion forward
   [mu,A] = QuatIntegration(mu,gyro_meas,Model.dt);
   % Propagate covariance forward
   sigma = A*sigma*A' + R_sigma;
   
   % EKF Update
   K = sigma*C'*inv(C*sigma*C' + Q_sigma);
   y_expect = vertcat(accel_expected(mu),mag_expected(mu));
   mu = mu + K*(y-y_expect);
   mu = mu/norm(mu);
   sigma = (eye(4) - K*C)*sigma;
   
   mu_hist(i+1,:) = mu;
   sigma_hist{i+1} = sigma;
end

%% Plot Simulation Results
for i = 1:4
    figure();
    plot(Model.ts,[Model.qs(:,i),mu_hist(:,i)]);
    title(sprintf('Quaternion: Component %d',i-1));
end



