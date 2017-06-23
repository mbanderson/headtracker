function [rms,errs,Model] = ekf(options)
%EKF Implements EKF filter for head-tracking.
%   INPUTS:
%       options - (ekfOptions instance) specifies filter parameters
%   OUTPUTS:
%       rms
%       errs
%       Model

narginchk(0,1);
if nargin < 1
    options = ekfOptions();
end

wrapToPi = @(x) mod(x+pi,2*pi)-pi;

%% Single-Stage EKF with Center of Head Measurements
%% Simulate Head Movement and Measurements
Model = HeadDynamicsModel(options.tf,options.noisy);

% Define sampler functions based on input parameters
% Gyroscope Model
% MEMS Gyroscope
if options.gyro_type == 1
    gyro_meas_model = @(i) Model.gyro_meas(i,:)';
% Accelerometer Constellation
elseif options.gyro_type == 2
    gyro_meas_model = @(i) Model.ws_cons(i,:)';
else
    error('Invalid gyroscope model flag.');
end

% Accelerometer Model
r = [0.04,0.04,-0.04]; % m, head-mounted accel offset
% Center of Head
if options.accel_type == 1
    accel_meas_model = @(i,~,~) Model.accel_meas(i,:)';
% Naive
elseif options.accel_type == 2
    accel_meas_model = @(i,~,~) Model.accel_meas_offset(i,:)';
% Compensated
elseif options.accel_type == 3
    accel_meas_model = @(i,w_gyro,wdot_gyro) Model.accel_meas_offset(i,:)' - ...
        (cross(wdot_gyro,r) + cross(w_gyro,cross(w_gyro,r)))';
% Accelerometer
elseif options.accel_type == 4
    accel_meas_model = @(i,~,~) Model.accel_meas_cons(i,:)';
else
    error('Invalid accelerometer model flag.');
end

%% Helper Functions
% Expected measurement models
% Accelerometer model, Jacobian
grav_mag = -1*norm(Model.env_model.grav_vec);
accel_expected = @(q) grav_mag*[2*q(2)*q(4)-2*q(1)*q(3);
    2*q(1)*q(2)+2*q(3)*q(4);
    q(1)^2-q(2)^2-q(3)^2+q(4)^2];
accel_jacob = @(q) grav_mag*[-2*q(3), 2*q(4), -2*q(1), 2*q(2);
    2*q(2),  2*q(1), 2*q(4), 2*q(3);
    2*q(1), -2*q(2) -2*q(3), 2*q(4)];

% Magnetometer model, Jacobian
mag_expected = @(q) [2*q(2)*q(3)+2*q(1)*q(4);
    q(1)^2-q(2)^2+q(3)^2-q(4)^2;
    2*q(3)*q(4)-2*q(1)*q(2)];
mag_jacob = @(q) [2*q(4), 2*q(3), 2*q(2), 2*q(1);
    2*q(1), -2*q(2), 2*q(3), -2*q(4);
    -2*q(2), -2*q(1), 2*q(4), 2*q(3)];

%% Simulator Variables
Q_mu = zeros(6,1);
Q_sigma = blkdiag(Model.accel.sigma,Model.mag.sigma);

R_mu = zeros(4,1);
R_sigma = 0.1*eye(4);

%% EKF
% Initial state estimate
mu = [1,0,0,0]';
% mu = rand(4,1);
% mu = mu / norm(mu);
sigma = 50000*eye(4);

mu_hist = zeros(numel(Model.ts)+2,4); mu_hist(1,:) = mu;
sigma_hist = cell(numel(Model.ts)+2,1); sigma_hist{1} = sigma;

prev_gyro = NaN; wdot_gyro = NaN;
loop_times = zeros(1,numel(Model.ts)+1);
for i = 1:numel(Model.ts)+1
    w = Model.ws(i,:)';
    
    % True State
    q = Model.qs(i+1,:)'; % Updated q
    
    % Sensor Measurements in True State (with Noise)
    gyro_meas = gyro_meas_model(i); % i since no initial measurement
    mag_meas = Model.mag_meas(i,:)';
    
    tic;
    % If doing compensation, use naive model for first iteration (before
    % wdot is defined)
    if ~(options.accel_type == 3)
        accel_meas = accel_meas_model(i,gyro_meas,wdot_gyro);
    else
        if i == 1
            accel_meas = Model.accel_meas(i,:)';
        else
            wdot_gyro = (gyro_meas - prev_gyro) / Model.dt;
            accel_meas = accel_meas_model(i,gyro_meas,wdot_gyro);
        end
        prev_gyro = gyro_meas;
    end        
            
    % Measurement at true state
    y = vertcat(accel_meas,mag_meas);
    
    % Update Accel Jacobian
    A = dynamics_jacob(gyro_meas,Model.dt);
    H1 = accel_jacob(mu);
    H2 = mag_jacob(mu);
    C = vertcat(H1,H2);
    
    % EKF Predict
    % Propagate estimated quaternion forward
    mu = QuatIntegration(mu,gyro_meas,Model.dt);
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
    
    loop_times(i) = toc;
end

% Average filter loop time
avg_time_filter = sum(loop_times)/numel(loop_times);
Model.avg_time_filter = avg_time_filter;

% Write average time back to Model
if options.accel_type == 4
    Model.avg_time_total = Model.avg_time_filter + Model.avg_time_cons;
else
    Model.avg_time_total = Model.avg_time_filter;
end

%% Plot Simulation Results

t_hist = horzcat(0,Model.ts);
if options.plotresults
    % Quaternion plots
    for i = 1:4
        h = figure();
        plot(t_hist,[Model.qs(1:end-1,i),mu_hist(1:end-1,i)]);
        titlestr = sprintf('Quaternion Component: $q_%d$',i-1);
        title(titlestr,'interpreter','latex','fontsize',14);
        xlabel('Time (s)');
        legend('Simulated','Estimated','location','northeast');
        if i ~= 1
            ylim([0,1]);
        end
        grid on;
        savestr = sprintf('quatcompq%d',i-1);
        print(h,savestr,'-dpng');
    end
end

%% Compute RMS Error
% Error calculation based on http://mathworld.wolfram.com/Quaternion.html
quat_inv = @(q) [q(1); -q(2:4)];
quat_mult = @(a,b) [a(1)*b(1)-a(2)*b(2)-a(3)*b(3)-a(4)*b(4);
    a(1)*b(2)+a(2)*b(1)+a(3)*b(4)-a(4)*b(3);
    a(1)*b(3)-a(2)*b(4)+a(3)*b(1)+a(4)*b(2);
    a(1)*b(4)+a(2)*b(3)-a(3)*b(2)+a(4)*b(1)];

nquats = size(Model.qs,1);
errs = zeros(nquats,4);
for i = 1:nquats
    q_true = Model.qs(i,:)';
    q_est = mu_hist(i,:)';
    z = quat_mult(q_true,quat_inv(q_est));
    e_i = 2*asin(z);
    e_i = [e_i(1); 180/pi*wrapToPi(e_i(2:4))];
    errs(i,:) = e_i;
end
rms = sqrt(1/nquats*sum(errs.^2));  

%% Create simulation animation
if ~strcmp(options.gifname,'')  
    % Lock down figure parameters before animating
    h = figure();
    colorstrs = {'b','r','g'};
    truth_origin = {-0.75,0,0}; est_origin = {0.75,0,0};
    
    nquats = size(Model.qs,1);
    i_body = [1,0,0]'; j_body = [0,1,0]'; k_body = [0,0,-1]';
    
    % Plot rotation from each quaternion
    for i = 1:nquats-1
        truth = Model.qs(i,:)';
        est = mu_hist(i,:)';
        
        truth_R = Quaternion.Rbw(truth);
        est_R = Quaternion.Rbw(est);
        
        % Generate world-frame axes
        truth_i = truth_R*i_body; truth_j = truth_R*j_body; truth_k = truth_R*k_body;
        truth_vecs = {truth_i,truth_j,truth_k};
        est_i = est_R*i_body; est_j = est_R*j_body; est_k = est_R*k_body;
        est_vecs = {est_i,est_j,est_k};
        
        % Plot arrows
        for j = 1:3
            t_vec = truth_vecs{j}; e_vec = est_vecs{j};
            quiver3(truth_origin{:},t_vec(1),t_vec(2),t_vec(3),...
                'AutoScale','off','color',colorstrs{j},'LineStyle','-');
            hold on;
            quiver3(est_origin{:},e_vec(1),e_vec(2),e_vec(3),...
                'AutoScale','off','color',colorstrs{j},'LineStyle',':');
        end
        hold off;
        
        % Record
        title(sprintf('Truth (Left) vs. Filter (Right)\nt=%.2f',t_hist(i)),'FontSize',15);
        axis manual;
        xlim([-1.1,1.1]); ylim([-1.1,1.1]); zlim([-1.1,1.1]);
        set(gca,'XTick',[],'YTick',[],'ZTick',[]);
        drawnow; frame = getframe(h);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        if i == 1
            imwrite(imind,cm,gifname,'gif','Loopcount',inf,'DelayTime',0);
        else
            imwrite(imind,cm,gifname,'gif','WriteMode','append','DelayTime',0);
        end
    end  
end

%% Save quaternions to file for animation in Blender
if ~strcmp(options.quatname,'')
    fname1 = strcat('q_',quatname);
    fname2 = strcat('mu_',quatname);
    
    dlmwrite(fname1,Model.qs);
    dlmwrite(fname2,mu_hist);
end

end

