%% Single-Stage EKF with Center of Head Measurements
%
clear Model; close all; clc;
wrapToPi = @(x) mod(x+pi,2*pi)-pi;
%% Simulate Head Movement and Measurements
Model = HeadDynamicsModel();

% () = a_g + (wdot + w x w)*r + v
% Want a_g

% Ground truth in the body frame:
% a_truth_body = a_truth_grav_vec + (wdot_truth + w_truth x w_ruth)*offset
% + v
%
% a_expected = a_g_expected + (gyrodot + w_gyro x w_gyro)*offset + v
% a_expected = Rwb(mu) + (gyrodot + gyro x gyro)*offset + v


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
sigma = 50000*eye(4);

mu_hist = zeros(numel(Model.ts)+2,4); mu_hist(1,:) = mu;
sigma_hist = cell(numel(Model.ts)+2,1); sigma_hist{1} = sigma;

prev_gyro = NaN;
for i = 1:numel(Model.ts)+1 
   w = Model.ws(i,:)';
 
   % True State (no process noise)
   q = Model.qs(i+1,:)'; % Updated q
   % Sensor Measurements in True State (with Noise)
   gyro_meas = Model.gyro_meas(i,:)'; % i since no initial measurement
   %accel_meas = Model.accel_meas(i,:)'; % Center of Head
   
   %accel_meas = Model.accel_meas_offset(i,:)'; % Naive Offset
   if isnan(prev_gyro)
       accel_meas = Model.accel_meas(i,:)'; % Naive first iteration
   else
       wdot_gyro = (gyro_meas - prev_gyro) / Model.dt;
       wdot_gyro_rad = wdot_gyro * pi/180;
       w_gyro_rad = gyro_meas * pi/180;
       
       r = [0.04,0.04,-0.04];
       pred_linacc = cross(wdot_gyro_rad,r) + cross(w_gyro_rad,cross(w_gyro_rad,r));
       accel_meas = Model.accel_meas_offset(i,:)' - pred_linacc';
       
       % FLAG
       %accel_meas = Quaternion.Rwb(q)*accel_meas;
   end
   prev_gyro = gyro_meas;
   
   if i == 10
       disp('hi');
   end
      
   mag_meas = Model.mag_meas(i,:)';
   
   y = vertcat(accel_meas,mag_meas);

   % Update accel Jacobian
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
end

%% Plot Simulation Results
t_hist = horzcat(0,Model.ts);
for i = 1:4
    figure();
    plot(t_hist,[Model.qs(1:end-1,i),mu_hist(1:end-1,i)]);
    title(sprintf('Quaternion: Component %d',i-1));
end

%% Write Quaternion Histories to File
fname1 = 'q_hist.txt';
dlmwrite(fname1,Model.qs);

fname2 = 'mu_hist.txt';
dlmwrite(fname2,mu_hist);

%% Compute RMS Error
% http://mathworld.wolfram.com/Quaternion.html
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
RMS = sqrt(1/nquats*sum(errs.^2));
fprintf('RMS Error for (x,y,z):\n'); disp(RMS);

%% Create simulation animation
user = input('Create output gif? [y/n]: ','s');
if strcmp(user,'y')
    gifname = input('Output file name (extension included): ','s');
    
    % Lock down figure parameters
    h = figure();
    
    colorstrs = {'b','r','g'};
    truth_origin = {-0.75,0,0}; est_origin = {0.75,0,0};
    
    nquats = size(Model.qs,1);
    i_body = [1,0,0]'; j_body = [0,1,0]'; k_body = [0,0,-1]';
    % Plot rotation from each quaternion
    for i = 1:nquats
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
            imwrite(imind,cm,gifname,'gif','Loopcount',inf);
        else
            imwrite(imind,cm,gifname,'gif','WriteMode','append');
        end
    end
    
end
