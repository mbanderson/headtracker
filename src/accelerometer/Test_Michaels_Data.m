% Accelerometer-based Angular Velocity Measurement Simulation
% Rate Gyro + Accelerometer

% Jin Woo Park
% June 4, 2017

clear all
close all
clc

load('model.mat');

%% Initial conditions

noisy = true; %false;
% samplingRate = 100;
% tf = 50;
% tspan = 0:(1/samplingRate):tf;
% N = length(tspan);

dt = Model.dt;
tspan = Model.ts;
tf = tspan(end);
N = length(tspan);

%% Get Accelerometer Quantity and Location
[r_jc_c, pair_r_cw, P_c, invR] = getAccNumLoc (0.04,0.04,0.04);

%% The Equation of Motion of the Brick (NEED ANGULAR VELOCITY)

w_jc_c = deg2rad(Model.ws)';
w_jc_c_dot = [diff(w_jc_c(1,:));diff(w_jc_c(2,:));diff(w_jc_c(3,:))]/dt;
% w_jc_c_dot(:,end+1) = w_jc_c_dot(:,end);
w_jc_c_dot(:,2:end+1) = w_jc_c_dot;
w_jc_c_dot(:,1) = w_jc_c_dot(:,2)/10;

%% DATA GENERATION AND EXTRACTION
% Generate Sensor Readout (HALF DONE) (r^ic_b_ddot) % if r^cw_b = 0
[r_jw_c_ddot, r_jc_c_ddot, r_cw_c_ddot] = constelAccelGen(w_jc_c, w_jc_c_dot, tspan, noisy);
 
r_jc_c_ddot_est = zeros(24,N);
r_cw_c_ddot_est = zeros(3,N);
r_cw_w_ddot_est = zeros(3,N);
w_jc_c_dot_est = zeros(3,N);
w_jc_c_TA = zeros(3,N);
w_jc_c_CANP = zeros(3,N);
 
% Intial quaterion position
q0 = initialQuaternion (0,0,0); % assume coincident
q = zeros(4,tf); q(:,1) = q0;
 
for i = 1:N
    
    [r_jc_c_ddot_est(:,i), r_cw_c_ddot_est(:,i), w_jc_c_dot_est(:,i), ...
        W, W_centripetal, W_tangential] = ...
            constelAccelExtract(r_jw_c_ddot(:,i), pair_r_cw);
    if i == 1
        w_jc_c_TA = w_jc_c(:,1);
        w_jc_c_CANP = w_jc_c(:,1);
    else
        [w_jc_c_TA(:,i), w_jc_c_CANP(:,i)] = ...
            constelAngVel(W, w_jc_c_dot_est(:,i-1:i), dt, w_jc_c_TA(:,i-1));
    end
    
    
    % Body to World
    % estimate r_cw_c_ddot
    qdot = getqdot(w_jc_c(:,i),q(:,i));     % time rate change of q
        if i == 1
            r_cw_w_ddot_est(:,1) = getRw2b(q(:,1))'*r_cw_c_ddot_est(:,1);
            delt = tspan(2)-tspan(1);
            qnext = q(:,1) + delt*qdot;
            q(:,2) = qnext/norm(qnext);
        else
            delt = tspan(i) - tspan(i-1);         % time difference 
            qnext = q(:,i) + delt*qdot;           % integration
            q(:,i+1) = qnext/norm(qnext);         % normalization
            
            Rb2w = getRw2b(q(:,i));
            r_cw_w_ddot_est(:,i) = Rb2w'*r_cw_c_ddot_est(:,i);
        end
    
end

%% Plots

figure(1); clf;
for j = 1:3
    subplot(3,1,j); hold on; grid on
    plot(tspan, w_jc_c(j,1:end-1),'linew', 2)
    plot(tspan, w_jc_c_TA(j,1:end),'r--', 'linew', 2)
%     plot(tspan, w_jc_c_CANP(j,:),'k:','linew', 2) 
%     ylim([-0.5 0.5])
    legend('True','TA','CANP', 'location', 'best');
    title(['$\omega^{jc}_{c,' num2str(j) '}$'], 'interpreter', 'latex', 'fontsize', 12);
end

figure(2); clf;
for j = 1:3
    subplot(3,1,j); hold on; grid on
    plot(tspan, r_cw_c_ddot(1,1:end-1), 'linew', 2);
    plot(tspan, r_cw_c_ddot_est(1,1:end), 'r--', 'linew', 2);
    legend('True', 'Estimate', 'location', 'best');
    title(['$\ddot{r}^{cw}_{c,' num2str(j) '}$'], 'interpreter', 'latex', 'fontsize', 12);
end

figure(3); clf;
for j = 1:3
    subplot(3,1,j); hold on; grid on
    plot(tspan, r_cw_w_ddot_est(j,1:end), 'r--', 'linew', 2);
    if j == 1 || j == 2, ylim([-1 1]), end
    if j == 3, ylim([-10 10]), end
    title(['$\ddot{r}^{cw}_{w,' num2str(j) '}$'], 'interpreter', 'latex', 'fontsize', 12);
end