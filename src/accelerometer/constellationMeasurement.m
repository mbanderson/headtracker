% Accelerometer-based Angular Velocity Measurement Simulation
% Rate Gyro + Accelerometer

% Jin Woo Park
% June 4, 2017

% clear all
% close all
% clc

function [w_jc_c_est, w_jc_c_dot_est, r_cw_c_ddot_est, accel_meas_offset] = ...
    constellationMeasurement(Model, noisy)

    %% Initial conditions

%     noisy = true; %false;
    % samplingRate = 100;
    % tf = 50;
    % tspan = 0:(1/samplingRate):tf;
    % N = length(tspan);

    dt = Model.dt;
    tspan = [0 Model.ts];
    tf = tspan(end);
    N = length(tspan);

    %% Get Accelerometer Quantity and Location
    [r_jc_c, pair_r_cw, P_c, invR] = getAccNumLoc (0.04,0.04,0.04);

    %% The Equation of Motion of the Brick (NEED ANGULAR VELOCITY)

    w_jc_c = deg2rad(Model.ws)';
    w_jc_c_dot = deg2rad(Model.wdots)';
%     w_jc_c_dot = [diff(w_jc_c(1,:));diff(w_jc_c(2,:));diff(w_jc_c(3,:))]/dt;
%     w_jc_c_dot(:,2:end+1) = w_jc_c_dot;
%     w_jc_c_dot(:,1) = w_jc_c_dot(:,2)/10;

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

    w_jc_c_est = w_jc_c_TA;
    
    % to have same format with Michael
    w_jc_c_est = w_jc_c_est';
    w_jc_c_dot_est = w_jc_c_dot_est';
    r_cw_c_ddot_est = r_cw_c_ddot_est';
    accel_meas_offset = r_jc_c_ddot_est(10:12,:)'+r_cw_c_ddot_est;
end
