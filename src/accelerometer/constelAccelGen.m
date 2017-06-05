% AA 273 | Final Project | Accelerometer Measurement Generator 
% (for Constellation)
% Jin Woo Park
% June 1, 2017

function [r_jw_c_ddot, r_jc_c_ddot, r_cw_c_ddot] = constelAccelGen(w_jc_c, w_jc_c_dot, t, noisy)

% To generate accelerometer sensor reading given angular velocity and accelerations 
%   INPUTS:
%       w_jc_c: Angular velocity of head movement in head center frame
%               matrix of size (3,N).
%  w_jc_c_ddot: Angular acceleration, finite difference of w_jc_c.
%           t : time at each step
%
%
%   OUTPUTS:
%  r_jw_c_ddot: The accelerometer measurement reading.
%  r_jc_c_ddot: The acceleration component due to head rotation. (for debugging)
%

    % function handles
    crossop = @(w) [ 0, -w(3), w(2); 
                   w(3), 0, -w(1);
                  -w(2), w(1), 0  ];   
              
    gAccel = -9.81;     % m/s^2
    r_cw_w_ddot = [0; 0; gAccel];

    Q = 1.0e-04*[0.1042, 0, 0.0004;
                 0, 0.1010, -0.0005;
                 0.0004, -0.0005, 0.2463];
    
    tf = length(w_jc_c_dot); % or tf = length(t)
        
    % Consetellation Dimensions in meter (4 cm)
    l = 0.04; w = 0.04; h = 0.04;
    
    % Get the Location of Accelerometers
    [r_jc_c, ~, ~, ~] = getAccNumLoc (l,w,h);

    % Intial quaterion position
    q0 = initialQuaternion (0,0,0); % assume coincident
    q = zeros(4,tf); q(:,1) = q0;
    
    r_jc_c_ddot = zeros(24,tf);
    r_cw_c_ddot = zeros(24,tf);
    r_jw_c_ddot = zeros(24,tf);
    
    for i = 1:tf
        
        W_c_j = crossop(w_jc_c_dot(:,i)) + ( crossop(w_jc_c(:,i)) * crossop(w_jc_c(:,i)) );
        for n = 1:8
            r_jc_c_ddot( 3*(n-1)+1:3*n, i ) = W_c_j * r_jc_c(:,n);
        end
        
        % estimate r_cw_c_ddot
        qdot = getqdot(w_jc_c(:,i),q(:,i));     % time rate change of q
        if i == 1
            r_cw_c_ddot(:,1) = repmat(getRw2b(q(:,1))*r_cw_w_ddot,8,1);
            delt = t(2)-t(1);
            qnext = q(:,1) + delt*qdot;
            q(:,2) = qnext/norm(qnext);
        else
            delt = t(i) - t(i-1);                   % time difference 
            qnext = q(:,i) + delt*qdot;           % integration
            q(:,i+1) = qnext/norm(qnext);           % normalization
            
            Rw2b = getRw2b(q(:,i));
            r_cw_c_ddot(:,i) = repmat(Rw2b*r_cw_w_ddot,8,1);
        end
        
        % accelerometer sensor reading
        r_jw_c_ddot(:,i) = r_jc_c_ddot(:,i) + r_cw_c_ddot(:,i);
        
        if noisy
            r_jw_c_ddot(:,i) = r_jw_c_ddot(:,i) ...
                + mvnrnd(zeros(24,1), blkdiag(Q,Q,Q,Q,Q,Q,Q,Q))';
        end
        
    end

end