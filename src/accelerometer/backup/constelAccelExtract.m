% AA 273 | Final Project | Accelerometer Measurement Extract
% (for Constellation)
% Jin Woo Park
% June 1, 2017

function [r_jc_c_ddot_est, r_cw_c_ddot_est, w_jc_c_dot_est, W, ...
            W_centripetal, W_tangential] = ...
            constelAccelExtract(r_jw_c_ddot, pair_r_cw)
    
% To extract kinematic center translational acceleration and acceleration
% at the sensor with respect to the kinematic center from given
% accelerometer sensor measurements
%   INPUTS:
%    r_jw_c_ddot: The accelerometer sensor readings, column vector of
%                 (3x8,1) = (24,1).
%      pair_r_cw: The pair index of accelerometers that cancels out to leave
%                 with only kinematic center acceleration.
% w_jc_c_dot_est: The angular acceleration estimate from accelerometer
%                 constellation measurements
%
%   OUTPUTS:
%   r_jc_c_ddot: The acceleration component due to head rotation. (angular
%                velcoity) [24 x 1]
%   r_cw_c_ddot: The accelerometer measurement reading. (gravitation)
%                [3 x 1]
%             W: The angular acceleration matrix
%                ( W =  w_jc_c_dot^ + (w_jc_c_dot^)^2 ).
% W_centripetal: The angular acceleration component in W, w_jc_c_dot^
%  W_tangentail: The angular velocity component in W, (W_jc_c_dot^)^2 .

    % function handle: uncrossop
    uncrossop = @(u) (1/2)*[u(3,2)-u(2,3); u(1,3)-u(3,1); u(2,1)-u(1,2)];

    % obtaining the acceleration of center of head with respect to center
    % frame: r_cw_ddot_est
    r_cw_c_ddot_est = zeros(3,1);

    for i = 1:4 % (numAcc/2)
        r_cw_b_ddot_pair( 3*(i-1)+1 : 3*i , 1 ) = ...
            (1/2) * ( r_jw_c_ddot( pair_r_cw(i,1:3) , 1 ) + r_jw_c_ddot( pair_r_cw(i,4:6) , 1 ) );
        r_cw_c_ddot_est = r_cw_c_ddot_est + r_cw_b_ddot_pair( 3*(i-1)+1:3*i, 1 );
    end

    r_cw_c_ddot_est = r_cw_c_ddot_est / 4;   % Averaging (divide by no. of pairs)
    
    r_jc_c_ddot_est = r_jw_c_ddot - repmat(r_cw_c_ddot_est,8,1);
    
    % geometry of the rigid body: l = w = h = 0.04m (4 cm)
    [r_jc_c, ~, ~, invR] = getAccNumLoc (0.04,0.04,0.04);
    
    % angular acceleration matrix: W = w_jc_c_dot^ + (w_jc_c_dot^)^2. 
    W = reshape(r_jc_c_ddot_est,3,8) * r_jc_c' * invR;
    
    W_centripetal = (1/2)*(W + W');
    W_tangential = W - W_centripetal;
    
    w_jc_c_dot_est = uncrossop(W_tangential);
    
end