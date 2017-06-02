% AA 273 | Final Project | Accelerometer Measurement Generator 
% (for Constellation)
% Jin Woo Park
% June 1, 2017

% To extract angular acceleration from accelerometer sensor measurements
%   INPUTS:
%       w_jc_c: Angular velocity of head movement in head center frame
%               matrix of size (3,N).
%  w_jc_c_ddot: Angular acceleration, finite difference of w_jc_c.
%
%
%   OUTPUTS:
%  r_jw_c_ddot: The accelerometer measurement reading.
%  r_jc_c_ddot: The acceleration component due to head rotation. (for debugging)
%

function [r_jw_c_ddot, r_jc_c_ddot] = constelAccelExtract(r_i, w_jc_c, w_jc_c_dot)
    % initializing
    r_cw_b_ddot_est = zeros( 3 , 1 );

    for i = 1:(numAcc/2)
        r_cw_b_ddot_pair( 3*(i-1)+1 : 3*i , 1 ) = ...
            (1/2) * ( r_iw_b_ddot( pair_r_cw(i,1:3) , 1 ) + r_iw_b_ddot( pair_r_cw(i,4:6) , 1 ) );
        r_cw_b_ddot_est = r_cw_b_ddot_est + r_cw_b_ddot_pair( 3*(i-1)+1:3*i, 1 );
    end

    r_cw_b_ddot_est = r_cw_b_ddot_est / (numAcc/2);   % Averaging (divide by no. of pairs)
end