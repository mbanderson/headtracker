% AA 273 | Final Project | Accelerometer Measurement Extract
% (for Constellation)
% Jin Woo Park
% June 1, 2017

function [w_jc_c_TA, w_jc_c_CANP] = constelAngVel(W, w_jc_c_dot_est, deltaT, w_jc_c0)

% Computes the estimated angular velocity using angular acceleration info.
%   INPUTS:
%               W: The angular acceleration matrix
%                   ( W =  w_jc_c_dot^ + (w_jc_c_dot^)^2 )
%  w_jc_c_dot_est: The angular acceleration extraction from constellation
%                   ( expects 3 x 2 matrix where 2 columns are t-1 and t)
%          deltaT: The duration of time step
%
%  OUTPUTS:
%    w_jc_c_est: 
%

%% TA Method
% time at t-1;
if nargin == 3 % unknown initial condition (i.e., estimate using constellation only)
    W_centripetal = (1/2)*(W + W');
    W_extract = (-1/2)*trace(W_centripetal)*eye(3) + W_centripetal;
    w_jc_c0_abs = [ sqrt(W_extract(1,1)); sqrt(W_extract(2,2)); sqrt(W_extract(3,3))];

    if W_extract(1,2) > 0
        w_jc_c0_sign = [1;  1; 0;];
    else
        w_jc_c0_sign = [1; -1; 0;];
    end

    if W_extract(1,3) > 0
        w_jc_c0_sign(3,1) = w_jc_c0_sign(1,1);
    else
        w_jc_c0_sign(3,1) = -w_jc_c0_sign(1,1);
    end
    
    w_jc_c_TA = diag(w_jc_c0_sign)*w_jc_c0_abs;

else % known initial condition
    w_jc_c_TA = w_jc_c0;
end

% time at t
w_jc_c_TA = w_jc_c_TA + (1/2) * deltaT * ...
                (w_jc_c_dot_est(:,1) + w_jc_c_dot_est(:,2));
            
%% CANP method

% Adjoint Matrix ( i.e. Cofactor^T )
adjMat = @(Mat) cofactor(Mat)';

% 1st IF STATEMENT
    if w_jc_c_TA == zeros(1,3)
        w_jc_c_CANP = zeros(1,3);
    end

    i = 1;
    W_centripetal = (1/2) * ( W + W' );
    mu_2 = - ( W_centripetal(1,1) + W_centripetal(2, 2) + W_centripetal(3,3) );
    s_12 = W_centripetal( i , 2 )^2;
    s_23 = W_centripetal( 2 , 3 )^2;
    s_31 = W_centripetal( 3 , 1 )^2;
    s_13 = W_centripetal( i , 2 )*W_centripetal( 2 , 3 );
    d_12 = W_centripetal( i , 1 )*W_centripetal( 2 , 2 );
    mu_1 = d_12 + W_centripetal(2, 2)*W_centripetal(3, 3) + W_centripetal(3, 3)*W_centripetal(i,1) - s_12 - s_23 - s_31;
    mu_0 = -d_12*W_centripetal(3, 3) - 2*s_13*W_centripetal(3, 1) + s_12*W_centripetal(3, 3) + s_23*W_centripetal(i, 1) + s_31*W_centripetal(2, 2);

    nu_2 = (1/3)*mu_2;
    theta_2 = nu_2^2;
    q = (1/3)*mu_1 - theta_2;

    % 2nd IF STATEMENT    
    if q >= 0
        w_jc_c_CANP = zeros(1,3);      
    end
    
    r = (1/2)*(mu_1*nu_2 - mu_0) - nu_2*theta_2;
    alpha = sqrt(-q);
    beta = alpha^3;
    
    % 3rd IF STATEMENT
    if beta <= r
        w_jc_c_CANP = zeros(1,3);
    end
    
    lambda = 2*alpha*cos( acos(r/beta)/3 ) - nu_2;
    delta = (lambda + mu_2) / 2;

    % 4th IF STATEMENT    
    if (delta <= 0) || ( lambda*mu_0 > 0 )
        w_jc_c_CANP = zeros(1,3);              
    end
    
    w_ba_b_CANP_abs = sqrt(delta);
%     zeta_11 = W_centripetal(i, 1) - lambda;
%     zeta_22 = W_centripetal(2, 2) - lambda;
%     zeta_33 = W_centripetal(3, 3) - lambda;
%     xi_11 = zeta_22*zeta_33 - s_23;
%     xi_22 = zeta_33*zeta_11 - s_31;
%     xi_33 = zeta_11*zeta_22 - s_12;
%     xi_12 = W_centripetal(2,3)*W_centripetal(3,1) - W_centripetal(i,2)*W_centripetal(3,3);
%     xi_23 = W_centripetal(i,2)*W_centripetal(3,1) - W_centripetal(2,3)*W_centripetal(i,1);
%     xi_31 = s_13 - W_centripetal(3,1)*W_centripetal(2,2);

    Xb = W_centripetal - lambda*ones(3);
    v_b = adjMat(Xb)*w_jc_c_TA ;

    w_jc_c_CANP = ( w_ba_b_CANP_abs / norm(v_b) ) * v_b';

end