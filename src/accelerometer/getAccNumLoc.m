% AA 273 | Final Project | Accelereometer: Quantity and Location
% (for Constellation)
% Jin Woo Park
% June 1, 2017 

% To generate accelerometer sensor reading given angular velocity and accelerations 
%   INPUTS:
%           l: length (m)
%           w: width (m)
%           h: height (m)
%
%   OUTPUTS:
%      r_jc_c: The accelerometer measurement reading.
%   pair_r_cw: The acceleration component due to head rotation. (for debugging)
%         P_c: 

 function [r_jc_c, pair_r_cw, P_c, invR] = getAccNumLoc (l,w,h)
 
    % 8 Accelerometers
    % (x y z)' in Strapdown Frame: (3 x 1)
    r_jc_c(:,1) = [ l/2   -w/2   h/2]';
    r_jc_c(:,2) = [ l/2    w/2   h/2]';
    r_jc_c(:,3) = [ l/2   -w/2  -h/2]';
    r_jc_c(:,4) = [ l/2    w/2  -h/2]';
    r_jc_c(:,5) = [-l/2   -w/2   h/2]';
    r_jc_c(:,6) = [-l/2    w/2   h/2]';
    r_jc_c(:,7) = [-l/2   -w/2  -h/2]';
    r_jc_c(:,8) = [-l/2    w/2  -h/2]';

    % Simply, adding all sensor readouts and divide by numAcc will give r_co_c_ddot 
    % This Matrix is Pairing Symmetric Points
    pair_r_cw = [ 1:3     22:24;  
                  4:6     19:21;    
                  7:9     16:18;   
                  10:12   13:15 ];
    
    % function handles
    crossop = @(w) [ 0, -w(3), w(2); 
                     w(3), 0, -w(1);
                    -w(2), w(1), 0  ];   

    % Calculate P^ic_c
	P_c = zeros(3,3*8);
    for i = 1:8
        P_c(:,3*(i-1)+1:3*i) = -(1/2)*crossop(r_jc_c(:,i));
    end 
    
    R = r_jc_c*r_jc_c';
    invR = inv(R);
    
%     r_ic_b_col = reshape(r_ic_b,3*numAcc,1);
 end