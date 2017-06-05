function [w,wdot,wdot_handles] = AxisAngVals(axis,ts)
%AXISANGVALS Generates angular velocity values based on model.
%   Uses AngVelModel.m to simulate angular velocity values for a single
%   rotation axis.
%   INPUTS:
%       axis - (int vec) Enable specific axes -> 1: x-axis, 2: y-axis, 3: z-axis
%       ts - (optional)(Nx1 float vec) time domain
%   OUTPUTS:
%       w - (Nx3 float vec) angular velocity values RAD/S
%       wdot - (Nx3 float vec) angular acceleration values RAD/S^2
%       wdot_handles - (cell) handles for wdot on each input axis
%           NOTE: Order of returned handles is order of input axes
%

narginchk(1,2);
if nargin < 2
    dt = 0.01;
    t0 = dt;
    tf = 60;
    ts = t0:dt:tf; % sim time, s
end
w = zeros(numel(ts),3); % RAD/S
wdot = zeros(numel(ts),3); % RAD/S^2

nominal_head_rate = 144; % deg/s
amp = nominal_head_rate * pi/180; % max angular velocity, rad/s
T = 2.5; % rotation period, s

% Generate model values for each axis
wdot_handles = cell(numel(axis),1);
for i = 1:numel(axis)
    [w_handle, wdot_handle] = AngVelModel(T,amp);
    wvals = w_handle(ts); wdot_vals = wdot_handle(ts);  
    w(:,axis(i)) = wvals;
    wdot(:,axis(i)) = wdot_vals;
    
    % Return wdot handle in case initial conditions needed
    wdot_handles{i} = wdot_handle;
end

end