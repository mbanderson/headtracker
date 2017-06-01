function [w] = AxisAngVals(axis,ts)
%AXISANGVALS Generates angular velocity values based on model.
%   Uses AngVelModel.m to simulate angular velocity values for a single
%   rotation axis.
%   INPUTS:
%       axis - (int) 1: x-axis, 2: y-axis, 3: z-axis
%       ts - (optional)(Nx1 float vec) time domain
%   OUTPUTS:
%       w - (Nx3 float vec) angular velocity values RAD/S
%

narginchk(1,2);
if nargin < 2
    t0 = 0;
    dt = 0.1;
    tf = 100;
    ts = t0:dt:tf; % sim time, s
end
w = zeros(numel(ts),3); % RAD/S

% Maximum angular velocity
ysat = 0.5; % max ang vel, RAD/S
T = 2.5; % rotation period, s

% Generate model values and assign to specified axis
ys = AngVelModel(T,ysat,ts);
w(:,axis) = ys;

end