function [x_f] = motionModel(M,F,T,x,ts)
%motionModel uses ode45 propogate a motion model of a particle with
%attitude
%   INPUTS:
% 
%   M = particle information structure with following elements
%   M.m = mass of particle
%   M.J = rotational inertia matrix
% 
%   F = 3D force input in body frame
%   T = 3D torque input about body frame
% 
%   x = 12x1 state vector containing following state information
%   x(1:3) = 3D position information in inertial frame
%   x(10:12) = angular rate of particle wrt inertial, expressed in body
%   x(4:6) = 3D velocity of partile in inertial frame
%   x(7:9) = euler angles corresponding to a 3-1-2 rotation
%   in radians
% 
%   ts = timestep
% 
%   OUTPUTS:
%   x_f = 12x1 vector with state information in same format as input

%call ode45
[~, xHist] = ode45( @(t,X)odeMotionFunction(t,X,M,F,T), [0 ts], x);

%create output structure
x_f = xHist(end,:);
end


