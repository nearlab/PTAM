clear
close all
clc

ts = .01;
M.m = 1;
M.J = eye(3);
F = [0 0 0]';
T = [1 0 0]';

L = 100;

%initialize state
x = zeros(12,L+1);

%position
x(1:3,1) = [0 0 0]';

%velocity
x(4:6,1) = [0 0 0]';

%attitude
x(7:15,1) = reshape(eye(3),9,1);

%angular velocity
x(16:18,1) = [0 0 0]';

for ii = 1:L
   x(:,ii+1) = motionModel(M,F,T,x(:,ii),ts);
   R(:,:,ii) = reshape(x(7:15,ii), 3, 3);
end

figure
plot3(x(1,:),x(2,:),x(3,:))