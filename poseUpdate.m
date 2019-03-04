function [x_hat, R_hat] = poseUpdate(I, x0, R0, X, K)
%poseUpdate Performs nonlinear LS to estimate pose based on most recent
%image
%   INPUTS:
%     I: 2xN image, containing pixel location information for N features
%     x0: camera position initial estimate
%     R0: Estimate of DCM from inertial to camera coordinate frames
%     
%     OUTPUTS:
%     x_hat: LS position estimate
%     R_hat: LS DCM estimate

%create anoynmous function
f_min = @(u)reprojE(u,I,X,K);

%generate initial guess at u
u0 = zeros(6,1);
u0(1:3) = x0;
u0(4:6) = dcm2angle(R0, 'XYZ');

%call nonlinear least squares
options = optimoptions('lsqnonlin','Display','iter','Algorithm','levenberg-marquardt',...
    'FunctionTolerance',1e-8);
u_hat = lsqnonlin(f_min, u0,[],[],options);

%assign output variables
x_hat = u_hat(1:3);
R_hat = angle2dcm(u_hat(4), u_hat(5), u_hat(6), 'XYZ');
end

