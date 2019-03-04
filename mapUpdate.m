function [X_sba, x_sba, R_sba] = mapUpdate(I_KF, X_hat, x_hat, R_hat, K)
%mapUpdate updates the map using the sba library
% %   INPUTS:
%     I_KF: 2xNxM array containing pixel information of each keyframe. N
%     points, M keyframes
%     X_hat: 3xN array containing estimates of feature
%     locations x_hat: 3xM camera position estimates R_hat: 3x3xM camera
%     rotation matrix estimates, inertial to camera frame K: camera
%     calibration matrix
%     
%     OUTPUTS: X_sba: 3xN array containing information about bundle
%     adjusted feature locations x_sba: 3xM estimates of camera positions
%     R_sba: 3x3xM estimates of camera poses

cams = dcm2quat(R_hat);
cams(:,5:7) = x_hat';

[refined_cams, X_sba] = sba(cams, X_hat', I_KF, K);

X_sba = X_sba';

x_sba = refined_cams(:,5:7)';
R_sba = quat2dcm(refined_cams(:,1:4));
end

