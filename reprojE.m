function [e] = reprojE(x, I_obs, X, K)
%reprojE detirmines the reprojction error between an observed image and an
%predicted image. To be used for minimization in pose update
% 
%     INPUTS: 
%     x is a 6x1 vector which is to be minimized. the first 3 elements
%     correspond to camera position. While the final 3 are euler angles for
%     a 1-2-3 rotation in radians
% 
%     I_obs is the 2xN observed image. contains feature locations
%     in pixels
%     
%     X is a 3xN matrix containing spacial coordinates of the N observed
%     points
%     
%     K is the 3x3 camera calibration matrix
%     
%     OUTPUTS:
%     
%     e is the 2Nx1 reprojection error for each feature. Features which are
%     not visible in one image will have something done with them. Not sure
%     what yet. For now, they've been assigned an abitrary error
%     

%create rotation matrix
R = angle2dcm(x(4), x(5), x(6), 'XYZ');

%generate reprojected image
I_pred = imageGen(X, x(1:3), R, K);

%calculate reprojection error
e = I_obs - I_pred;

%reshape e
[~, N] = size(e);
e = reshape(e,2*N,1);

%trim out NaN values, happens when point out of bounds of image.
e(isnan(e)) = 100;

end

