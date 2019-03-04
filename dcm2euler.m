function [e] = dcm2euler(R_BW)
% dcm2euler : Converts a direction cosine matrix R_BW to Euler angles phi =
% e(1), theta = e(2), and psi = e(3) (in radians) for a 3-1-2
% rotation.
%
% Let the world (W) and body (B) reference frames be initially aligned. In
% a 3-1-2 order, rotate B away from W by angles psi (yaw, about the body Z
% axis, -pi to pi), phi (roll, about the body X axis, -pi/2 to pi/2), and
% theta (pitch, about the body Y axis, -pi/2 to pi/2). R_BW can then be
% used to cast a vector expressed in W coordinates as a vector in B
% coordinates: vB = R_BW * vW
%
% INPUTS
%
% R_BW ------- 3-by-3 direction cosine matrix
%
%
% OUTPUTS
%
% e ---------- 3-by-1 vector containing the Euler angles in radians: phi =
% e(1), theta = e(2), and psi = e(3)
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author: Corey Marcus
%+==============================================================================+

%initialize e
e = zeros(3,1);

%calculate phi
e(1) = asin(R_BW(2,3));

cP = cos(e(1));

%calculate psi
e(3) = atan2(-R_BW(2,1),R_BW(2,2));

%calculate theta
e(2) = atan2(-R_BW(1,3),R_BW(3,3));
end
    
    
    