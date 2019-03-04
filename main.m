clear
close all
clc

% Corey Marcus
% PTAM Implementation

%Camera properties
%Resolution = 1280x960
f = 1000; %camera focal length
p1 = 640; %principal point x
p2 = 480; %principal point y
s = 0; %skew

K = [f s p1;
    0 f p2;
    0 0 1]; %camera calibration matrix

%create experiment
Nx = 30;
Ny = 10;
N = Nx*Ny; %total number of points
obj_x = [linspace(-.5,.5,Nx)];% linspace(0,3,Nx/2)];
obj_y = [linspace(-.75,.75,Ny)];% linspace(0,.1,Ny/2)];
obj_z = 10;

%initialize points
X = zeros(3,N);
kk = 1;

for ii = 1:Nx
    for jj = 1:Ny
        X(:,kk) = [obj_x(ii) obj_y(jj) obj_z]';
        kk = kk + 1;
    end
end

%initialize map estimate
X_hat = X; %+ sqrt(.0001)*randn(3,N);

%number of camera images
M = 4*60;

%frame rate (hZ)
fr = 4*30;
ts = 1/fr;

%motion model parameters
F = [0*ones(1,M);
    0*ones(1,M);
    0*ones(1,M)];

T = [0*ones(1,M);
    0*ones(1,M);
    0*ones(1,M)];

Mod.m = 1;
Mod.J = eye(3);

%initialize estimated state
x_hat = zeros(18,M);

%position
x_hat(1:3,1) = [0 0 0]';

%velocity
x_hat(4:6,1) = [.2 0 0]';

%attitude
x_hat(7:15,1) = reshape(eye(3),9,1);

%angular velocity
x_hat(16:18,1) = [0 0 0]';

%initialize images
I = zeros(2,N,M);
I_hat = zeros(2,N,M);

%initialize error vectors
e_reproj = zeros(1,M);
e_pos = zeros(1,M);
e_map = zeros(1,M);
e_quat = zeros(M,4);
e_eul = zeros(M,3);
e_R = zeros(3,3,M);

%initialize camera vectors
cVect = zeros(3,M);
cVect_hat = zeros(3,M);

%initialize truth model
sig_F = 0;
sig_T = 0;

x = x_hat;
x(1:6,1) = x(1:6,1);% + sqrt(0.001)*randn(6,1); %uncertainty in initial position and velocity
x(16:18,1) = x(16:18,1);% + sqrt(0.001)*randn(3,1); %uncertainty in initial angular velocity

%add uncertainty in attitude
R = reshape(x(7:15,1),3,3);
%R = R*angle2dcm(sqrt(0.001), sqrt(0.001), sqrt(0.001), 'XYZ');
x(7:15,1) = reshape(R,9,1);

%keyframe parameters
kf_idx = 3; %keyframe index (start at 3)
kf_cnt = 3; %keyframe counter to indicate when a new one should be formed
kf_freq = 20; %frequency of keyframe creation
I_kf = zeros(2,N,2); %unknown number of keyframes will be created
x_hat_kf = zeros(3,2); %we need to keep track of the mapping updated poses
R_kf = zeros(3,3,2);

%% Initialize Simulation
%first two camera frames become keyframes so we'll do them outside the main
%loop

%reshape R
R = reshape(x(7:15,1),3,3);

%reshape R_hat
R_kf(:,:,1) = reshape(x_hat(7:15,1),3,3);

%generate true image
I(:,:,1) = imageGen(X,  x(1:3,1), R, K);

%generate estimated image
I_hat(:,:,1) = imageGen(X_hat, x_hat(1:3,1), reshape(x_hat(7:15,1),3,3), K);

%calculate reprojection error
e_reproj(1) = norm(I(:,:,1) - I_hat(:,:,1), 'fro');

%calculate position error
e_pos(1) = norm(x_hat(1:3,1) - x(1:3,1));

%calculate map error
e_map(1) = norm(mean(abs(X-X_hat),2));

%error DCM
e_R(:,:,1) = R_kf(:,:,1)'*R;

%error DCM trace
e_Rtrace = ones(1,M);

%calculate true quaternion
quat_true = dcm2quat(R,'None',.01);

%calculate estimated quaternion
quat_est = dcm2quat(eye(3));

%find error quaternion
e_quat(1,:) = quatmultiply(quatconj(quat_est),quat_true);

%calculate approximate euler angle errors
e_eul(1,:) = 2*atan(e_quat(1,2:4)/e_quat(1,1));

%propogate estimate model
x_hat(:,2) = motionModel(Mod,F(:,1),T(:,1),x_hat(:,1),ts);

%reshape R_hat
R_kf(:,:,2) = reshape(x_hat(7:15,2),3,3);

%Propogate truth model
x(:,2) = motionModel(Mod, F(:,1) + sqrt(sig_F)*randn(3,1), ...
    T(:,1) + sqrt(sig_T)*randn(3,1), x(:,1),ts);

%reshape R
R = reshape(x(7:15,2),3,3);

%generate true image
I(:,:,2) = imageGen(X, x(1:3,2), R, K);

%add key frames
I_kf(:,:,:) = I(:,:,1:2);

%add estimated poses to key frame state
x_hat_kf(:,:) = x_hat(1:3,1:2);

%update map
[X_hat, x_hat_kf, R_kf] = mapUpdate(I_kf, X_hat, x_hat_kf, R_kf, K);

%update x_hat
x_hat(1:3,2) = x_hat_kf(:,2);
x_hat(7:15,2) = reshape(R_kf(:,:,2),9,1);

%generate estimated image
I_hat(:,:,2) = imageGen(X_hat, x_hat(1:3,2), reshape(x_hat(7:15,2),3,3), K);

%calculate reprojection error
e_reproj(2) = norm(I(:,:,2) - I_hat(:,:,2), 'fro');

%calculate position error
e_pos(2) = norm(x_hat(1:3,2) - x(1:3,2));

%calculate map error
e_map(2) = norm(mean(abs(X-X_hat),2));

%error DCM
e_R(:,:,2) = R_kf(:,:,2)'*R;

%calculate true quaternion
quat_true = dcm2quat(R,'None',.01);

%calculate estimated quaternion
quat_est = dcm2quat(R_kf(:,:,end));

%find error quaternion
e_quat(2,:) = quatmultiply(quatconj(quat_est),quat_true);

%calculate approximate euler angle errors
e_eul(2,:) = 2*atan(e_quat(2,2:4)/e_quat(2,1));

%propogate estimate model
x_hat(:,3) = motionModel(Mod,F(:,2),T(:,2),x_hat(:,2),ts);

%Propogate truth model
x(:,3) = motionModel(Mod, F(:,2) + sqrt(sig_F)*randn(3,1), ...
    T(:,2) + sqrt(sig_T)*randn(3,1), x(:,2),ts);

I_pPose = I; %pre-pose update images
e_reprojPP = e_reproj; %pre pose update error

%% run simulation loop
for ii = 3:M   
    %% Truth
    %reshape R
    R = reshape(x(7:15,ii),3,3);
    
    %generate true image
    I(:,:,ii) = imageGen(X, x(1:3,ii), R, K);
    
    %propogate truth model (dont propogate on last frame)
    if ii ~= M
        x(:,ii + 1) = motionModel(Mod, F(:,ii) + sqrt(sig_F)*randn(3,1),...
            T(:,ii) + sqrt(sig_T)*randn(3,1), x(:,ii),ts);
    end
    
    %generate camera vector
    cVect(:,ii) = R'*[0 0 1]';
    
    %% Pose Update
    
    %reshape R_hat
    R_hat = reshape(x_hat(7:15,ii),3,3);
    
    %pre pose update image
    I_pPose(:,:,ii) = imageGen(X_hat, x_hat(1:3,ii), R_hat, K);
    
    %update pose
    [x_hat(1:3,ii), R_hat] = poseUpdate(I(:,:,ii), x_hat(1:3,ii), R_hat, X_hat, K);
    
    %update x_hat attitude
    x_hat(7:15,ii) = reshape(R_hat,9,1);
    
    %% Mapping
    
    %increment keyframe counting index
    kf_cnt = kf_cnt + 1;
    
    %check to see if it's time for a new keyframe
    if kf_cnt > kf_freq
        
        %reset keyframe counting
        kf_cnt = 1;
        
        %add current image to keyframe database
        I_kf(:,:, kf_idx) = I(:,:,ii);
        
        %add current estimate to keyframe estimates
        x_hat_kf(:,kf_idx) = x_hat(1:3,ii);
        
        %add current R_hat to keyframe poses
        R_kf(:,:,kf_idx) = R_hat;
        
        %perform bundle adjustment
        [X_hat, x_hat_kf, R_kf] = mapUpdate(I_kf, X_hat, x_hat_kf, R_kf, K);
        
        %update current pose estimate
        x_hat(1:3,ii) = x_hat_kf(:,end);
        x_hat(7:15,ii) = reshape(R_kf(:,:,end),9,1);
        
        %increase keyframe index
        kf_idx = kf_idx + 1;
        
        %reshape R_hat
        R_hat = reshape(x_hat(7:15,ii),3,3);
    end
        
    %% Reprojected image generation
    
    %generate estimated image
    I_hat(:,:,ii) = imageGen(X_hat, x_hat(1:3,ii), R_hat, K);
    
    %% Error Tracking
    
    %calculate reprojection error
    e_reproj(ii) = norm(I(:,:,ii) - I_hat(:,:,ii), 'fro');
    
    %calculate reprojection error
    e_reprojPP(ii) = norm(I(:,:,ii) - I_pPose(:,:,ii), 'fro');

    %calculate camera position error
    e_pos(ii) = norm(x_hat(1:3,ii) - x(1:3,ii));
    
    %calculate map error
    e_map(ii) = norm(mean(abs(X-X_hat),2));
    
    %calculate true quaternion
    quat_true = dcm2quat(R,'None',eps(5));
    
    %calculate estimated quaternion
    quat_est = dcm2quat(R_hat,'None',eps(5));
    
    %find error quaternion
    e_quat(ii,:) = quatmultiply(quatconj(quat_est),quat_true);
    
    %calculate approximate euler angle errors
    e_eul(ii,:) = 2*atan(e_quat(ii,2:4)/e_quat(ii,1));
    
    %error DCM
    e_R(:,:,ii) = R_hat'*R;
    
    %error DCM trace
    e_Rtrace(ii) = abs(3 - trace(e_R(:,:,ii)));
    
    %% Estimate 
    %propogate estimate model (dont propogate on last frame)
    if ii ~= M
        x_hat(:,ii + 1) = motionModel(Mod,F(:,ii),T(:,ii),x_hat(:,ii),ts);
    end
end

%% plot 4 representative images
figure
subplot(2,2,1)
scatter(I(1,:,1),I(2,:,1),5,'filled')
hold on
scatter(I_hat(1,:,1),I_hat(2,:,1),5,'filled')
set(gca, 'ydir','reverse')
axis([p1-640 p1+640 p2-480 p2+480])
title('Image 1')
legend('True Image','Projected Image','Location','best')

subplot(2,2,2)
scatter(I(1,:,round(M/3)),I(2,:,round(M/3)),5,'filled')
hold on
scatter(I_hat(1,:,round(M/3)),I_hat(2,:,round(M/3)),5,'filled')
set(gca, 'ydir','reverse')
axis([p1-640 p1+640 p2-480 p2+480])
title('Image 2')
legend('True Image','Projected Image','Location','best')

subplot(2,2,3)
scatter(I(1,:,round(2*M/3)),I(2,:,round(2*M/3)),5,'filled')
hold on
scatter(I_hat(1,:,round(2*M/3)),I_hat(2,:,round(2*M/3)),5,'filled')
set(gca, 'ydir','reverse')
axis([p1-640 p1+640 p2-480 p2+480])
title('Image 3')
legend('True Image','Projected Image','Location','best')

subplot(2,2,4)
scatter(I(1,:,end),I(2,:,end),5,'filled')
hold on
scatter(I_hat(1,:,end),I_hat(2,:,end),5,'filled')
set(gca, 'ydir','reverse')
axis([p1-640 p1+640 p2-480 p2+480])
title('Image 4')
legend('True Image','Projected Image','Location','best')

%% plot experiment setup
figure

%plot 3D structure
scatter3(X(1,:),X(2,:),X(3,:),5,'filled')
hold on

%plot 3D structure estimate
scatter3(X_hat(1,:),X_hat(2,:),X_hat(3,:),5,'filled')

%plot camera estimate
plot3(x_hat(1,:),x_hat(2,:),x_hat(3,:))
axis equal
quiver3(x_hat(1,:),x_hat(2,:),x_hat(3,:),cVect_hat(1,:),cVect_hat(2,:),cVect_hat(3,:),'AutoScale','off')
xlabel('x')
ylabel('y')
zlabel('z')

%plot camera truth
plot3(x(1,:),x(2,:),x(3,:))
quiver3(x(1,:),x(2,:),x(3,:),cVect(1,:),cVect(2,:),cVect(3,:),'AutoScale','off')

axis equal

%% plot error representations
figure
semilogy(e_reproj)
hold on
semilogy(e_reprojPP)
legend('Post Pose Update','Pre Pose Update','Location','best')
xlabel('Camera Frame')
ylabel('Frobinius Norm of reprojection error')

figure
semilogy(e_pos)
xlabel('Camera Frame')
ylabel('2-norm of position error')

figure
semilogy(e_map)
xlabel('Camera Frame')
ylabel('2-norm of average mapping error')

figure
semilogy(abs(e_eul(:,1)))
hold on
semilogy(abs(e_eul(:,2)))
semilogy(abs(e_eul(:,3)))
xlabel('Camera Frame')
ylabel('Euler Angle Error (rad.)')
legend('x','y','z')

figure
semilogy(e_Rtrace(3:end))





