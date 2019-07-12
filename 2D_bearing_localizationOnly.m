close all
clear
clc

%add useful library
addpath './tools/g2o_wrapper'
source "./tools/utilities/geometry_helpers_2d.m"
source "./LS_algorithm_localizationOnly.m"


%load the data from the .g2o file (robot pose and landmark position, bearing measurements)

[landmarks, poses, transitions, observations] = loadG2o('./dataset/slam-2d-bearing-only_PROJECT.g2o');

%analyze_data(landmarks, poses, transitions, observations);

%ELABORATE THE DATASET
num_landmarks = length(landmarks)
num_poses=length(poses)

# XL_true (given by the landmarks..........................) (landmarks in a matrix, one per column)
XL_true = zeros(2,num_landmarks);
XL_true_ids = zeros (1,num_landmarks);
for i = 1:num_landmarks
    XL_true(:,i) = [landmarks(i).x_pose landmarks(i).y_pose]';
    XL_true_ids(i) = landmarks(i).id;
endfor

%XR_true (given by the poses........................)
XR_true = zeros(3,3,num_poses);
for i=1:length(poses)
    XR_true(:,:,i) = v2t([poses(i).x, poses(i).y, poses(i).theta]); 
endfor

%# XR_guess (given by the odometry..........................)
XR_guess = zeros(3,3,num_poses);
XR_guess(:,:,1) = v2t([poses(1).x poses(1).y poses(1).theta]); %FIX THE FIRST VARIABLE DUE TO 6 DOF (NO PRIOR UP TO NOW)
for i=1:length(transitions)
%    transitions(i).id_from
%    transitions(i).id_to
%    transitions(i).id_from - 1299
%    transitions(i).id_to - 1299
%    if transitions(i).id_from - 1299==93
%      disp(transitions(i).v)
%    elseif transitions(i).id_from - 1299 ==94 
%      disp(transitions(i).v)
%    endif
%    disp("*****")
    
    trans = transitions(i).v;   
    u_x = trans(1);
    u_theta = trans(3); 
    
    robot_pose_prec = XR_guess(:,:,i);
    delta_pose = v2t([u_x 0 u_theta]);
    robot_pose = robot_pose_prec * delta_pose; 
    XR_guess(:,:,i+1) = robot_pose; 
endfor


%%******************************************************************************
%plot_world(XL_true, [], XR_true, XR_guess, landmarks, poses, transitions, []);
%%******************************************************************************

%########################### LS algorithm ######################################
################################################################################
################################################################################
################################################################################
################################################################################
################################################################################
################################################################################
################################################################################

%reorder the measurements like: pose-lm_id in a 2xlength(observations) matrix; build also the lm_measurements as 1xlength(observations)


for i=1:length(landmarks)
  lm_ids(end+1) = landmarks(i).id;
endfor

for i=1:length(observations)
  pose_id = observations(i).pose_id;
  observation = observations(i).observation;
  for j=1:length(observation)
    lm_id = observation(j).id;
    z = observation(j).bearing;
    landmark_associations(:,end+1) = [pose_id-1299 find(lm_ids==lm_id)]';
    Zl(1,end+1) = z;
  endfor
endfor

%observations(46).observation.id; %15  observations
%observations(93).observation;    %9   observations
%observations(99).observation;    %8   observations
%observations(47).observation;    %15  observations
%observations(90).observation;    %10  observations
%
%Omega = 0.01;
%damping=0.0001;
%kernel_threshold=10000;
%num_iterations=67;

%best configuration (up to now)
Omega = 0.01
damping=0.0001
kernel_threshold=Omega*((pi/6)^2)
num_iterations=32
printf("\nThe parameters are:\n",Omega)
printf("Omega = %f\n",Omega)
printf("damping = %f\n",damping)
printf("kernel_threshold = %f\n",kernel_threshold)
printf("num_iterations = %f\n\n",num_iterations)

[XR, XL,chi_stats_l, num_inliers_l, H, b]=LSLAM_bearingOnly(XR_guess, XL_true, 
												      Zl, landmark_associations,
												      num_poses, 
												      num_landmarks,
												      num_iterations, 
												      damping, 
												      kernel_threshold,Omega);


printf("The minimum error is given by the iteration %d\n",find(chi_stats_l==min(chi_stats_l)))

h=figure(1);
for i=1:length(poses)
    robot_pose_true = t2v(XR_true(:,:,i));
    drawRobot(robot_pose_true, poses(i).id,zeros(3,3),'b');
    
    robot_pose_guess = t2v(XR_guess(:,:,i));
    drawRobot(robot_pose_guess, poses(i).id,zeros(3,3),'y');
    robot_pose = t2v(XR(:,:,i));
    drawRobot(robot_pose, poses(i).id,zeros(3,3),'r');
endfor


h=figure(2);
subplot(2,1,1);
title("Poses Initial Guess");
plot(XR_true(1,3,:),XR_true(2,3,:),'b*',"linewidth",2);
hold on;
plot(XR_guess(1,3,:),XR_guess(2,3,:),'ro',"linewidth",2);
legend("Poses True", "Guess",'Location','SouthEast');grid;

subplot(2,1,2);
title("Poses After Optimization");
plot(XR_true(1,3,:),XR_true(2,3,:),'b*',"linewidth",2);
hold on;
plot(XR(1,3,:),XR(2,3,:),'ro',"linewidth",2);
legend("Poses True", "Guess",'Location','SouthEast'); grid;
size(XR)

h=figure(3);
hold on;
grid;
title("chi evolution");
%
subplot(3,1,1);
plot(chi_stats_l, 'r-', "linewidth", 2);
legend("Chi Landmark",'Location','NorthEast'); grid; xlabel("iterations");
subplot(3,1,2);
plot(num_inliers_l, 'b-', "linewidth", 2);
legend("#inliers",'Location','SouthEast'); grid; xlabel("iterations");
%waitfor(h)

h = figure(4);
title("H matrix");
size(H)
H_ =  H./H;                      # NaN and 1 element
H_(isnan(H_))=0;                 # Nan to Zero
H_ = abs(ones(size(H_)) - H_);   # switch zero and one
H_ = flipud(H_);                 # switch rows
colormap(gray(64));
hold on;
image([0.5, size(H_,2)-0.5], [0.5, size(H_,1)-0.5], H_*64);
hold off;
waitfor(h)






