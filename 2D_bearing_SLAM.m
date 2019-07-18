close all
clear
clc

%add useful library
addpath './tools/g2o_wrapper'
source "./tools/geometry_helpers_2d.m"
source "./LS_algorithm.m"
source "./tools/utilities.m"


%load the data from the .g2o file (robot pose and landmark position, bearing measurements and transitions)
[landmarks, poses, transitions, observations] = loadG2o('./dataset/slam-2d-bearing-only_PROJECT.g2o');

%analyze_data(landmarks, poses, transitions, observations); %just to see which dataset i have

%ELABORATE THE DATASET
num_landmarks = length(landmarks);
num_poses=length(poses);

% id mapping landmarks and poses (the id of the array is the new id)
for i=1:length(poses)
  poses_ids(end+1) = poses(i).id;
endfor

for i=1:length(landmarks)
  lm_ids(end+1) = landmarks(i).id;
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
    
    trans = transitions(i).v;   
    u_x = trans(1);
    u_theta = trans(3); 
    
    robot_pose_prec = XR_guess(:,:,i);
    delta_pose = v2t([u_x 0 u_theta]);
    robot_pose = robot_pose_prec * delta_pose; 
    XR_guess(:,:,i+1) = robot_pose; 
endfor

# XL_true (given by the landmarks..........................) (landmarks in a matrix, one per column)
XL_true = zeros(2,num_landmarks);
XL_true_ids = zeros (1,num_landmarks);
for i = 1:num_landmarks
    XL_true(:,i) = [landmarks(i).x_pose landmarks(i).y_pose]';
    XL_true_ids(i) = landmarks(i).id;
endfor

# XL_guess (given by the triangulation: a landmark can be triangulated using consecutive poses or lines intertersection
           #---consecutive poses: a landmark is triangulated iff
               # - 2 consecutive poses have seen the same landmark && z_ob1!=0 && u_x!=0 && |z_obs2-z_obs1| > threshold
               # otherwise the triangulation is not possible
           #---lines intersection: a landmark is triangulated by considering the closest point to the lines generated by the observation of the poses,
                                   #you can see the readme for more details)
           
#triangulation
[XL_guess_id, XL_guess] = triangulation(poses, transitions, observations);
%[XL_guess_id, XL_guess] = triangulation_lineIntersection(XR_guess,observations,poses_ids,lm_ids);

%which are the landmarks not triangulated?
num_landmarks_triangulated = size(XL_guess_id,2);
miss_lm = [];
printf("\nTriangulated landmarks: %d/%d, miss %d\n",num_landmarks_triangulated,num_landmarks, num_landmarks - num_landmarks_triangulated)
printf("The missing landmarks are\n")
for i=1:num_landmarks
  if length(find(XL_guess_id==landmarks(i).id))==0
     miss_lm(end+1) = landmarks(i).id;
     printf("%d, ",landmarks(i).id)
  endif
endfor
printf("\n")

%reorder the measurements like: pose-lm_id in a 2xlength(observations) matrix; build also the lm_measurements as 1xlength(observations).
                               #The very same for pose-pose measurements

%pose-landmark measurements
for i=1:length(observations)
  pose_id = observations(i).pose_id;
  observation = observations(i).observation;
  for j=1:length(observation)
    lm_id = observation(j).id;
    if length(find(XL_guess_id==lm_id))==0
      continue
    endif
    z = observation(j).bearing;
    landmark_associations(:,end+1) = [find(poses_ids==pose_id) find(XL_guess_id==lm_id)]';
    Zl(1,end+1) = z;
  endfor
endfor

%pose-pose measurements
num_pose_measurements=num_poses-1;
Zr=zeros(3,3,num_pose_measurements);
pose_associations=zeros(2,num_pose_measurements);

measurement_num=1;
for i=1:length(transitions)    
    trans = transitions(i).v;   
    u_x = trans(1);
    u_theta = trans(3); 
    pose_associations(:,measurement_num)=[i, i+1]';
    Zr(:,:,measurement_num)=v2t([u_x 0 u_theta]);
    measurement_num++;
endfor



%%******************************************************************************
%plot_world(XL_true, XL_guess, XR_true, XR_guess, landmarks, poses, transitions, XL_guess_id, miss_lm);
%%******************************************************************************

%########################### LS algorithm ######################################
################################################################################
################################################################################
################################################################################
################################################################################
################################################################################
################################################################################
################################################################################

%best parameters found experimentally

%triangulation with consecutive poses
%POSE-LM 
%Omega = 0.005;
%damping=0.0001;
%kernel_threshold=100;
%num_iterations=40;

%TLS
Omega = 4;
damping=0.01;
kernel_threshold=1000;
num_iterations=40;

%triangulation with line intersection
%POSE-LM 
%Omega = 0.005;
%damping=0.0001;
%kernel_threshold=100;
%num_iterations=40;

%TLS 
%Omega = 10;
%damping=0.01;
%kernel_threshold=1000;
%num_iterations=1;

printf("\nThe parameters are:\n",Omega)
printf("Omega = %f\n",Omega)
printf("damping = %f\n",damping)
printf("kernel_threshold = %f\n",kernel_threshold)
printf("num_iterations = %f\n\n",num_iterations)

[XR, XL,chi_stats_l_omega, num_inliers_l, chi_stats_l, H, b]=LSLAM_bearingOnly(XR_guess, XL_guess, 
												      Zl, landmark_associations, Zr, pose_associations,
												      num_poses, 
												      num_landmarks_triangulated,
												      num_iterations, 
												      damping, 
												      kernel_threshold,Omega);


printf("The minimum error is given by the iteration %d\n",find(chi_stats_l==min(chi_stats_l)))
close all
h=figure(1);#########################################################################################################################
for i=1:length(poses)
    robot_pose_true = t2v(XR_true(:,:,i));
    drawRobot(robot_pose_true, poses(i).id,zeros(3,3),'b');
    
    robot_pose_guess = t2v(XR_guess(:,:,i));
    drawRobot(robot_pose_guess, poses(i).id,zeros(3,3),'y');
    robot_pose = t2v(XR(:,:,i));
    drawRobot(robot_pose, poses(i).id,zeros(3,3),'r');
endfor

h=figure(2);#########################################################################################################################
drawLandmarks(landmarks);
for i=1:length(XL_guess_id)
  lm_guess_struct = landmark(XL_guess_id(i), XL_guess(:,i)'); %create a landmark struct
  lm_guess(end+1) = lm_guess_struct;    
endfor
drawLandmarks(lm_guess,'magenta','fill');
drawLandmarks(landmarks,'r','fill',miss_lm,'k');

lm_guess = [];
for i=1:length(XL_guess_id)
  lm_guess_struct = landmark(XL_guess_id(i), XL(:,i)'); %create a landmark struct
  lm(end+1) = lm_guess_struct;    
endfor
drawLandmarks(lm,'c','fill');

h = figure(3);#########################################################################################################################
s=subplot(2,2,1);
title("Landmark Initial Guess");
hold on;
plot(XL_true(1,:),XL_true(2,:),'b*',"linewidth",2)
miss_XL = [];
for i=1:length(miss_lm)
    miss_XL(:,end+1) = XL_true(:,find(lm_ids==miss_lm(i)));
endfor
if size(miss_XL,2)>0
  plot(miss_XL(1,:),miss_XL(2,:),'k*',"linewidth",2);
  plot(XL_guess(1,:),XL_guess(2,:),'ro',"linewidth",2);
endif
legend('lm\_true','lm\_not\_triang','lm\_guess',s,"location", "northeast");grid;
ylim([-5 20])

s=subplot(2,2,2);
title("Landmark After Optimization");
hold on;
plot(XL_true(1,:),XL_true(2,:),'b*',"linewidth",2);
plot(miss_XL(1,:),miss_XL(2,:),'k*',"linewidth",2);
plot(XL(1,:),XL(2,:),'ro',"linewidth",2);
legend('lm\_true','lm\_not\_triang','lm\_guess',s,"location", "northeast");grid;
ylim([-5 20])

s=subplot(2,2,3);
title("Poses Initial Guess");
hold on;
plot(XR_true(1,3,:),XR_true(2,3,:),'b*',"linewidth",2);
hold on;
plot(XR_guess(1,3,:),XR_guess(2,3,:),'ro',"linewidth",2);
legend("Poses True", "Guess",'Location','SouthEast',s,"location", "northeast");grid;

s=subplot(2,2,4);
title("Poses After Optimization");
hold on;
plot(XR_true(1,3,:),XR_true(2,3,:),'b*',"linewidth",2);
hold on;
plot(XR(1,3,:),XR(2,3,:),'ro',"linewidth",2);
legend("Poses True", "Guess",'Location','SouthEast',s,"location", "northeast"); grid;

h=figure(4);#########################################################################################################################
subplot(3,1,1);
title("chi evolution");
hold on;
plot(chi_stats_l_omega, 'r-', "linewidth", 2);
legend("Chi Landmark",'Location','NorthEast'); grid; xlabel("iterations");
subplot(3,1,2);
title("inliers evolution");
hold on;
plot(num_inliers_l, 'b-', "linewidth", 2);
legend("#inliers",'Location','SouthEast'); grid; xlabel("iterations");grid;

h=figure(5);#########################################################################################################################
plot(chi_stats_l, 'r-', "linewidth", 2);
legend("Chi Landmark",'Location','NorthEast'); grid; xlabel("iterations");

h = figure(6);#########################################################################################################################
title("H matrix");
hold on;
H_ =  H./H;                      # NaN and 1 element
H_(isnan(H_))=0;                 # Nan to Zero
H_ = abs(ones(size(H_)) - H_);   # switch zero and one
H_ = flipud(H_);                 # switch rows
colormap(gray(64));
hold on;
image([0.5, size(H_,2)-0.5], [0.5, size(H_,1)-0.5], H_*64);
hold off;

h=figure(7);#########################################################################################################################
drawLandmarks(lm,'c','fill');
for i=1:length(poses)
    robot_pose = t2v(XR(:,:,i));
    drawRobot(robot_pose, poses(i).id,zeros(3,3),'r');
endfor

waitfor(h)






