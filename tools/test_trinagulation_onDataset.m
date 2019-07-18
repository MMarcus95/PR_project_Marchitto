close all
clear
clc

%add useful library
addpath '../'
addpath './g2o_wrapper'
addpath './visualization'
source "./geometry_helpers_2d.m"
source "../least_squares_indices.m"
source "./utilities.m"

[landmarks, poses, transitions, observations] = loadG2o('../dataset/slam-2d-bearing-only_PROJECT.g2o');
num_landmarks = length(landmarks)
num_poses=length(poses)
for i=1:length(poses)
  poses_ids(end+1) = poses(i).id;
endfor
for i=1:length(landmarks)
  lm_ids(end+1) = landmarks(i).id;
endfor
XR_guess = zeros(3,3,num_poses);
XR_guess(:,:,1) = v2t([poses(1).x poses(1).y poses(1).theta]);
for i=1:length(transitions)
    
    trans = transitions(i).v;   
    u_x = trans(1);
    u_theta = trans(3); 
    
    robot_pose_prec = XR_guess(:,:,i);
    delta_pose = v2t([u_x 0 u_theta]);
    robot_pose = robot_pose_prec * delta_pose; 
    XR_guess(:,:,i+1) = robot_pose; 
endfor

%TEST 
range = 100; %select the observations you want to consider

[triangulated_lm_id, triangulated_lm] = triangulation(poses(1:range+1), transitions(1:range), observations(1:range));
%[triangulated_lm_id, triangulated_lm] = triangulation_lineIntersection(XR_guess,observations,poses_ids,lm_ids);

triangulated_lm_id;
triangulated_lm;

for i=1:length(triangulated_lm_id)
  lm_true_struct = searchById(landmarks, triangulated_lm_id(i));
  lm_true(end+1) = lm_true_struct;
  lm_guess_struct = landmark(triangulated_lm_id(i), triangulated_lm(:,i)');
  lm_guess(end+1)= lm_guess_struct;
endfor

ids_lm_true = [];

for i=1:length(landmarks)
    ids_lm_true(end+1) = landmarks(i).id;
endfor

h=figure(1);
drawLandmarks(lm_guess,'y','fill');
drawLandmarks(lm_true);
drawLandmarks(lm_guess,'g','fill');
waitfor(h)