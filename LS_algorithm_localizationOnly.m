
addpath './tools/visualization'
source "./tools/utilities/geometry_helpers_2d.m"
source "./least_squares_indices.m"


function [XR, XL]=boxPlus(XR, XL, num_poses, num_landmarks, dx)
  global pose_dim;
  global landmark_dim;
  for(pose_index=1:num_poses)
    pose_matrix_index=poseMatrixIndex(pose_index, num_poses, num_landmarks);
    dxr=dx(pose_matrix_index:pose_matrix_index+pose_dim-1);
    XR(:,:,pose_index)=v2t(dxr)*XR(:,:,pose_index);
  endfor;
%  for(landmark_index=1:num_landmarks)
%    landmark_matrix_index=landmarkMatrixIndex(landmark_index, num_poses, num_landmarks);
%    dxl=dx(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,:);
%    XL(:,landmark_index)+=dxl;
%  endfor;
endfunction;


function [XR, XL, chi_stats_l, num_inliers_l, H, b] = LSLAM_bearingOnly(XR, XL,
	     Zl, landmark_associations,
	     num_poses,
	     num_landmarks,
	     num_iterations,
	     damping,
	     kernel_threshold,omega)
  
  global pose_dim;
  global landmark_dim;

  chi_stats_l=zeros(1,num_iterations);
  num_inliers_l=zeros(1,num_iterations);

  chi_stats_r=zeros(1,num_iterations);
  num_inliers_r=zeros(1,num_iterations);
  
  # size of the linear system
  system_size=pose_dim*num_poses;%+landmark_dim*num_landmarks; 
  for (iteration=1:num_iterations)
    if mod(iteration,10)==0
      printf("iteration %d\n",iteration)
    endif
    H=zeros(system_size, system_size);
    b=zeros(system_size,1);
   
    [H_landmarks, b_landmarks, chi_, num_inliers_] = linearizeLandmarks(XR, XL, Zl, landmark_associations,num_poses, num_landmarks, kernel_threshold,omega);
    chi_stats_l(iteration)=chi_;
    num_inliers_l(iteration)=num_inliers_;
%    
    
    H = H_landmarks;
    b = b_landmarks;
    
    H+=eye(system_size)*damping;
    
%    ATTEMPT TO REGULATE THE 47 AND 94 POSE BY GIVE MORE DAMPING7
%    pose_matrix_index=poseMatrixIndex(47, num_poses, num_landmarks);
%
%    H(pose_matrix_index:pose_matrix_index+pose_dim-1,
%      pose_matrix_index:pose_matrix_index+pose_dim-1)+=eye(3,3)*(3);
%    %      
%    pose_matrix_index=poseMatrixIndex(94, num_poses, num_landmarks);
%
%    H(pose_matrix_index:pose_matrix_index+pose_dim-1,
%      pose_matrix_index:pose_matrix_index+pose_dim-1)+=eye(3,3)*(10);  
     
    dx=zeros(system_size,1);

    dx(pose_dim+1:end)=-(H(pose_dim+1:end,pose_dim+1:end)\b(pose_dim+1:end,1));
    [XR, XL]=boxPlus(XR,XL,num_poses, num_landmarks, dx);
    
%    ANALIZE H AND b W.R.T. THE POSE 47
%    pose_matrix_index=poseMatrixIndex(47, num_poses, num_landmarks); 
%    a=H(pose_matrix_index:pose_matrix_index+pose_dim-1,
%      pose_matrix_index:pose_matrix_index+pose_dim-1);
%    inv(a)
%    b(pose_matrix_index:pose_matrix_index+pose_dim-1)
%    dx(pose_matrix_index:pose_matrix_index+pose_dim-1)
%    disp("")
%    XR(:,:,47)
%    disp("**********************")

  endfor
endfunction

function [e,Jr,Jl]=landmarkErrorAndJacobian(Xr,Xl,z)
  # inverse transform
  iR=Xr(1:2,1:2)';
  it=-iR*Xr(1:2,3);

  %where I should see that landmark
  bearing_measurement = iR*Xl+it;
  z_hat = atan2(bearing_measurement(2), bearing_measurement(1));
  
  %compute its Jacobian
  e=z_hat-z;
  Jr = zeros(1,3);
  
  x_hat = bearing_measurement(1); y_hat = bearing_measurement(2);
  delta_t = Xl-Xr(1:2,3);
  theta = t2v(Xr)(3);
  c=cos(theta);
  s=sin(theta);
  Rtp=[-s,c;-c,-s];
  Jr = 1/(x_hat^2 + y_hat^2)*[-y_hat x_hat]*[-iR Rtp*delta_t];
endfunction;


function [H,b, chi_tot, num_inliers]=linearizeLandmarks(XR, XL, Zl, associations,num_poses, num_landmarks, kernel_threshold,Omega)
  global pose_dim;
  global landmark_dim;
  system_size=pose_dim*num_poses;%+landmark_dim*num_landmarks; 
  H=zeros(system_size, system_size);
  b=zeros(system_size,1);
  chi_tot=0;
  num_inliers=0;
  for (measurement_num=1:size(Zl,2))
    pose_index=associations(1,measurement_num);
    landmark_index=associations(2,measurement_num);
    z=Zl(:,measurement_num);
    Xr=XR(:,:,pose_index);
    Xl=XL(:,landmark_index);
    [e,Jr] = landmarkErrorAndJacobian(Xr, Xl, z);
    
    #OMEGA_X
    %Je = -eye(1,2);
    %Omega_x = inv(Je*inv(Omega)*Je');
    chi=e'*Omega*e;
    if (chi>kernel_threshold)
      e*=sqrt(kernel_threshold/chi);
      chi=kernel_threshold;
    else
      num_inliers++;
    endif;
    chi_tot+=chi;

    pose_matrix_index=poseMatrixIndex(pose_index, num_poses, num_landmarks);
    landmark_matrix_index=landmarkMatrixIndex(landmark_index, num_poses, num_landmarks);

    H(pose_matrix_index:pose_matrix_index+pose_dim-1,
      pose_matrix_index:pose_matrix_index+pose_dim-1)+=Jr'*Omega*Jr;

    b(pose_matrix_index:pose_matrix_index+pose_dim-1)+=Jr'*Omega*e;
  endfor
endfunction