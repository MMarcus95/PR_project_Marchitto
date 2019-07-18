addpath './tools/visualization'
source "./tools/geometry_helpers_2d.m"
source "./least_squares_indices.m"

function [XR, XL]=boxPlus(XR, XL, num_poses, num_landmarks, dx)
  global pose_dim;
  global landmark_dim;
  for(pose_index=1:num_poses)
    pose_matrix_index=poseMatrixIndex(pose_index, num_poses, num_landmarks);
    dxr=dx(pose_matrix_index:pose_matrix_index+pose_dim-1);
    XR(:,:,pose_index)=v2t(dxr)*XR(:,:,pose_index);
  endfor;
  for(landmark_index=1:num_landmarks)
    landmark_matrix_index=landmarkMatrixIndex(landmark_index, num_poses, num_landmarks);
    dxl=dx(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,:);
    XL(:,landmark_index)+=dxl;
  endfor;
endfunction;


function [XR, XL, chi_stats_l_omega, num_inliers_l, chi_stats_l, H, b] = LSLAM_bearingOnly(XR, XL,
	     Zl, landmark_associations, Zr, pose_associations,
	     num_poses,
	     num_landmarks,
	     num_iterations,
	     damping,
	     kernel_threshold,omega)
  
  global pose_dim;
  global landmark_dim;

  chi_stats_l_omega=zeros(1,num_iterations);
  num_inliers_l=zeros(1,num_iterations);
  
  
  chi_stats_r=zeros(1,num_iterations);
  num_inliers_r=zeros(1,num_iterations);
  
  # size of the linear system
  system_size=pose_dim*num_poses+landmark_dim*num_landmarks; 
  for (iteration=1:num_iterations)
    if mod(iteration,10)==0
      printf("iteration %d\n",iteration)  
    endif
    H=zeros(system_size, system_size);
    b=zeros(system_size,1);
   
    [H_landmarks, b_landmarks, chi_omega, num_inliers_, chi_] = linearizeLandmarks(XR, XL, Zl, landmark_associations,num_poses, num_landmarks, kernel_threshold,omega);
    chi_stats_l_omega(iteration)=chi_omega;
    num_inliers_l(iteration)=num_inliers_;
    chi_stats_l(iteration)=chi_;
    
    [H_poses, b_poses, chi_, num_inliers_] = linearizePoses(XR, XL, Zr, pose_associations, num_poses, num_landmarks, kernel_threshold);
    chi_stats_r(iteration)+=chi_;
    num_inliers_r(iteration)=num_inliers_;
    
    H=H_poses;
    b=b_poses;
    if (num_landmarks) 
       H+=H_landmarks;
       b+=b_landmarks;
    endif;
    
    H+=eye(system_size)*damping;
    dx=zeros(system_size,1);

    dx(pose_dim+1:end)=-(H(pose_dim+1:end,pose_dim+1:end)\b(pose_dim+1:end,1));
    [XR, XL]=boxPlus(XR,XL,num_poses, num_landmarks, dx);
  endfor
endfunction

function [e,Jr,Jl]=landmarkErrorAndJacobian(Xr,Xl,z)
  # inverse transform
  iR=Xr(1:2,1:2)';
  it=-iR*Xr(1:2,3);

  %where I should see that landmark
  bearing_measurement = iR*Xl+it;
  z_hat = atan2(bearing_measurement(2), bearing_measurement(1));
  
  %compute its Jacobian (jacobian wrt deltaX!!)
  e=z_hat-z;
  Jr = zeros(1,3);
  Jl = zeros(1,2);
  
  x_hat = bearing_measurement(1); y_hat = bearing_measurement(2);
  J_atan = 1/(x_hat^2 + y_hat^2)*[-y_hat x_hat];
  Jr = J_atan*[-iR iR*[Xl(2) -Xl(1)]'];
  Jl = J_atan*iR;
endfunction;


function [H,b, chi_tot_omega, num_inliers, chi_tot]=linearizeLandmarks(XR, XL, Zl, associations,num_poses, num_landmarks, kernel_threshold,Omega)
  global pose_dim;
  global landmark_dim;
  system_size=pose_dim*num_poses+landmark_dim*num_landmarks; 
  H=zeros(system_size, system_size);
  b=zeros(system_size,1);
  chi_tot_omega=0;
  num_inliers=0;
  chi_tot = 0;
  for (measurement_num=1:size(Zl,2))
    pose_index=associations(1,measurement_num);
    landmark_index=associations(2,measurement_num);
    z=Zl(:,measurement_num);
    Xr=XR(:,:,pose_index);
    Xl=XL(:,landmark_index);
    [e,Jr,Jl] = landmarkErrorAndJacobian(Xr, Xl, z);
    
    #OMEGA_X (need to be validated)
    %Je = -eye(1,2);
    %Omega_x = Je*inv(Omega_x)*Je';
    chi_omega=e'*Omega*e;
    if (chi_omega>kernel_threshold)
      if pose_index==48
        printf("________48_________kernel, %f\n", chi_omega)
      elseif pose_index==94
        printf("________94_________kernel, %f\n", chi_omega)
      else
        printf("%d__kernel, %f\n", pose_index, chi_omega)
      endif
      e*=sqrt(kernel_threshold/chi_omega);
      chi_omega=kernel_threshold;
    else
      num_inliers++;
    endif;
    chi_tot_omega+=chi_omega;
    
    chi_ = e'*e;    
    chi_tot+=chi_;
    
    pose_matrix_index=poseMatrixIndex(pose_index, num_poses, num_landmarks);
    landmark_matrix_index=landmarkMatrixIndex(landmark_index, num_poses, num_landmarks);

    H(pose_matrix_index:pose_matrix_index+pose_dim-1,
      pose_matrix_index:pose_matrix_index+pose_dim-1)+=Jr'*Omega*Jr;

    H(pose_matrix_index:pose_matrix_index+pose_dim-1,
      landmark_matrix_index:landmark_matrix_index+landmark_dim-1)+=Jr'*Omega*Jl;

    H(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,
      landmark_matrix_index:landmark_matrix_index+landmark_dim-1)+=Jl'*Omega*Jl;

    H(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,
      pose_matrix_index:pose_matrix_index+pose_dim-1)+=Jl'*Omega*Jr;

    b(pose_matrix_index:pose_matrix_index+pose_dim-1)+=Jr'*Omega*e;
    
    b(landmark_matrix_index:landmark_matrix_index+landmark_dim-1)+=Jl'*Omega*e;
  endfor
endfunction

function [e,Ji,Jj]=poseErrorAndJacobian(Xi,Xj,Z)
  Ri=Xi(1:2,1:2);
  Rj=Xj(1:2,1:2);
  ti=Xi(1:2,3);
  tj=Xj(1:2,3);
  tij=tj-ti;
  Ri_transpose=Ri';
  Ji=zeros(6,3);
  Jj=zeros(6,3);
  
  dR_0 = [0 -1;1 0];
  dR = Ri_transpose*dR_0*Rj;
  
  Jj(1:4,3)=reshape(dR, 4, 1);
  Jj(5:6,1:2)=Ri_transpose;
  
  Jj(5:6,3)=Ri_transpose*dR_0*tj;
  Ji=-Jj;

  Z_hat=eye(3); %homogeneous transformation!
  Z_hat(1:2,1:2)=Ri_transpose*Rj; 
  Z_hat(1:2,3)=Ri_transpose*tij;
  e=flattenIsometryByColumns(Z_hat-Z); %flatten the chordal distance (matrix difference)
                                       %put in one column the rotation vectors and translation one
 endfunction;

function [H,b, chi_tot, num_inliers]=linearizePoses(XR, XL, Zr, associations,num_poses, num_landmarks, kernel_threshold)
  global pose_dim;
  global landmark_dim;
  system_size=pose_dim*num_poses+landmark_dim*num_landmarks; 
  H=zeros(system_size, system_size);
  b=zeros(system_size,1);
  chi_tot=0;
  num_inliers=0;
  for (measurement_num=1:size(Zr,3))
    
    gain  = 1;
    Omega=eye(6)*gain;
    Omega(1:4,1:4)*=gain;
    pose_i_index=associations(1,measurement_num);
    pose_j_index=associations(2,measurement_num);
    Z=Zr(:,:,measurement_num);
    Xi=XR(:,:,pose_i_index);
    Xj=XR(:,:,pose_j_index);
    [e,Ji,Jj] = poseErrorAndJacobian(Xi, Xj, Z);
    chi=e'*Omega*e;
    if (chi>kernel_threshold)
      Omega*=sqrt(kernel_threshold/chi);
      chi=kernel_threshold;
    else
      num_inliers ++;
    endif;
    chi_tot+=chi;
    
    pose_i_matrix_index=poseMatrixIndex(pose_i_index, num_poses, num_landmarks);
    pose_j_matrix_index=poseMatrixIndex(pose_j_index, num_poses, num_landmarks);
    
    H(pose_i_matrix_index:pose_i_matrix_index+pose_dim-1,
      pose_i_matrix_index:pose_i_matrix_index+pose_dim-1)+=Ji'*Omega*Ji;

    H(pose_i_matrix_index:pose_i_matrix_index+pose_dim-1,
      pose_j_matrix_index:pose_j_matrix_index+pose_dim-1)+=Ji'*Omega*Jj;

    H(pose_j_matrix_index:pose_j_matrix_index+pose_dim-1,
      pose_i_matrix_index:pose_i_matrix_index+pose_dim-1)+=Jj'*Omega*Ji;

    H(pose_j_matrix_index:pose_j_matrix_index+pose_dim-1,
      pose_j_matrix_index:pose_j_matrix_index+pose_dim-1)+=Jj'*Omega*Jj;

    b(pose_i_matrix_index:pose_i_matrix_index+pose_dim-1)+=Ji'*Omega*e;
    b(pose_j_matrix_index:pose_j_matrix_index+pose_dim-1)+=Jj'*Omega*e;
  endfor
endfunction



