function [triangulated_lm_id, triangulated_lm] = triangulation(poses, transitions, observations)  #consecutive observations needs to have some lm seen
  triangulated_lm_id = []; % the lm id of the triangulated landmark
  triangulated_lm = [];
  robot_pose = v2t([poses(2).x poses(2).y poses(2).theta]);
  
  for i=1:length(observations)-1
     ob1 = observations(i).observation;
     ob2 = observations(i+1).observation;
     trans = transitions(i+1).v;
     u_x = trans(1);
     u_theta = trans(3);
     
     
     %find the landmark to triangulate
     ids_ob1 = zeros(1,length(ob1));
     ids_ob2 = zeros(1,length(ob2));
     for j=1:length(ob1)
      ids_ob1(j) = ob1(j).id;
     endfor
     for j=1:length(ob2)
      ids_ob2(j) = ob2(j).id;
     endfor
     
     adiacent_ids=intersect(ids_ob1,ids_ob2);
     lm_to_triangulate = setdiff(adiacent_ids,triangulated_lm_id);
     
     delta_pose = v2t([u_x 0 u_theta]);
     robot_pose = robot_pose * delta_pose; %position of the robot w.r.t the word in the second observation (l_2_2)
     
     %compute the landmarks to triangulate
     for j=1:length(lm_to_triangulate)
        lm_id = lm_to_triangulate(j);
        lm = ones(3,1); %HOMOGENEOUS COORDINATES
        %find measurements
        z_ob1_index = find(ids_ob1==lm_id);
        z_ob1 = ob1(z_ob1_index).bearing;
        if abs(z_ob1)<0.0001 || abs(u_x)<0.2 %if the first orientation or u_x is 0 discard the candidate landmark since a triangulation is not possible due to
                                             %the chosen formula
          %printf("discarded landmark %d for these coupled poses\n",lm_id)
          continue
        endif
        z_ob2_index = find(ids_ob2==lm_id);
        z_ob2 = ob2(z_ob2_index).bearing;
        if abs(z_ob2-z_ob1)<0.03 % the observation of the landmark is almost on the same line: the triangulation is not possible
          continue  
        endif
        a11 = delta_pose(1,1);
        a12 = delta_pose(1,2);
        a21 = delta_pose(2,1);
        a22 = delta_pose(2,2);
        u_x = delta_pose(1,3);

        A = a11*cos(z_ob2) + a12*sin(z_ob2);
        B = a21*cos(z_ob2) + a22*sin(z_ob2);

        k = -u_x*sin(z_ob1) / (sin(z_ob1)*A - cos(z_ob1)*B);
        l_2_2 = [k*cos(z_ob2);k*sin(z_ob2)];

        
        %update triangulated_lm (shift the lm in the word frame)
        lm_world = robot_pose * [l_2_2;1];
        triangulated_lm(:,end+1) = lm_world(1:2,1);
        triangulated_lm_id(end+1) = lm_id;
     endfor
  endfor
endfunction


function [triangulated_lm_id, triangulated_lm] = triangulation_lineIntersection(X_guess,observations,poses_ids,lm_ids)  #consecutive observations needs to have some lm seen
  %!!!!!!!!!!!!!!!!!!!!NEED A CHECK FOR ROBOT FOV (a contrained LS should be needed to find the landmarks in the robot seen)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  %dk: direction of the line
  %ak: robot_pose (point from which the line starts (half-line...))
  for i=1:length(lm_ids)
    d = [];
    a = [];
    robot_pos_lm = [];
    for j=1:length(observations)
      pose_id = observations(j).pose_id;
      observation = observations(j).observation;
      for k=1:length(observation)
          lm_id = observation(k).id;
          if lm_ids(i) == lm_id
            %get d and a
            robot_pose  = X_guess(:,:,find(poses_ids==pose_id));
            z = observation(k).bearing;
            R_z1 = [cos(z) -sin(z);sin(z) cos(z)];
            dk = R_z1*robot_pose(1:2,1);              
            ak = robot_pose(1:2,3);
            d(:,end+1) = dk;
            a(:,end+1) = ak;
            robot_pos_lm(:,end+1) = ak;
            break
          endif
      endfor
    endfor
    %check on the parallelism
    d_filtered = [];
    a_filtered = [];
    
    for j=1:size(d,2)-1
      d_curr = d(:,j);
      parallel = 0;
      for k=j+1:size(d,2)
          d_next = d(:,k);
          if abs(d_curr'*d_next)>0.9999 %parallel line, discard triangulation
              parallel=1;
              break
          endif
       endfor
      if parallel==0
        d_filtered(:,end+1) = d_curr;
        a_filtered(:,end+1) = a(:,j);
      endif
    endfor        
    d_filtered(:,end+1) = d(:,size(d,2));
    a_filtered(:,end+1) = a(:,size(d,2));
    
    if size(d_filtered,2)<2
      continue;
    endif
    %do triangulation with d and a: least square approach to find the point at minimum distance from each line (closed form since it's uncostrained)   
    A = zeros(2,2);
    b = zeros(2,1);
    for j=1:size(d_filtered,2)
        A+= eye(2,2)-d(:,j)*d(:,j)';
        b+= (eye(2,2)-d(:,j)*d(:,j)')*a(:,j);
    endfor
    lm_world = A\b;
    triangulated_lm(:,end+1) = A\b;
    triangulated_lm_id(end+1) = lm_ids(i);
  endfor  
endfunction

function a=analyze_data(landmarks, poses, transitions, observations)

  for i=2:length(poses)
  
    x = poses(i).x-poses(i-1).x;
    y = poses(i).y-poses(i-1).y;
    theta = poses(i).theta-poses(i-1).theta;
    c=cos(poses(i-1).theta);
    s=sin(poses(i-1).theta);
    R= [c  -s;
        s  c];
    trans = transitions(i-1).v;
    t = R*trans(1:2);
    printf("%f %f *** %f %f *** %f %f\n",x,t(1),y,t(2),theta,trans(3)); %the y transition is basically 0 (up to some error) due to
                                                                                %unicycle model(motion only along the x direction)
  endfor
endfunction

function v=flattenIsometryByColumns(T)
v=zeros(6,1);
v(1:4)=reshape(T(1:2,1:2),4,1);
v(5:6)=T(1:2,3);
endfunction
















