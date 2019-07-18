function p = plot_world(XL_true, XL_guess, XR_true, XR_guess, landmarks, poses, transitions, XL_guess_id, miss_lm)

  h=figure(1);
  hold on;
  title("True configuration");
  
  drawLandmarks(landmarks);
  xlim([-16 16])
  ylim([-5 16])
  
  for i=1:length(poses)
    robot_pose_true = t2v(XR_true(:,:,i));
    drawRobot(robot_pose_true, poses(i).id,zeros(3,3),'b');
  endfor
  xlim([-16 16])
  ylim([-5 16])
  
  h = figure(2)
  title("Guess initial configuration");

  for i=1:length(XL_guess_id)
    lm_guess_struct = landmark(XL_guess_id(i), XL_guess(:,i)'); %create a landmark struct
    lm_guess(end+1) = lm_guess_struct;    
  endfor
  drawLandmarks(lm_guess,'magenta','fill');
  
  for i=1:length(transitions)
      robot_pose_guess = t2v(XR_guess(:,:,i));
      drawRobot(robot_pose_guess, poses(i).id,zeros(3,3),'y');
  endfor      
  xlim([-16 16])
  ylim([-5 16])  
  
  
  h=figure(3);
  hold on;
  title("True-Guess pose");
  
  for i=1:length(poses)
    robot_pose_true = t2v(XR_true(:,:,i));
    drawRobot(robot_pose_true, poses(i).id,zeros(3,3),'b');
  endfor
  
  for i=1:length(transitions)
      robot_pose_guess = t2v(XR_guess(:,:,i));
      drawRobot(robot_pose_guess, poses(i).id,zeros(3,3),'y');
  endfor
  
  h=figure(4);
  hold on;
  title("True-Guess landmarks");
 
  drawLandmarks(landmarks,'r','fill',miss_lm,'k');
  drawLandmarks(lm_guess,'magenta','fill');
  xlim([-16 16])
  ylim([-5 16]) 
  
%  h = figure(4)
%  title("Guess initial configuration");
%
%  for i=1:length(XL_guess_id)
%    lm_guess_struct = landmark(XL_guess_id(i), XL_guess(:,i)'); %create a landmark struct
%    lm_guess(end+1) = lm_guess_struct;    
%  endfor
%  drawLandmarks(lm_guess,'magenta','fill');
%  xlim([-14 16])
%  ylim([-5 16])
%  
%  for i=1:length(transitions)
%      robot_pose_guess = t2v(XR_guess(:,:,i));
%      drawRobot(robot_pose_guess, poses(i).id,zeros(3,3),'y');
%  endfor      
%  xlim([-14 16])
%  ylim([-5 16])
%  
  waitfor(h)
  
%
%  
%
%  subplot(2,2,2);
%  title("Landmark After Optimization");
%  plot(XL_true(1,:),XL_true(2,:),'b*',"linewidth",2);
%  hold on;
%  plot(XL(1,:),XL(2,:),'ro',"linewidth",2);
%  legend("Landmark True", "Guess");grid;
%
%
%  subplot(2,2,3);
%  title("Poses Initial Guess");
%  plot(XR_true(1,3,:),XR_true(2,3,:),'b*',"linewidth",2);
%  hold on;
%  plot(XR_guess(1,3,:),XR_guess(2,3,:),'ro',"linewidth",2);
%  legend("Poses True", "Guess",'Location','SouthEast');grid;
%
%  subplot(2,2,4);
%  title("Poses After Optimization");
%  plot(XR_true(1,3,:),XR_true(2,3,:),'b*',"linewidth",2);
%  hold on;
%  plot(XR(1,3,:),XR(2,3,:),'ro',"linewidth",2);
%  legend("Poses True", "Guess",'Location','SouthEast'); grid;
%  
%  % PLOT XL_true
%  drawLandmarks(landmarks);
%  
%
%  % PLOT XR_guess
%  for i=1:length(transitions)
%      robot_pose_guess = t2v(XR_guess(:,:,i));
%      drawRobot(robot_pose_guess, poses(i).id,zeros(3,3),'y');
%      xlim([-15 15])
%      ylim([-2 12])
%      %pause(0.1)
%  endfor
%  hold off
%
%  hold on
%  h  = figure(2)
%  %Draw X_true and X_guess as points 
%
%%  subplot(2,2,1);
%%  title("Landmark Initial Guess");
%%  plot(XL_true(1,:),XL_true(2,:),'b*',"linewidth",2);
%%  hold on;
%%  plot(XL_guess(1,:),XL_guess(2,:),'ro',"linewidth",2);
%%  legend("Landmark True", "Guess");grid;
%
%  subplot(2,2,3);
%  title("Poses Initial Guess");
%  plot(XR_true(1,3,:),XR_true(2,3,:),'b*',"linewidth",2);
%  hold on;
%  plot(XR_guess(1,3,:),XR_guess(2,3,:),'ro',"linewidth",2);
%  legend("Poses True", "Guess");grid;
%  hold off
%  waitfor(h)
%  
%
  %PLOT POSE AND LANDMARK

endfunction