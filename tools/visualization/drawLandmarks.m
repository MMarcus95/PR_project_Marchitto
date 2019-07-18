function done = drawLandmarks(land,color, mode, miss_lm,color_miss)
	
	if(nargin == 1)
		color = 'r';
		mode = 'fill';
    miss_lm = [];
    color_miss = '';
  elseif (nargin == 3)
    miss_lm = [];
    color_miss = '';
	endif

	N = length(land);
	radius = 0.1;
	for i=1:N
    if find(miss_lm==land(i).id)    
      drawShape('circle', [land(i).x_pose, land(i).y_pose, radius], mode, color_miss);
      hold on;
      drawLabels(land(i).x_pose, land(i).y_pose, land(i).id, '%d');
      hold on;
    else
      drawShape('circle', [land(i).x_pose, land(i).y_pose, radius], mode, color);
      hold on;
      drawLabels(land(i).x_pose, land(i).y_pose, land(i).id, '%d');
      hold on;
    endif      
	end
end
