function out = drawRobot(pose, pose_id, covariance, color)

	hold on;
	dim = 0.25;
	arr_len = 0.5;

	drawShape('rect', [pose(1), pose(2), dim, dim, pose(3)], 'fill',color);
  hold on;
	drawLabels(pose(1), pose(2), pose_id-1299, '%d');

end
