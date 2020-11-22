% MATLAB controller for Webots
% File:          my_controller.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

TIME_STEP = 64;

% get and enable devices, e.g.:

%  motor = wb_robot_get_device('motor');
motors = [];
motors_names = ["1","2","3","4","5","6","7","7 left"];
for i = 1:8
   motors(i) = wb_robot_get_device('motors_names(i)');
end

while wb_robot_step(TIME_STEP) ~= -1

  % read the sensors, e.g.:
;
for i = 1:8
  wb_robot_set_position(motors(i),1)
end
  % Process here sensor data, images, etc.




  drawnow;

end


