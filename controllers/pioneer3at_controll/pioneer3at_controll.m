% MATLAB controller for Webots
% File:          pioneer3at_controll.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

TIME_STEP = 32;

sensors = zeros(16, 1);
for i = 1 : numel(sensors)
  sensors(i) = wb_robot_get_device(['so', num2str(i - 1)]);
  wb_distance_sensor_enable(sensors(i), TIME_STEP);
end

velocity = 3;
sensors_data = zeros(numel(sensors), 1);
message = "";

wheel_motors = zeros(4, 1);
wheel_motors(1) = wb_robot_get_device('front left wheel');
wheel_motors(2) = wb_robot_get_device('front right wheel');
wheel_motors(3) = wb_robot_get_device('back left wheel');
wheel_motors(4) = wb_robot_get_device('back right wheel');
for i = 1 : 4
  wb_motor_set_position(wheel_motors(i), inf);
  wb_motor_set_velocity(wheel_motors(i), 3);
end
compass = wb_robot_get_device('compass');
wb_compass_enable(compass, TIME_STEP);

while wb_robot_step(TIME_STEP) ~= -1

  for i = 1 : numel(sensors)
    sensors_data(i) = wb_distance_sensor_get_value(sensors(i));
    message = message + "  " + string(sensors_data(i));
  end  
%  disp("Sensors data:" + newline + message);
  message = "";
  
  
  if sensors_data(4) > 900 | sensors_data(5) > 900
    pioneer_turn(90, "CW", wheel_motors, compass, velocity, TIME_STEP)
  end
  if sensors_data(3) > 900
    pioneer_turn(45, "CW", wheel_motors, compass, velocity, TIME_STEP)
  end
    if sensors_data(6) > 900
    pioneer_turn(45, "CCW", wheel_motors, compass, velocity, TIME_STEP)
  end
  % read the sensors, e.g.:
  %  rgb = wb_camera_get_image(camera);

  drawnow;

end

% cleanup code goes here: write data to files, etc.
