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
n_of_turns = 0;
n_of_alignments = 0;
direction_vector = [0 0 1];
reaction_distance = 900;

wheel_motors = zeros(4, 1);
wheel_motors(1) = wb_robot_get_device('front left wheel');
wheel_motors(2) = wb_robot_get_device('front right wheel');
wheel_motors(3) = wb_robot_get_device('back left wheel');
wheel_motors(4) = wb_robot_get_device('back right wheel');
for i = 1 : 4
  wb_motor_set_position(wheel_motors(i), inf);
  wb_motor_set_velocity(wheel_motors(i), velocity);
end
compass = wb_robot_get_device('compass');
wb_compass_enable(compass, TIME_STEP);


while wb_robot_step(TIME_STEP) ~= -1
  for i = 1 : numel(sensors)
    sensors_data(i) = wb_distance_sensor_get_value(sensors(i));
    message = message + "  " + string(sensors_data(i));
  end  
%  disp("Sensors data:" + newline + message);
%  message = "";
%  disp(wb_compass_get_values(compass));
        
  
  if n_of_turns > 0
    pioneer_turn(80, "CW", 0.5, wheel_motors, compass, velocity, TIME_STEP)
    n_of_turns = n_of_turns - 1;
  end
  
  if sensors_data(4) > reaction_distance || sensors_data(5) > reaction_distance
    pioneer_turn(90, "CW", 0.5, wheel_motors, compass, velocity, TIME_STEP)
  end
  if sensors_data(3) > reaction_distance
    pioneer_turn(45, "CW", 0.5, wheel_motors, compass, velocity, TIME_STEP)
  end
  if sensors_data(6) > reaction_distance
    pioneer_turn(45, "CCW", 0.5, wheel_motors, compass, velocity, TIME_STEP)
  end
  if sensors_data(2) > reaction_distance
    pioneer_turn(30, "CW", 0.5, wheel_motors, compass, velocity, TIME_STEP)
  end
  if sensors_data(7) > reaction_distance
    pioneer_turn(30, "CCW", 0.5, wheel_motors, compass, velocity, TIME_STEP)
  end
  if sensors_data(8) > reaction_distance
    pioneer_move_along(reaction_distance, "right", velocity, sensors,...
                     wheel_motors, compass, TIME_STEP)
  end
  if sensors_data(1) > reaction_distance
    pioneer_move_along(reaction_distance, "left", velocity, sensors,...
                     wheel_motors, compass, TIME_STEP)
  end
  
  if n_of_alignments > 0
    pioneer_align_to_vector(direction_vector, compass,...
                             wheel_motors, velocity, TIME_STEP);
    n_of_alignments = n_of_alignments - 1;
  end

  drawnow;

end

% cleanup code goes here: write data to files, etc.
