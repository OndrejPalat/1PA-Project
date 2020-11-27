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
turn_requests = 0;
n_of_alignments = 0;
direction_vector = [-1 0 1];
reaction_distance = 900;
time_set_direction = 0;
gps_destination = [-10 20 -10];
set_direction_request = 1;

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
emitter = wb_robot_get_device('emitter');
receiver = wb_robot_get_device('receiver');
wb_receiver_enable(receiver, TIME_STEP);
gps = wb_robot_get_device('gps');
wb_gps_enable(gps, TIME_STEP);


while wb_robot_step(TIME_STEP) ~= -1

  for i = 1 : numel(sensors)
    sensors_data(i) = wb_distance_sensor_get_value(sensors(i));
    message = message + "  " + string(sensors_data(i));
  end
  
  if wb_robot_get_time() > time_set_direction + 10
    set_direction_request = 1;
  end
  
  if set_direction_request > 0
    pioneer_set_direction(gps_destination, 100, velocity,...
                          wheel_motors, gps, compass, TIME_STEP)
    time_set_direction = wb_robot_get_time();
    set_direction_request = set_direction_request - 1;
  end
  
%  disp("Sensors data:" + newline + message);
%  message = "";
%  disp(wb_compass_get_values(compass));
        
  
  if sensors_data(2) > reaction_distance
    pioneer_turn(30, "CW", 0.5, wheel_motors, compass, velocity, TIME_STEP)
  end
  if sensors_data(3) > reaction_distance
    pioneer_turn(45, "CW", 0.5, wheel_motors, compass, velocity, TIME_STEP)
  end
  if sensors_data(4) > reaction_distance || sensors_data(5) > reaction_distance
    compass_vector = wb_compass_get_values(compass);
    cross_product = cross(compass_vector, direction_vector);
    if cross_product(2) < 0
      turn_orientation = "CW";
    else
      turn_orientation = "CCW";
    end
    pioneer_turn(90, turn_orientation, 0.5, wheel_motors, compass,...
                 velocity, TIME_STEP)
  end
  if sensors_data(6) > reaction_distance
    pioneer_turn(45, "CCW", 0.5, wheel_motors, compass, velocity, TIME_STEP)
  end
  if sensors_data(7) > reaction_distance
    pioneer_turn(30, "CCW", 0.5, wheel_motors, compass, velocity, TIME_STEP)
  end
  
  if sensors_data(8) > reaction_distance / 1.2
    turn_requests = 0;
    pioneer_move_along(reaction_distance, "right", velocity, sensors,...
                     wheel_motors, compass, TIME_STEP)
    turn_orientation = "CW";
    turn_requests = 6;
  end  
  if sensors_data(1) > reaction_distance / 1.2
    turn_requests = 0;
    pioneer_move_along(reaction_distance, "left", velocity, sensors,...
                     wheel_motors, compass, TIME_STEP)
    turn_orientation = "CCW";
    turn_requests = 6;
%    pioneer_turn(90, "CCW", 0.5, wheel_motors, compass, velocity, TIME_STEP)
  end
  
  if n_of_alignments > 0
    pioneer_align_to_vector(direction_vector, compass,...
                             wheel_motors, velocity, TIME_STEP);
    n_of_alignments = n_of_alignments - 1;
  end

  if turn_requests > 0
    pioneer_turn(15, turn_orientation, 0, wheel_motors, compass,...
                 velocity, TIME_STEP)
    turn_requests = turn_requests - 1;
  end
  
  drawnow;

end

% cleanup code goes here: write data to files, etc.
