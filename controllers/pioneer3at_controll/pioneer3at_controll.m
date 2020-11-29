% MATLAB controller for Webots
% File:          pioneer3at_controll.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
desktop;
keyboard;

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
direction_vector = [0 0 0];
reaction_distance = 900;
time_set_direction = 0;
gps_destination = [100 20 -100];
set_direction_requests = 1;
angle_deviation = 0;

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
    set_direction_requests = 1;
  end
  
  if set_direction_requests > 0
    direction_vector = pioneer_set_direction(gps_destination, 100,...
                       velocity, wheel_motors, gps, compass, TIME_STEP);
    pioneer_align_to_vector(direction_vector, compass,...
                             wheel_motors, velocity, TIME_STEP);
    angle_deviation = 0;
    turn_requests = 0;
    time_set_direction = wb_robot_get_time();
    set_direction_requests = set_direction_requests - 1;
  end
  
%  disp("Sensors data:" + newline + message);
%  message = "";
%  disp(wb_compass_get_values(compass));
        
  
  if sensors_data(2) > reaction_distance
    pioneer_turn(30, "CW", 0, wheel_motors, compass, velocity, TIME_STEP)
    angle_deviation = angle_deviation + 30;
  end
  if sensors_data(3) > reaction_distance
    pioneer_turn(45, "CW", 0, wheel_motors, compass, velocity, TIME_STEP)
    angle_deviation = angle_deviation + 45;
  end
  if sensors_data(4) > reaction_distance || sensors_data(5) > reaction_distance
    compass_vector = wb_compass_get_values(compass);
    cross_product = cross(compass_vector, direction_vector);
    if cross_product(2) < 0
      turn_orientation = "CW";
      angle_deviation = angle_deviation + 90;
    else
      turn_orientation = "CCW";
      angle_deviation = angle_deviation - 90;
    end
    pioneer_turn(90, turn_orientation, 0, wheel_motors, compass,...
                 velocity, TIME_STEP)
  end
  if sensors_data(6) > reaction_distance
    pioneer_turn(45, "CCW", 0, wheel_motors, compass, velocity, TIME_STEP)
    angle_deviation = angle_deviation - 45;
  end
  if sensors_data(7) > reaction_distance
    pioneer_turn(30, "CCW", 0, wheel_motors, compass, velocity, TIME_STEP)
    angle_deviation = angle_deviation - 30;
  end
 
  if sensors_data(1) > reaction_distance / 1.2
    turn_requests = 0;
    [angle_deviation, turn_requests] = pioneer_move_along(reaction_distance, "left", velocity, angle_deviation,...
                       direction_vector, sensors, wheel_motors, compass, gps, TIME_STEP);
    turn_orientation = "CCW";
    time_set_direction = wb_robot_get_time - 5;
  end  
  if sensors_data(8) > reaction_distance / 1.2
    turn_requests = 0;
    [angle_deviation, turn_requests] = pioneer_move_along(reaction_distance, "right", velocity, angle_deviation,...
                       direction_vector, sensors, wheel_motors, compass, gps, TIME_STEP);
    turn_orientation = "CW";
    time_set_direction = wb_robot_get_time - 5;
  end  


  if turn_requests > 0
    pioneer_turn(15, turn_orientation, 0, wheel_motors, compass,...
                 velocity, TIME_STEP)
    if turn_orientation == "CW"
      angle_deviation = angle_deviation + 15;
    end
    if turn_orientation == "CCW"
      angle_deviation = angle_deviation - 15;
    end
    turn_requests = turn_requests - 1;
  end
  
  drawnow;

end

% cleanup code goes here: write data to files, etc.
