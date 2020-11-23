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

TIME_STEP = 32;

% get and enable devices, e.g.:
motors = [];
motor1 = wb_robot_get_device('1');
motors(1) = motor1;
motor2 = wb_robot_get_device('2');
motors(2) = motor2;
motor3 = wb_robot_get_device('3');
motors(3) = motor3;
motor4 = wb_robot_get_device('4');
motors(4) = motor4;
motor5 = wb_robot_get_device('5');
motors(5) = motor5;
motor6 = wb_robot_get_device('6');
motors(6) = motor6;
motor7 = wb_robot_get_device('7');
motors(7) = motor7;
motor8 = wb_robot_get_device('7 left');
motors(8) = motor8;

ps = [];
ps1 = wb_robot_get_device('1 sensor');
wb_position_sensor_enable(ps1,TIME_STEP);
ps(1) = ps1;
ps2 = wb_robot_get_device('2 sensor');
wb_position_sensor_enable(ps2,TIME_STEP);
ps(2) = ps2;
ps3 = wb_robot_get_device('3 sensor');
wb_position_sensor_enable(ps3,TIME_STEP);
ps(3) = ps3;
ps4 = wb_robot_get_device('4 sensor');
wb_position_sensor_enable(ps4,TIME_STEP);
ps(4) = ps4;
ps5 = wb_robot_get_device('5 sensor');
wb_position_sensor_enable(ps5,TIME_STEP);
ps(5) = ps5;
ps6 = wb_robot_get_device('6 sensor');
wb_position_sensor_enable(ps6,TIME_STEP);
ps(6) = ps6;
ps7 = wb_robot_get_device('7 sensor');
wb_position_sensor_enable(ps7,TIME_STEP);
ps(7) = ps7;
ps8 = wb_robot_get_device('7 left sensor');
wb_position_sensor_enable(ps8,TIME_STEP);
ps(8) = ps8;

dsG = wb_robot_get_device('gripper middle distance sensor');
wb_distance_sensor_enable(dsG,TIME_STEP);
ds1L = wb_robot_get_device('left finger sensor 0');
wb_distance_sensor_enable(ds1L,TIME_STEP);
ds2L = wb_robot_get_device('left finger sensor 1');
wb_distance_sensor_enable(ds2L,TIME_STEP);
ds3L = wb_robot_get_device('left finger sensor 2');
wb_distance_sensor_enable(ds3L,TIME_STEP);
%inner_ds
ds4L = wb_robot_get_device('left finger sensor 3');
wb_distance_sensor_enable(ds4L,TIME_STEP);

min_value = wb_distance_sensor_get_min_value(dsG);

ds1R = wb_robot_get_device('right finger sensor 0');
wb_distance_sensor_enable(ds1R,TIME_STEP);
ds2R = wb_robot_get_device('right finger sensor 1');
wb_distance_sensor_enable(ds2R,TIME_STEP);
ds3R = wb_robot_get_device('right finger sensor 2');
wb_distance_sensor_enable(ds3R,TIME_STEP);
%inner_ds
ds4R = wb_robot_get_device('right finger sensor 3');
wb_distance_sensor_enable(ds4R,TIME_STEP);
while wb_robot_step(TIME_STEP) ~= -1

  % read the sensors, e.g.:
%value_dsG = wb_distance_sensor_get_value(dsG)
%if value_dsG == min_value
%  init_grab_position(motors)
%end

%if value_dsG == 100 || value_dsG == 1500
value_ps1 = wb_position_sensor_get_value(ps1);
if value_ps1 < 0.1
init_grab_position(motors)
open_gripper(motors)
end

%else
  %stop_current_position(ps,motors)
%end
value_ps1 = wb_position_sensor_get_value(ps1);
value_ps2 = wb_position_sensor_get_value(ps2);
value_ps7 = wb_position_sensor_get_value(ps7);
%value_ps8 = wb_position_sensor_get_value(ps8)
%value_ds1L = wb_distance_sensor_get_value(ds1L)
%value_ds2L = wb_distance_sensor_get_value(ds2L)
%value_ds3L = wb_distance_sensor_get_value(ds3L)
%value_ds4L = wb_distance_sensor_get_value(ds4L)
%value_ds1R = wb_distance_sensor_get_value(ds1R)
%value_ds2R = wb_distance_sensor_get_value(ds2R)
%value_ds3R = wb_distance_sensor_get_value(ds3R)
%value_ds4R = wb_distance_sensor_get_value(ds4R)
%disp('value_dsG')
value_dsG = wb_distance_sensor_get_value(dsG);
if value_ps2 > 1.56
  grip(motors)
end
if value_ps7 < 0.28 && value_ps2 > 1.56
  second_position(motors)
end
if value_ps1 > 0.05
  reset_torque(motors)
end
%value_ps3 = wb_position_sensor_get_value(ps3)
%disp(value_ps3)
%type = wb_distance_sensor_get_type(dsG)
%a = wb_distance_sensor_get_aperture(dsG)
  % Process here sensor data, images, etc.
%if value_dsG



  drawnow;

end

