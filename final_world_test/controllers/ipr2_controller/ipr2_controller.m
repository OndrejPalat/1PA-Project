% MATLAB controller for Webots
% File:          ipr2_controller.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

TIME_STEP = 64;

%motors
motors = [];
base = wb_robot_get_device('base');
motors(1) = base;
forearm = wb_robot_get_device('forearm');
motors(2) = forearm;
LG = wb_robot_get_device('left_gripper');
motors(3) = LG;
RG = wb_robot_get_device('right_gripper');
motors(4) = RG;
RW = wb_robot_get_device('rotationalwrist');
motors(5) = RW;
upper = wb_robot_get_device('upperarm');
motors(6) = upper;
wrist = wb_robot_get_device('wrist');
motors(7) = wrist;

%position sensors
BPS = wb_robot_get_device('base_sensor');
wb_position_sensor_enable(BPS,TIME_STEP);
FPS = wb_robot_get_device('forearm_sensor');
wb_position_sensor_enable(FPS,TIME_STEP);
LGPS = wb_robot_get_device('left_gripper_sensor');
wb_position_sensor_enable(LGPS,TIME_STEP);
RGPS = wb_robot_get_device('right_gripper_sensor');
wb_position_sensor_enable(RGPS,TIME_STEP);
RWPS = wb_robot_get_device('rotational_wrist_sensor');
wb_position_sensor_enable(RWPS,TIME_STEP);
UPS = wb_robot_get_device('upperarm_sensor');
wb_position_sensor_enable(UPS,TIME_STEP);
WPS = wb_robot_get_device('wrist_sensor');
wb_position_sensor_enable(WPS,TIME_STEP);

%distance sensors
dsG = wb_robot_get_device('ds4');
wb_distance_sensor_enable(dsG,TIME_STEP);
ds1 = wb_robot_get_device('ds1');
wb_distance_sensor_enable(ds1,TIME_STEP);
ds6 = wb_robot_get_device('ds6');
wb_distance_sensor_enable(ds6,TIME_STEP);

dsCB2 = wb_robot_get_device('dsCB2');
wb_distance_sensor_enable(dsCB2,TIME_STEP);

%touch_sensor
ts0 = wb_robot_get_device('ts0');
wb_touch_sensor_enable(ts0,TIME_STEP);
ts2 = wb_robot_get_device('ts2');
wb_touch_sensor_enable(ts2,TIME_STEP);
% main loop:

while wb_robot_step(TIME_STEP) ~= -1

value_dsCB2 = wb_distance_sensor_get_value(dsCB2);
if value_dsCB2 < 999
  init_grab_IPR2(motors)
  open_gripper(motors)
end

value_BPS = wb_position_sensor_get_value(BPS);
value_dsG = wb_distance_sensor_get_value(dsG);
if value_dsG > 40 && value_BPS < 1.5
  close_gripper(motors)
end

value_ts0 = wb_touch_sensor_get_value(ts0);
value_LGPS = wb_position_sensor_get_value(LGPS);
if value_ts0 > 200 && value_BPS < 1.49 && LGPS > 0.01
  second_position2(motors)
end
if value_ts0 > 200 && value_BPS > 1.49
  second_position(motors)
end
if value_ts0 > 200 && value_BPS > 2.99
  open_gripper(motors)
  start_position(motors)
end

  % read the sensors, e.g.:
  %  rgb = wb_camera_get_image(camera);

  % Process here sensor data, images, etc.

  % send actuator commands, e.g.:
  %  wb_motor_set_postion(motor, 10.0);

  % if your code plots some graphics, it needs to flushed like this:
  drawnow;

end

% cleanup code goes here: write data to files, etc.
