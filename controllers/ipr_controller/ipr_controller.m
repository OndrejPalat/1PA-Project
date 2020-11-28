% MATLAB controller for Webots
% File:          ipr_controller.m
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

%touch_sensor
ts0 = wb_robot_get_device('ts0');
wb_touch_sensor_enable(ts0,TIME_STEP);
ts2 = wb_robot_get_device('ts2');
wb_touch_sensor_enable(ts2,TIME_STEP);
% main loop:

while wb_robot_step(TIME_STEP) ~= -1
value_dsG = wb_distance_sensor_get_value(dsG);
value_ds1 = wb_distance_sensor_get_value(ds1);
value_ds6 = wb_distance_sensor_get_value(ds6);

value_BPS = wb_position_sensor_get_value(BPS)
value_FPS = wb_position_sensor_get_value(FPS)
value_RWPS = wb_position_sensor_get_value(RWPS)
value_UPS = wb_position_sensor_get_value(UPS);
value_WPS = wb_position_sensor_get_value(WPS);

if value_BPS < 0.1 && value_FPS < 0.1 && value_RWPS < 0.1
IPR_init_grab(motors)
open_gripper(motors)
end
if value_dsG > 100
  close_gripper(motors)

end

value_ts0 = wb_touch_sensor_get_value(ts0)
value_ts2 = wb_touch_sensor_get_value(ts2)
min = wb_distance_sensor_get_min_value(dsG);
max = wb_distance_sensor_get_max_value(dsG);
if value_ts0 > 200 && value_ts2 > 200 
  second_position(motors)
end


if value_FPS > 1.89
  wb_motor_set_position(base,1.45)
end


if value_BPS > 1.44
  wb_motor_set_position(forearm,2.2)
  wb_motor_set_position(upper,-1.8)
end

if value_FPS > 2.15 && value_UPS < -1.75
  open_gripper(motors)
end

  drawnow;

end

% cleanup code goes here: write data to files, etc.
