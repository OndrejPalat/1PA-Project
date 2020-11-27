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

% get and enable devices, e.g.:
%  camera = wb_robot_get_device('camera');
%  wb_camera_enable(camera, TIME_STEP);
%  motor = wb_robot_get_device('motor');
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
BS = wb_robot_get_device('base_sensor');
wb_position_sensor_enable(BS,TIME_STEP);
FS = wb_robot_get_device('forearm_sensor');
wb_position_sensor_enable(FS,TIME_STEP);
LGS = wb_robot_get_device('left_gripper_sensor');
wb_position_sensor_enable(LGS,TIME_STEP);
RGS = wb_robot_get_device('right_gripper_sensor');
wb_position_sensor_enable(RGS,TIME_STEP);
RWS = wb_robot_get_device('rotational_wrist_sensor');
wb_position_sensor_enable(RWS,TIME_STEP);
US = wb_robot_get_device('upperarm_sensor');
wb_position_sensor_enable(US,TIME_STEP);
WS = wb_robot_get_device('wrist_sensor');
wb_position_sensor_enable(WS,TIME_STEP);

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
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%
while wb_robot_step(TIME_STEP) ~= -1
value_dsG = wb_distance_sensor_get_value(dsG)
value_ds1 = wb_distance_sensor_get_value(ds1)
value_ds6 = wb_distance_sensor_get_value(ds6)
IPR_init_grab(motors)
open_gripper(motors)
if value_dsG > 100
  %close_gripper
  wb_motor_set_torque(LG,-0.1)
wb_motor_set_torque(RG,-0.1)
end
  % read the sensors, e.g.:
  %  rgb = wb_camera_get_image(camera);
wb_touch_sensor_get_value(ts0)
wb_touch_sensor_get_value(ts2)
  % Process here sensor data, images, etc.

  % send actuator commands, e.g.:
  %  wb_motor_set_postion(motor, 10.0);

  % if your code plots some graphics, it needs to flushed like this:
  drawnow;

end

% cleanup code goes here: write data to files, etc.
