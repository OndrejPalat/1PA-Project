function [] = stop_current_position(ps,motors)
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
ps = []
ps1 = wb_robot_get_device('1 sensor');
value_ps1 = wb_position_sensor_get_value(ps1);
ps(1) = value_ps1;
ps2 = wb_robot_get_device('2 sensor');
value_ps2 = wb_position_sensor_get_value(ps2);
ps(2) = value_ps2;
ps3 = wb_robot_get_device('3 sensor');
value_ps3 = wb_position_sensor_get_value(ps3);
ps(3) = value_ps3;
ps4 = wb_robot_get_device('4 sensor');
value_ps4 = wb_position_sensor_get_value(ps4);
ps(4) = value_ps4;
ps5 = wb_robot_get_device('5 sensor');
value_ps5 = wb_position_sensor_get_value(ps5);
ps(5) = value_ps5;
ps6 = wb_robot_get_device('6 sensor');
value_ps6 = wb_position_sensor_get_value(ps6);
ps(6) = value_ps6;
ps7 = wb_robot_get_device('7 sensor');
value_ps7 = wb_position_sensor_get_value(ps7);
ps(7) = value_ps7;
ps8 = wb_robot_get_device('7 left sensor');
value_ps8 = wb_position_sensor_get_value(ps8);
ps(8) = value_ps8;
values = [];
for i = 1:8
    values(i)=ps(i);
end
wb_motor_set_position(motor1,values(1));
wb_motor_set_position(motor2,values(2));
wb_motor_set_position(motor3,values(3));
wb_motor_set_position(motor4,values(4));
wb_motor_set_position(motor5,values(5));
wb_motor_set_position(motor6,values(6));
wb_motor_set_position(motor7,values(7));
wb_motor_set_position(motor8,values(8));
disp('values')
end

