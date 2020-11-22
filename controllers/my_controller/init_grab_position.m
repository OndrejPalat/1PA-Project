function [] = init_grab_position(motors)
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
wb_motor_set_position(motor5,-1);
wb_motor_set_position(motor7,0.5);
wb_motor_set_position(motor8,0.5);
wb_motor_set_position(motor3,1);
wb_motor_set_position(motor2,1.5707);
end

