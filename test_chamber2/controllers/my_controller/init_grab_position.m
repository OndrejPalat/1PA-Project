function [] = init_grab_position(motors)
motor5 = motors(5);
motor3 = motors(3);
motor2 = motors(2);
wb_motor_set_position(motor5,-1);
wb_motor_set_position(motor3,1.05);
wb_motor_set_position(motor2,1.5707);
end

