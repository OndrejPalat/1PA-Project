function [] = second_position(motors)
motor1 = motors(1);
motor2 = motors(2);
motor3 = motors(3);
motor4 = motors(4);
motor5 = motors(5);
motor6 = motors(6);
wb_motor_set_position(motor1,1.5707);
wb_motor_set_position(motor2,1.5);
wb_motor_set_position(motor3,0);
wb_motor_set_position(motor4,0);
wb_motor_set_position(motor5,0);
wb_motor_set_position(motor6,0);
end

