function [] = grip(motors)
motor7 = motors(7);
motor8 = motors(8);
wb_motor_set_torque(motor7,-0.2);
wb_motor_set_torque(motor8,-0.2);
wb_motor_set_position(motor7,0);
wb_motor_set_position(motor8,0);
end

