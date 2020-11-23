function [] = reset_torque(motors)
motor7 = motors(7);
motor8 = motors(8);
wb_motor_set_torque(motor7,-0.01);
wb_motor_set_torque(motor8,-0.01);
end

