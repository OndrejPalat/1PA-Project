function [] = open_gripper(motors)
LG = motors(3);
RG = motors(4);
wb_motor_set_position(LG,-0.7)
wb_motor_set_position(RG,0.7)
end

