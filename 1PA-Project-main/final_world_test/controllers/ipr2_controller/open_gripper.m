function [] = open_gripper(motors)
LG = motors(3);
RG = motors(4);
wb_motor_set_position(LG,-1.21)
wb_motor_set_position(RG,1.21)
end

