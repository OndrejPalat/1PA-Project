function [] = close_gripper(motors)
LG = motors(3);
RG = motors(4);
% wrist = motors(7);
% forearm = motors(2);
wb_motor_set_position(LG,0)
wb_motor_set_position(RG,0)
% wb_motor_set_position(forearm,2.4)
% wb_motor_set_position(wrist,-3.9)
end

