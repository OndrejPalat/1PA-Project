function [] = second_position2(motors)
base = motors(1);
forearm = motors(2);
wrist = motors(7);
upper = motors(6);
RW = motors(5);
wb_motor_set_position(base,1.7)
wb_motor_set_position(forearm,1)
wb_motor_set_position(wrist,-3.2)
wb_motor_set_position(upper,-1.45)
wb_motor_set_position(RW,-2.95)
end

