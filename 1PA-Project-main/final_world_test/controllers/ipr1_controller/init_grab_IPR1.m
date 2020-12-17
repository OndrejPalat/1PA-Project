function [] = init_grab_IPR1(motors)
base = motors(1);
forearm = motors(2);
wrist = motors(7);
upper = motors(6);
RW = motors(5);
wb_motor_set_position(base,3)
wb_motor_set_position(forearm,2)
wb_motor_set_position(wrist,-3.55)
wb_motor_set_position(upper,-2)
wb_motor_set_position(RW,-3)
end

