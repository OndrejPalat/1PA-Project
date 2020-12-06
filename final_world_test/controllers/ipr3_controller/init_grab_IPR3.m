function [] = init_grab_IPR3(motors)
base = motors(1);
forearm = motors(2);
wrist = motors(7);
upper = motors(6);
RW = motors(5);
wb_motor_set_position(base,4.6)%4.6
wb_motor_set_position(forearm,1)%1
wb_motor_set_position(wrist,-2.8)%-2.8
wb_motor_set_position(upper,-1.8)%-1.8
wb_motor_set_position(RW,-3)%-3
end

