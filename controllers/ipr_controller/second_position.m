function [] = second_position(motors)
base = motors(1);
forearm = motors(2);
wrist = motors(7);
upper = motors(6);
RW = motors(5);
wb_motor_set_position(base,0)
wb_motor_set_position(forearm,1.9)
wb_motor_set_position(wrist,-4)
wb_motor_set_position(upper,-1.5)
wb_motor_set_position(RW,-1.5707)
end

