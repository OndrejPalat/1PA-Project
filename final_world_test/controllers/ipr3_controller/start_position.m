function [] = start_position(motors)
base = motors(1);
forearm = motors(2);
wrist = motors(7);
upper = motors(6);
RW = motors(5);
wb_motor_set_position(base,1.7)
wb_motor_set_position(forearm,0)%2
wb_motor_set_position(wrist,0)%-3.55
wb_motor_set_position(upper,0)
wb_motor_set_position(RW,-2.95)%-2.95
end

