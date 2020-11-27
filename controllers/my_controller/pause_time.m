function [actual_time] = pause_time(time)
start_time = wb_robot_get_time();
actual_time = start_time + time;
end

