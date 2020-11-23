function [time_step] = get_time_step()
time_step = -1;
if time_step == -1
    time_step = wb_robot_get_basic_time_step();
end
end

