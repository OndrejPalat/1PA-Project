function [] = pause_time(time)
start_time = wb_robot_get_time();
while true
    step();
    while start_time + time > wb_robot_get_time()
        step();
    end
end
end

