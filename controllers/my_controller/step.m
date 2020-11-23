function [] = step()
if wb_robot_step(get_time_step()) == -1
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
end
end

