function [angle_deviation, turn_requests] = pioneer_move_along(distance, left_right, velocity,angle_deviation,...
        direction_vector, sensors, wheel_motors, compass, gps, TIME_STEP)
    
    front_sensors_buffer = NaN(10, 1);
    rear_sensors_buffer = NaN(10, 1);
    sensors_idx = [];
    sensors_data = zeros(numel(sensors), 1);
    align_to_obstacle_requests = 1;
    time_general = 0;
    break_delay = 0;
    break_requests = 0;
    time_align_start = 0;
    orientation_ratio = 0;
    front_rear_difference = 0;
    turn_requests = 0;
    if left_right == "left"
        sensors_idx = [16, 1, 2, 3, 4, 5, 6, 7, 8];
        turn_orientation = "CW";
        turn_orientation2 = "CCW";
        orientation_ratio = 1;
    elseif left_right == "right"
        sensors_idx = [9, 8, 7, 6, 5, 4, 3, 2, 1];
        turn_orientation = "CCW";
        turn_orientation2 = "CW";
        orientation_ratio = -1;
    end
    
    while wb_robot_step(TIME_STEP) ~= -1        
        
        for i = 1 : numel(sensors)
            sensors_data(i) = wb_distance_sensor_get_value(sensors(i));
        end        
        front_sensors_buffer = [front_sensors_buffer(2:end);...
            sensors_data(sensors_idx(2))];
        rear_sensors_buffer = [rear_sensors_buffer(2:end);...
            sensors_data(sensors_idx(1))];
        
        if mean(front_sensors_buffer(~isnan(front_sensors_buffer))) < distance * 7/9
            break_requests = 1;
            turn_requests = 6;
            break_delay = 3;
            align_to_obstacle_requests = 0;
        end
        
        if align_to_obstacle_requests > 0
            front_sensors_buffer = NaN(10, 1);
            rear_sensors_buffer = NaN(10, 1);
            time_general = wb_robot_get_time;
            while wb_robot_step(TIME_STEP) ~= 1
                if wb_robot_get_time > time_general + 1
                    break
                end
            end
            time_align_start = wb_robot_get_time();            
            for i = 1 : numel(wheel_motors)
                wb_motor_set_velocity(wheel_motors(i), 0);
            end            
            while wb_robot_step(TIME_STEP) ~= -1                
                for i = 1 : numel(sensors)
                    sensors_data(i) = wb_distance_sensor_get_value(sensors(i));
                end                
                front_sensors_buffer = [front_sensors_buffer(2:end);...                    
                    sensors_data(sensors_idx(2))];
                rear_sensors_buffer = [rear_sensors_buffer(2:end);...                    
                    sensors_data(sensors_idx(1))];                
                front_rear_difference = mean(front_sensors_buffer) - mean(rear_sensors_buffer);
                
                if front_rear_difference < distance / 8000
                   for i = 1 : numel(wheel_motors) / 2
                       wb_motor_set_velocity(wheel_motors(2*i), orientation_ratio * velocity / 10);
                       wb_motor_set_velocity(wheel_motors(2*i-1), -orientation_ratio * velocity / 10);
                   end
                end
                if front_rear_difference > -distance / 8000
                   for i = 1 : numel(wheel_motors) / 2
                       wb_motor_set_velocity(wheel_motors(2*i), -orientation_ratio * velocity / 10);
                       wb_motor_set_velocity(wheel_motors(2*i-1), orientation_ratio * velocity / 10);
                   end
                end                
                if abs(front_rear_difference) < distance / 8000
                    for i = 1 : numel(wheel_motors)
                        wb_motor_set_velocity(wheel_motors(i), velocity);
                    end
                    break
                end
                if wb_robot_get_time() > time_align_start + 5
                    break_requests = 1;
                    break_delay = 1;
                    turn_requests = 0;
                    break
                end
            end
            align_to_obstacle_requests = 0;
        end
        
        if break_requests > 0
            for i = 1 : numel(wheel_motors)
                wb_motor_set_velocity(wheel_motors(i), velocity);
            end
            time_general = wb_robot_get_time();
            while wb_robot_step(TIME_STEP) ~= -1
                if wb_robot_get_time() > time_general + break_delay
                    break
                end
            end
            break
        end
        
        
        if sensors_data(sensors_idx(3)) > distance
            pioneer_turn(30, turn_orientation, 0, wheel_motors,...
                         compass, velocity, TIME_STEP)
            align_to_obstacle_requests = align_to_obstacle_requests + 1;
            angle_deviation = angle_deviation + orientation_ratio * 30;
        end
        if sensors_data(sensors_idx(4)) > distance
            pioneer_turn(45, turn_orientation, 0, wheel_motors,...
                         compass, velocity, TIME_STEP)
            align_to_obstacle_requests = align_to_obstacle_requests + 1;
            angle_deviation = angle_deviation + orientation_ratio * 45;
        end
        if sensors_data(sensors_idx(5)) > distance ||...
                sensors_data(sensors_idx(6)) > distance
            pioneer_turn(90, turn_orientation, 0, wheel_motors,...
                         compass, velocity, TIME_STEP)
            align_to_obstacle_requests = align_to_obstacle_requests + 1;
            angle_deviation = angle_deviation + orientation_ratio * 90;
        end
        if sensors_data(sensors_idx(7)) > distance
            pioneer_turn(135, turn_orientation, 0, wheel_motors,...
                         compass, velocity, TIME_STEP)
            align_to_obstacle_requests = align_to_obstacle_requests + 1;
            angle_deviation = angle_deviation + orientation_ratio * 135;
        end
        if sensors_data(sensors_idx(8)) > distance
            pioneer_turn(150, turn_orientation, 0, wheel_motors,...
                         compass, velocity, TIME_STEP)
            align_to_obstacle_requests = align_to_obstacle_requests + 1;
            angle_deviation = angle_deviation + orientation_ratio * 150;
        end
        
        if sensors_data(sensors_idx(2)) > 11/10 * distance
            pioneer_turn(5, turn_orientation, 0, wheel_motors,...
                         compass, velocity, TIME_STEP)
        end
        if sensors_data(sensors_idx(9)) < 8/9 * distance &&...
                sensors_data(sensors_idx(9)) > 7/9 * distance
            pioneer_turn(5, turn_orientation2, 0, wheel_motors,...
                         compass, velocity, TIME_STEP)
        end
        

        
    end
    
    
end

