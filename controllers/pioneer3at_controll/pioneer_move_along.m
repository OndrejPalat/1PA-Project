function pioneer_move_along(distance, left_right, velocity, sensors, wheel_motors, compass, TIME_STEP)
    
    front_sensors_buffer = NaN(10, 1);
    rear_sensors_buffer = NaN(10, 1);
    sensors_idx = [];
    sensors_data = zeros(numel(sensors), 1);
    start_of_function = true;
    if left_right == "left"
        sensors_idx = [16, 1, 2, 3, 4, 5, 6, 7];
        motors_close = [wheel_motors(1), wheel_motors(3)];
        motors_far = [wheel_motors(2), wheel_motors(4)];
        turn_orientation = "CW";
    elseif left_right == "right"
        sensors_idx = [9, 8, 7, 6, 5, 4, 3, 2];
        motors_close = [wheel_motors(2), wheel_motors(4)];
        motors_far = [wheel_motors(1), wheel_motors(3)];
        turn_orientation = "CCW";
    end
    
    while wb_robot_step(TIME_STEP) ~= -1
        
        for i = 1 : numel(sensors)
            sensors_data(i) = wb_distance_sensor_get_value(sensors(i));
        end        
        front_sensors_buffer = [front_sensors_buffer(1:end-1);...
            sensors_data(sensors_idx(2))];
        rear_sensors_buffer = [rear_sensors_buffer(1:end-1);...
            sensors_data(sensors_idx(1))];                           
%         mean(rear_sensor_buffer(~isnan(rear_sensor_buffer)))
        
        if start_of_function
            
            start_of_function = false;           
        end


        
        if sensors_data(sensors_idx(3)) > distance
            pioneer_turn(30, turn_orientation, 0.5, wheel_motors,...
                         compass, velocity, TIME_STEP)
        end
        if sensors_data(sensors_idx(4)) > distance
            pioneer_turn(45, turn_orientation, 0.5, wheel_motors,...
                         compass, velocity, TIME_STEP)
        end
        if sensors_data(sensors_idx(5)) > distance ||...
                sensors_data(sensors_idx(6)) > distance
            pioneer_turn(90, turn_orientation, 0.5, wheel_motors,...
                         compass, velocity, TIME_STEP)
        end
        if sensors_data(sensors_idx(7)) > distance
            pioneer_turn(135, turn_orientation, 0.5, wheel_motors,...
                         compass, velocity, TIME_STEP)
        end
        if sensors_data(sensors_idx(8)) > distance
            pioneer_turn(150, turn_orientation, 0.5, wheel_motors,...
                         compass, velocity, TIME_STEP)
        end
        
        if mean(front_sensors_buffer(~isnan(front_sensors_buffer))) < distance / 2            
            for i = 1 : numel(wheel_motors)
                wb_motor_set_velocity(wheel_motors(i), velocity);
            end
            time = wb_robot_get_time();
            while wb_robot_step(TIME_STEP) ~= -1
                if wb_robot_get_time() > time + 3
                    break
                end
            end
            break
        end
        
    end
    
    
end

