function pioneer_move_along(distance, left_right, velocity, sensors, wheel_motors, compass, TIME_STEP)
    
    front_sensors_buffer = NaN(10, 1);
    rear_sensors_buffer = NaN(10, 1);
    side_sensors_idx = [];
    sensors_data = zeros(numel(sensors), 1);
    if left_right == "left"
        side_sensors_idx = [1, 16];
        motors_close = [wheel_motors(1), wheel_motors(3)];
        motors_far = [wheel_motors(2), wheel_motors(4)];
    elseif left_right == "right"
        side_sensors_idx = [8, 9];
        motors_close = [wheel_motors(2), wheel_motors(4)];
        motors_far = [wheel_motors(1), wheel_motors(3)];
    end
    
    while wb_robot_step(TIME_STEP) ~= -1
        
        for i = 1 : numel(sensors)
            sensors_data(i) = wb_distance_sensor_get_value(sensors(i));
        end        
        front_sensors_buffer = [front_sensors_buffer(1:end-1);...
            sensors_data(side_sensors_idx(1))];
        rear_sensors_buffer = [rear_sensors_buffer(1:end-1);...
            sensors_data(side_sensors_idx(2))];                           
%         mean(rear_sensor_buffer(~isnan(rear_sensor_buffer)))


        if mean(front_sensors_buffer(~isnan(front_sensors_buffer)))...
                < distance / 2 || any(sensors_data(2:6) > distance)
            
            for i = 1 : numel(wheel_motors)
                wb_motor_set_velocity(wheel_motors(i), velocity);
            end
            break
        end
        
    end
    
    
end

