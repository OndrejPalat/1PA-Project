function [angle_deviation, turn_requests, turn_orientation2, set_radio_requests] =...
    pioneer_move_along(distance, left_right, velocity, angle_deviation, gps_destination,...
    sensors, parking_sensors, wheel_motors, compass, gps, receiver, TIME_STEP)
    
    front_sensors_buffer = NaN(10, 1);
    rear_sensors_buffer = NaN(10, 1);
    sensors_idx = [];
    parking_sensors_idx = [];
    sensors_data = zeros(numel(sensors), 1);
    parking_sensors_data = zeros(numel(parking_sensors), 1);
    align_to_obstacle_requests = 1;
    time_count = 0;
    break_delay = 0;
    break_requests = 0;
    break_time_count = 0;
    align_delay = 0.5;
    parking_delay = 1;
    time_align_start = 0;
    orientation_ratio = 0;
    front_rear_difference = 0;
    turn_requests = 0;
    parking_signal = 0;
    time_turn = 0;
    turn_delay = 1;
    set_radio_requests = 0;
    message = '';
    time_parking = 0;
    is_parked = 0;
    move_along_deviation = 0;
    move_along_deviation2 = 0;
    
    if left_right == "left"
        sensors_idx = [16, 1, 2, 3, 4, 5, 6, 7, 8];
        parking_sensors_idx = [1 2 3];
        turn_orientation = "CW";
        turn_orientation2 = "CCW";
        orientation_ratio = 1;
    elseif left_right == "right"
        sensors_idx = [9, 8, 7, 6, 5, 4, 3, 2, 1];
        parking_sensors_idx = [6 5 4];
        turn_orientation = "CCW";
        turn_orientation2 = "CW";
        orientation_ratio = -1;
    end
    
    while wb_robot_step(TIME_STEP) ~= -1        
        
        for i = 1 : numel(sensors)
            sensors_data(i) = wb_distance_sensor_get_value(sensors(i));
        end
        for i = 1 : numel(parking_sensors)
            parking_sensors_data(i) = wb_distance_sensor_get_value(parking_sensors(i));
        end
        
        if wb_receiver_get_queue_length(receiver) > 0
            message = wb_receiver_get_data(receiver, 'double');
            message = char(message');
            wb_receiver_next_packet(receiver);
        end
        
        front_sensors_buffer = [front_sensors_buffer(2:end);...
            sensors_data(sensors_idx(2))];
        rear_sensors_buffer = [rear_sensors_buffer(2:end);...
            sensors_data(sensors_idx(1))];
        
        if mean(front_sensors_buffer(~isnan(front_sensors_buffer)))...
                < distance * 7.8/9 && break_requests == 0 && parking_signal == 0
            break_requests = 1;
            turn_requests = 6;
            break_delay = 2;
            break_time_count = wb_robot_get_time;
            align_to_obstacle_requests = 0;
        end
        
        if align_to_obstacle_requests > 0
            front_sensors_buffer = NaN(10, 1);
            rear_sensors_buffer = NaN(10, 1);
            time_count = wb_robot_get_time();
            while wb_robot_step(TIME_STEP) ~= -1
                if wb_robot_get_time() > time_count + align_delay
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
                if wb_robot_get_time() > time_align_start + 5 && break_requests == 0
                    break_requests = 1;
                    break_delay = 1;
                    break_time_count = wb_robot_get_time;
                    turn_requests = 0;
                    break
                end
            end
            align_to_obstacle_requests = 0;
        end
        
        
        if sensors_data(sensors_idx(3)) > distance && parking_signal == 0
            pioneer_turn(30, turn_orientation, 0, wheel_motors,...
                         compass, velocity, TIME_STEP)
            align_to_obstacle_requests = align_to_obstacle_requests + 1;
            move_along_deviation = move_along_deviation + 30;
            break_requests = 0;
        end
        if sensors_data(sensors_idx(4)) > distance && parking_signal == 0
            pioneer_turn(45, turn_orientation, 0, wheel_motors,...
                         compass, velocity, TIME_STEP)
            align_to_obstacle_requests = align_to_obstacle_requests + 1;
            move_along_deviation = move_along_deviation + 45;
            break_requests = 0;
        end
        if sensors_data(sensors_idx(5)) > distance ||...
                sensors_data(sensors_idx(6)) > distance
            pioneer_turn(90, turn_orientation, 0, wheel_motors,...
                         compass, velocity, TIME_STEP)
            align_to_obstacle_requests = align_to_obstacle_requests + 1;
            move_along_deviation = move_along_deviation + 90;
            break_requests = 0;
        end
        if sensors_data(sensors_idx(7)) > distance
            pioneer_turn(135, turn_orientation, 0, wheel_motors,...
                         compass, velocity, TIME_STEP)
            align_to_obstacle_requests = align_to_obstacle_requests + 1;
            move_along_deviation = move_along_deviation + 135;
            break_requests = 0;
        end
        if sensors_data(sensors_idx(8)) > distance
            pioneer_turn(150, turn_orientation, 0, wheel_motors,...
                         compass, velocity, TIME_STEP)
            align_to_obstacle_requests = align_to_obstacle_requests + 1;
            move_along_deviation = move_along_deviation + 150;
            break_requests = 0;
        end
        
        if (sensors_data(sensors_idx(2)) > 11/10 * distance ||... 
                sensors_data(sensors_idx(2)) > 980) &&...
                wb_robot_get_time > time_turn + turn_delay &&...
                parking_signal == 0
            
            pioneer_turn(5, turn_orientation, 0, wheel_motors,...
                         compass, velocity, TIME_STEP)
            move_along_deviation = move_along_deviation + 5;
            time_turn = wb_robot_get_time;
        end
        if sensors_data(sensors_idx(2)) < 8.2/9 * distance &&...
                sensors_data(sensors_idx(2)) > 7.8/9 * distance &&...
                wb_robot_get_time > time_turn + turn_delay &&...
                parking_signal == 0
            
            pioneer_turn(5, turn_orientation2, 0, wheel_motors,...
                         compass, velocity, TIME_STEP);
            move_along_deviation2 = move_along_deviation2 + 5;
            time_turn = wb_robot_get_time;
        end
        
        if parking_sensors_data(parking_sensors_idx(1)) > 30 &&...
                parking_sensors_data(parking_sensors_idx(1)) < 250 &&...
                parking_sensors_data(parking_sensors_idx(2)) > 300 &&...
                parking_sensors_data(parking_sensors_idx(2)) < 750 &&...
                parking_sensors_data(parking_sensors_idx(3)) > 750 &&...
                parking_sensors_data(parking_sensors_idx(3)) < 950 &&...
                parking_signal == 0 &&...
                sensors_data(sensors_idx(2)) > 8/9 * distance
            
            direction_vector = pioneer_get_direction(gps_destination, 10,...                
                    velocity, wheel_motors, gps, compass, TIME_STEP);
            time_count = wb_robot_get_time;
            while wb_robot_step(TIME_STEP) ~= -1
                if wb_robot_get_time > time_count + parking_delay
                    break
                end
            end
            if norm(direction_vector) < 20
                pioneer_turn(30, turn_orientation2, 0, wheel_motors,...
                      compass, velocity, TIME_STEP);
                parking_signal = 1;
                align_to_obstacle_requests = 0;
                break_requests = 0;
                time_parking = wb_robot_get_time;
            end
        end
        
        if parking_sensors_data(parking_sensors_idx(3)) > 30 &&...
                parking_sensors_data(parking_sensors_idx(3)) < 250 &&...
                parking_sensors_data(parking_sensors_idx(2)) > 300 &&...
                parking_sensors_data(parking_sensors_idx(2)) < 750 &&...
                parking_sensors_data(parking_sensors_idx(1)) > 750 &&...
                parking_sensors_data(parking_sensors_idx(1)) < 950 &&...
                sensors_data(sensors_idx(2)) > 8/9 * distance
               
            
            direction_vector = pioneer_get_direction(gps_destination, 10,...
                    velocity, wheel_motors, gps, compass, TIME_STEP);
            time_count = wb_robot_get_time;
            while wb_robot_step(TIME_STEP) ~= -1
                if wb_robot_get_time > time_count + parking_delay
                    break
                end
            end
            if norm(direction_vector) < 20
                pioneer_turn(180, turn_orientation2, 0, wheel_motors,...
                        compass, velocity, TIME_STEP);
                break_requests = 1;
                break_delay = 0;
                break_time_count = wb_robot_get_time;
                turn_requests = 0;
            end
        end
        
        if sensors_data(sensors_idx(3)) > 1000 && parking_signal == 1
            pioneer_turn(25, turn_orientation, 0, wheel_motors,...
                         compass, velocity, TIME_STEP)
        end
        
        if parking_signal == 1 &&...
                parking_sensors_data(parking_sensors_idx(1)) > 960 &&...
                parking_sensors_data(parking_sensors_idx(2)) > 700 &&...
                parking_sensors_data(parking_sensors_idx(2)) < 920 &&...
                parking_sensors_data(parking_sensors_idx(3)) > 960
            
            for i = 1 : numel(wheel_motors)
                wb_motor_set_velocity(wheel_motors(i), 0);
                is_parked = 1;
            end            
        end
        
        if parking_signal == 1 && strcmp(message, 'loading_complete') &&...
                is_parked == 1
            pioneer_turn(90, turn_orientation, 0, wheel_motors,...
                         compass, velocity, TIME_STEP)
            for i = 1 : numel(wheel_motors)
                wb_motor_set_velocity(wheel_motors(i), velocity);
            end
            time_count = wb_robot_get_time;
            while wb_robot_step(TIME_STEP) ~= 1
                if wb_robot_get_time > time_count + 3
                    break
                end
            end
            is_parked = 0;
            set_radio_requests = 1;
            break_requests = 1;
            break_delay = 0;
            break_time_count = wb_robot_get_time;
            turn_requests = 0;
        end
        
        if parking_signal == 1 && is_parked == 0 &&...
                wb_robot_get_time > time_parking + 20
            parking_signal = 0;
        end
        
        if move_along_deviation2 >= 160 && break_requests == 0
            pioneer_turn(90, turn_orientation, 0, wheel_motors,...
                    compass, velocity, TIME_STEP)
            break_requests = 1;
            break_delay = 3;
            break_time_count = wb_robot_get_time;
            turn_requests = 0;
        end
        
        if break_requests > 0 && wb_robot_get_time > break_time_count + break_delay
            for i = 1 : numel(wheel_motors)
                wb_motor_set_velocity(wheel_motors(i), velocity);
            end
            angle_deviation = (move_along_deviation -...
                move_along_deviation2) * orientation_ratio;
            break
        end
    end
   
end

