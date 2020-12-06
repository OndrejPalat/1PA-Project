function angle_deviation = pioneer_react_to_obstacle(collision_CCW_angle,...
            turn_orientation_request, angle_deviation, wheel_motors, compass,... 
            velocity, gps_destination, gps, TIME_STEP)
    
    direction_obstacle_angle = NaN;
    if turn_orientation_request ~= "CW" && turn_orientation_request ~= "CCW"
        compass_vector = wb_compass_get_values(compass);
        direction_vector = pioneer_get_direction(gps_destination, 100, velocity,... 
                           wheel_motors, gps, compass, TIME_STEP);
        compass_vector(2) = 0;
        direction_vector(2) = 0;
        direction_angle = acosd(compass_vector * direction_vector' /...
                          norm(compass_vector) / norm(direction_vector));
        cross_product = cross(compass_vector, direction_vector);
        if cross_product(2) < 0
            direction_angle = - direction_angle;
        end
        direction_obstacle_angle = direction_angle + collision_CCW_angle;
        if direction_obstacle_angle < 0
            direction_obstacle_angle = direction_obstacle_angle + 360;
        elseif direction_obstacle_angle > 360
            direction_obstacle_angle = direction_obstacle_angle - 360;
        end
    end
    
    if direction_obstacle_angle > 90 && direction_obstacle_angle < 270 ||...
            turn_orientation_request == "CCW"
        turn_angle = 180 - collision_CCW_angle;
        angle_deviation = angle_deviation - turn_angle;
        pioneer_turn(turn_angle, "CCW", 0, wheel_motors,...
            compass, velocity, TIME_STEP)
    else
        turn_angle = collision_CCW_angle;
        angle_deviation = angle_deviation + turn_angle;
        pioneer_turn(turn_angle, "CW", 0, wheel_motors,...
            compass, velocity, TIME_STEP)
    end
    
end

