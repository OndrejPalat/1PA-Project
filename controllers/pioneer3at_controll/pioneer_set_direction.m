function direction_vector = pioneer_set_direction(gps_destination, precision,...
                            velocity, wheel_motors, gps, compass, TIME_STEP)

    gps_coordinates = zeros(precision, 3);
    for i = 1 : 4
      wb_motor_set_position(wheel_motors(i), inf);
      wb_motor_set_velocity(wheel_motors(i), 0);
    end
    i = 0;
    while wb_robot_step(TIME_STEP) ~= -1
      i = i + 1;
      gps_coordinates(i, :) = wb_gps_get_values(gps);  
      if i == precision
        break
      end      
    end
    gps_position = mean(gps_coordinates);
    gps_position(2) = 0;
    gps_destination(2) = 0;
    direction_vector = gps_destination - gps_position;
                         
end

