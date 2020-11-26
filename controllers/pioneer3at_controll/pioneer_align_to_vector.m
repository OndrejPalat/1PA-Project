function pioneer_align_to_vector(direction_vector, compass,...
        wheel_motors, velocity, TIME_STEP)
    compass_vector = wb_compass_get_values(compass);
    vectors_angle = acosd((compass_vector * direction_vector') /...
                    (norm(compass_vector) * norm(direction_vector)));
    vectors_orientation = cross(direction_vector, compass_vector);
    if vectors_orientation(2) >= 0
      pioneer_turn(vectors_angle, "CW", 0.5, wheel_motors, compass,...
                    velocity, TIME_STEP);
    elseif vectors_orientation(2) < 0
      pioneer_turn(vectors_angle, "CCW", 0.5, wheel_motors, compass,...
                    velocity, TIME_STEP);
    end
end

