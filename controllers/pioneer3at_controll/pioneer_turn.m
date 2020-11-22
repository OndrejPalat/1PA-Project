function pioneer_turn(turn_angle, orientation, wheel_motors, compass,...
            velocity, TIME_STEP)
  
  k = 0;  
  min_angle_interval = 0;
  max_angle_interval = 0;
  if string(orientation) == "CCW"
    k = -1;
    min_angle_interval = -5;
  else
    k = 1;
    max_angle_interval = 5;
  end
  compass_vector = wb_compass_get_values(compass);
  start_angle = acosd(compass_vector(1));
  if compass_vector(3) < 0
    start_angle = -start_angle;
  end
  if start_angle < 0
    start_angle = start_angle + 360;
  end
  end_angle = start_angle + k * turn_angle;
  if end_angle < 0
    end_angle = end_angle + 360;
  elseif end_angle > 360
    end_angle = end_angle - 360;
  end
  min_angle = end_angle + min_angle_interval;  
  if min_angle < 0
    min_angle = end_angle + 360;
  end
  max_angle = end_angle + max_angle_interval;
  if max_angle > 360
    max_angle = max_angle - 360;
  end
  if turn_angle ~= 0
    wb_motor_set_velocity(wheel_motors(1),k * velocity);
    wb_motor_set_velocity(wheel_motors(2),k * -velocity);
    wb_motor_set_velocity(wheel_motors(3),k * velocity);
    wb_motor_set_velocity(wheel_motors(4),k * -velocity);
  end
  
  while wb_robot_step(TIME_STEP) ~= -1
    compass_vector = wb_compass_get_values(compass);
    now_angle = acosd(compass_vector(1));
    if compass_vector(3) < 0
      now_angle = -now_angle;
    end
    if now_angle < 0
      now_angle = now_angle + 360;
    end
    if now_angle > min_angle & now_angle < max_angle
      for i = 1 : numel(wheel_motors)
        wb_motor_set_velocity(wheel_motors(i), velocity);
      end
      break
    end
    if min_angle > max_angle & (now_angle > min_angle | now_angle < max_angle)
      for i = 1 : numel(wheel_motors)
        wb_motor_set_velocity(wheel_motors(i), velocity);
      end        
      break
    end
  end    

end

