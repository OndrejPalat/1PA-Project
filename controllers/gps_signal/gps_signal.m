
%desktop;
%keyboard;

TIME_STEP = 64;

emitter = wb_robot_get_device('emitter');
receiver = wb_robot_get_device('receiver');
gps = wb_robot_get_device('gps');
wb_gps_enable(gps, TIME_STEP);
wb_receiver_enable(receiver, TIME_STEP);
wb_receiver_set_channel(receiver, 1);
wb_emitter_set_channel(emitter, 1);

precision = 100;
gps_data = zeros(precision, 3);
gps_position = zeros(1, 3);
message = '';
send_gps_requests = 0;
i = 0;


while wb_robot_step(TIME_STEP) ~= -1
  i = i + 1;
  gps_data(i, :) = wb_gps_get_values(gps);
  if i == precision
    gps_position = mean(gps_data);
    break
  end 
end


while wb_robot_step(TIME_STEP) ~= -1

  while wb_receiver_get_queue_length(receiver) > 0    
    message = wb_receiver_get_data(receiver, 'double');
    message = char(message')
    if strcmp(message, 'send_gps')
      send_gps_requests = 1;
    end
    wb_receiver_next_packet(receiver);
  end
  
  if send_gps_requests == 1
    wb_emitter_send(emitter, gps_position);
    send_gps_requests = 0;
  end
  
  drawnow;

end