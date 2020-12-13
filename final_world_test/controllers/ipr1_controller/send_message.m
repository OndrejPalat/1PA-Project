function [message] = send_message(emitter)
message = wb_emitter_send(emitter, 'loading complete')
end

