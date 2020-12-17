function [message] = send_message(emitter)
message = double('loading_complete')
wb_emitter_send(emitter, message)
end

