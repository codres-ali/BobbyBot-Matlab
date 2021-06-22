function com_data = UpdateTopic(com_data)

if com_data.type == 1
    msg.data = typecast(com_data.data_raw(1:4),'single');
    msg.stamp = typecast(com_data.data_raw(5:8),'single');
    msg.status = 1;
elseif com_data.type == 3
    msg.data_size = typecast(com_data.data_raw(1:4),'uint32');
    msg.data = typecast(com_data.data_raw(5:4*msg.data_size+4),'single');
    msg.stamp = typecast(com_data.data_raw(4*msg.data_size+5:4*msg.data_size+8),'single');
    msg.status = 1;
elseif com_data.type == 4
    msg.data_size = typecast(com_data.data_raw(1:4),'uint32');
    msg.data = typecast(com_data.data_raw(5:4*msg.data_size+4),'single');
    msg.stamp = typecast(com_data.data_raw(4*msg.data_size+5:4*msg.data_size+8),'single');
    msg.t_samling = typecast(com_data.data_raw(4*msg.data_size+9:4*msg.data_size+12),'single');
    msg.status = 1;
elseif com_data.type == 5
    msg.data_size = typecast(com_data.data_raw(1:4),'uint32');
    msg.distance = typecast(com_data.data_raw(5:4*msg.data_size+4),'single');
    msg.stamp = typecast(com_data.data_raw(4*msg.data_size+5:4*msg.data_size+8),'single');
    msg.angle_min = typecast(com_data.data_raw(4*msg.data_size+9:4*msg.data_size+12),'single');
    msg.angle_max = typecast(com_data.data_raw(4*msg.data_size+13:4*msg.data_size+16),'single');
    msg.status = 1;
end

com_data.msgs(com_data.name.str) = msg;


return