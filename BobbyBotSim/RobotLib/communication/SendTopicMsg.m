function done = SendTopicMsg(com_data,topic,msg)

if ~isfield(msg, 'type')
    msg.type = 1;
end

if msg.type == 1
    msg1 = [uint8(254) uint8(238) uint8(msg.type) typecast(int32(size(uint8(topic),2)),'uint8') typecast(int32(8),'uint8')];
    msg2 = [uint8(topic) typecast(single(msg.data),'uint8') typecast(single(msg.stamp),'uint8')];
elseif msg.type == 3
        msg1 = [uint8(254) uint8(238) uint8(msg.type) typecast(int32(size(uint8(topic),2)),'uint8') typecast(int32(4*size(msg.data,2)+8),'uint8')];
        msg2 = [uint8(topic) typecast(uint32(size(msg.data,2)),'uint8') typecast(single(msg.data),'uint8') typecast(single(msg.stamp),'uint8')];
elseif msg.type == 4
        msg1 = [uint8(254) uint8(238) uint8(msg.type) typecast(int32(size(uint8(topic),2)),'uint8') typecast(int32(4*size(msg.data,2)+12),'uint8')];
        msg2 = [uint8(topic) typecast(uint32(size(msg.data,2)),'uint8') typecast(single(msg.data),'uint8') typecast(single(msg.stamp),'uint8') typecast(single(msg.t_sampling),'uint8')];
end
    
 checksum1 = sum(msg1(3:end));
 checksum2 = sum(msg2);

 msg_all = [msg1 typecast(int32(checksum1),'uint8') msg2 typecast(int32(checksum2),'uint8')];

if com_data.udp==0
    write(com_data.client,msg_all);
    done = true;
else
%     fwrite(com_data.client,msg_all);
    UdpSend(com_data.client,msg_all);
    done = true;
end

return