function com_data = SpinOnce(com_data)

    if com_data.udp == 0
        if com_data.client.BytesAvailable>0
            tmp_buf = uint8(read(com_data.client,com_data.client.BytesAvailable,'uint8'));
            com_data.buffer = [com_data.buffer tmp_buf];
        end
    else
        exit=0;
        while exit == 0
            tmp_buf = UdpReceive(com_data.client);
            if size(tmp_buf,1)>0
                com_data.buffer = [com_data.buffer tmp_buf'];
            else
                exit = 1;
            end
        end
        
    end
    
exit = 0;
    while exit==0
        if com_data.status == 0   % status == start
            if com_data.indx<=size(com_data.buffer,2)-1
                if com_data.buffer(com_data.indx)==254 && com_data.buffer(com_data.indx+1)==238
                    com_data.indx = com_data.indx + 2;
                    com_data.status = 1;
                else
                    com_data.indx = com_data.indx + 1;
                end
            else
                exit = 1;
            end
        end
        if com_data.status == 1   % status == type
            if com_data.indx<=size(com_data.buffer,2)
                com_data.type = com_data.buffer(com_data.indx);
                com_data.indx = com_data.indx + 1;
                com_data.status = 2;
            else
                exit = 1;
            end
        end
        if com_data.status == 2  % status == name_size
            if com_data.indx<=size(com_data.buffer,2)-3
                com_data.name_size = typecast(com_data.buffer(com_data.indx:com_data.indx+3),'int32');
                com_data.indx = com_data.indx + 4;
                com_data.status = 3;
            else
                exit = 1;
            end
        end
        if com_data.status == 3  % status == data_size
            if com_data.indx<=size(com_data.buffer,2)-3
                com_data.data_size = typecast(com_data.buffer(com_data.indx:com_data.indx+3),'int32');
                com_data.indx = com_data.indx + 4;
                com_data.status = 4;
            else
                exit = 1;
            end
        end
        if com_data.status == 4  % status == checksum1
            if com_data.indx<=size(com_data.buffer,2)-3
                msg_checksum = typecast(com_data.buffer(com_data.indx:com_data.indx+3),'int32');
                checksum = int32(com_data.type) + sum(typecast(com_data.name_size,'uint8')) + sum(typecast(com_data.data_size,'uint8'));
                com_data.indx = com_data.indx + 4;
                if msg_checksum==checksum
                    com_data.status = 5;
                else
                    com_data.status = 0;
                    disp('Message Lost');
                end
            else
                exit = 1;
            end
        end
        if com_data.status == 5  % status == name
            if com_data.indx<=size(com_data.buffer,2)-com_data.name_size+1
                com_data.name.str = char(com_data.buffer(com_data.indx:com_data.indx+com_data.name_size-1));
                com_data.indx = com_data.indx + com_data.name_size;
                com_data.status = 6;
            else
                exit = 1;
            end
        end
        if com_data.status == 6  % status == data
            if com_data.indx<=size(com_data.buffer,2)-com_data.data_size+1
                com_data.data_raw = com_data.buffer(com_data.indx:com_data.indx+com_data.data_size-1);
                com_data.indx = com_data.indx + com_data.data_size;
                com_data.status = 7;
            else
                exit = 1;
            end
        end
        if com_data.status == 7  % status == checksum2
            if com_data.indx<=size(com_data.buffer,2)-3
                msg_checksum = typecast(com_data.buffer(com_data.indx:com_data.indx+3),'int32');
                checksum = sum(int32(char(com_data.name.str)))+sum(int32(com_data.data_raw));
                com_data.indx = com_data.indx + 4;
                if msg_checksum==checksum
                    com_data = UpdateTopic(com_data);
                else
                    disp('Message Lost');
                end
                com_data.status = 0;
            else
                exit = 1;
            end
        end
    end

    com_data.buffer = com_data.buffer(com_data.indx:size(com_data.buffer,2));
    com_data.indx = 1;

return