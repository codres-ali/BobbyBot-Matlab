function Disconnect(com_data)
   
if com_data.udp == 0
    clear com_data.client;
else
%     fclose(com_data.client);
    UdpClose(com_data.client);
end;

return