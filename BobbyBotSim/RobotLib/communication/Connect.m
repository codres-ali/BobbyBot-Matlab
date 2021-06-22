function com_data = Connect(ip,port,protocol)
   
com_data.udp=0;
if nargin == 3
    if strcmpi(protocol,'udp')
        com_data.udp = 1;
    end
end
com_data.t_start=tic;
com_data.t_offset=0;
com_data.t_err=0;
com_data.status = 0;
com_data.buffer = [];
com_data.indx = 1;
com_data.msgs = containers.Map();
com_data.time=0;
if com_data.udp == 0
    com_data.client = tcpclient(ip, port);
else
    com_data.client = UdpConnect(ip, port+1);
end

return