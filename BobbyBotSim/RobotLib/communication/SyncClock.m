function com_data = SyncClock(com_data,timeout)
    
exit = 0;

msg.data = com_data.time;
msg.stamp = 1000*com_data.time;


SendTopicMsg(com_data,'SyncClock',msg);
t2 = tic;
while exit==0
    com_data = SpinOnce(com_data);
    msg_time = GetTopicMsg(com_data,'SyncClock');
    
    com_data.topics
    
    if msg_time.status == 1
        com_data.t_err = toc(t2);
        com_data.t_offset = msg_time.data+com_data.t_err/2-toc(com_data.t_start);
        disp('Clock syncronised with server');
        d = ['Clock offset: ' num2str(com_data.t_offset) ',  error: ' num2str(com_data.t_err)];
        disp(d);
        exit=1;
    end;
    
    if toc(t2)>timeout
        exit=1;
        disp('Could not syncronise with the server');
    end;
    
    pause(0.01);
end;

return