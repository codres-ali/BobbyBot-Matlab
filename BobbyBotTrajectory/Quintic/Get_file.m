function [ xpoint, ypoint ] = Get_file(Fullpathname )

    temp_num=[];
    num=[];
    Numbers='0123456789-.';
    n=0;
    text = fileread(Fullpathname);
    for j=1:length(text)
       z=text(j);
       Not_number=strfind(Numbers,z);
       if isempty(Not_number)
           if n
               num = [num str2double(temp_num)];
               temp_num=[];
               n=0;
           end
       else
           temp_num=[temp_num z];
           n=1;
       end 
    end
    xpoint=[];
    ypoint=[];
    n=0;
    for j=2:length(num)
        if n
            ypoint=[ypoint num(j)];
            n=0;
        else
            xpoint=[xpoint num(j)];
            n=1;
        end
    end

end

