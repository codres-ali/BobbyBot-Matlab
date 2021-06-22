function [ Fullpathname ] = Get_path()

[filename, Pathanme] = uigetfile({'*.txt'},'Browse');
Fullpathname = strcat(Pathanme,filename);

end

