function path_f = filter_path_straight(path)

path_f(1,:) = path(1,:);

dx = path(2,1) - path(1,1);
dy = path(2,2) - path(1,2);

angle_current = atan2(dy,dx);

k = 2;

for i=2:size(path,1)-1
    dx = path(i+1,1) - path(i,1);
    dy = path(i+1,2) - path(i,2);

    angle = atan2(dy,dx);
    if(abs(angle_current-angle)>0.01)
        angle_current = angle;
        path_f(k,:) = path(i,:);
        k = k+1;
    end
end

path_f(k,:) = path(end,:);

return