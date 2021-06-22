clear all;

[ Fullpathname ] = Get_path();

[ xpoint, ypoint ] = Get_file(Fullpathname);

PathReal = [xpoint' ypoint'];

PathRealFS = filter_path_straight(PathReal);

    D = 0.4;

    for i=2:size(PathRealFS,1)-1
        dx1 = PathRealFS(i,1)-PathRealFS(i-1,1);
        dy1 = PathRealFS(i,2)-PathRealFS(i-1,2);
        dx2 = PathRealFS(i+1,1)-PathRealFS(i,1);
        dy2 = PathRealFS(i+1,2)-PathRealFS(i,2);
        
        angle1 = atan2(dy1,dx1);
        angle2 = atan2(dy2,dx2);
        
        length1 = (dx1^2+dy1^2)^0.5;
        length2 = (dx2^2+dy2^2)^0.5;
        
        min_length = min([length1 length2]);
        
        if min_length/2 < D
            d = min_length/2;
        else
            d = D;
        end
        
        P0(1) = PathRealFS(i,1) - d*cos(angle1);
        P0(2) = PathRealFS(i,2) - d*sin(angle1);
        P0(3) = angle1;
        P1(1) = PathRealFS(i,1) + d*cos(angle2);
        P1(2) = PathRealFS(i,2) + d*sin(angle2);
        P1(3) = angle2;
        
        curves(i-1) = CurveQuintic('P0',P0,'P1',P1,'eta',d*1.75);
    end


figure(1);

% plot(xpoint,ypoint,'*');
hold on;
plot(PathRealFS(:,1),PathRealFS(:,2),'*r');

for i=1:size(curves,2)
curves(i).plot_xy();
end

axis equal;
