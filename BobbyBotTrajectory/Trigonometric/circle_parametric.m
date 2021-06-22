clearvars; 

t = 0:0.01:6.28;

R = 1;
a = 0.1;
freq = 4;

k=1;

for t = 0:0.01:6.28
    x1(k) = (R+a*cos(freq*t))*sin(t);
    y1(k) = (R+a*cos(freq*t))*cos(t);
    x2(k) = (R+a*cos(3.1415+freq*t))*sin(t);
    y2(k) = (R+a*cos(3.1415+freq*t))*cos(t);
    k=k+1;
end

k=1;
for s = 0:0.01:3.14
    x1s(k) = (R+a*cos(freq*s/R))*sin(s/(R+a*cos(freq*s/R)));
    y1s(k) = (R+a*cos(freq*s/R))*cos(s/(R+a*cos(freq*s/R)));
    k=k+1;
end
plot(x1,y1);
axis equal;
hold on;
plot(x2,y2);

figure(2);
plot(x1s,y1s);
axis equal;
