clearvars; 

syms r a n t xi

x(t) = (r+a*cos(xi+n*t))*sin(t);
y(t) = -(r+a*cos(xi+n*t))*cos(t);
% x(t) = r*sin(t)+8*a*(cos(t))^4*sin(t)-8*a*(cos(t))^2*sin(t)+a*sin(t);
% y(t) = r*cos(t)+8*a*(cos(t))^5-8*a*(cos(t))^3+a*cos(t);

x_1 = diff(x);
x_2 = diff(x_1);
x_3 = diff(x_2);

y_1 = diff(y);
y_2 = diff(y_1);
y_3 = diff(y_2);

k = (x_1*y_2 - x_2*y_1)/(x_1^2+y_1^2)^(3/2);
k_dot = diff(k);

s = (x_1^2+y_1^2)^(1/2);

s_i = int(s);
