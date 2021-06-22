clearvars; 

syms R R1 R2 t

R = 2;

x(t) = R*sin(t);
y(t) = R*cos(t);

x_1 = diff(x);
x_2 = diff(x_1);

y_1 = diff(y);
y_2 = diff(y_1);

k = (x_1*y_2 - x_2*y_1)/(x_1^2+y_1^2)^(3/2);
k_dot = diff(k);

s = (x_1^2+y_1^2)^0.5;

s_int = int(s);

