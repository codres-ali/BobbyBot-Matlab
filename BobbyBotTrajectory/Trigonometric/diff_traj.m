clearvars; 

syms t

r = 1;
a = 0.15;
n = 2;

v = 0.7;

x(t) = (r+a*cos(n*t))*sin(t);
y(t) = -(r+a*cos(n*t))*cos(t);

x_1 = diff(x);
x_2 = diff(x_1);
x_3 = diff(x_2);

y_1 = diff(y);
y_2 = diff(y_1);
y_3 = diff(y_2);

k = (x_1*y_2 - x_2*y_1)/(x_1^2+y_1^2)^(3/2);
k_dot = diff(k);

omega_r = (v+v*0.05*k)/0.05;
omega_l = (v-v*0.05*k)/0.05;

omega_r_h = (v*x_1*3^0.5/6-v*0.5*y_1)/0.05;
omega_l_h = (-v*x_1*3^0.5/6-v*0.5*y_1)/0.05;
omega_c_h = (v*y_1)/0.05;

s = (x_1^2+y_1^2)^0.5;
% s_int = int(s);
% 
% double(s_int(25))^0.5

% s_int = 0;
% for u=0:0.0001:1
%     s_int = s_int + (double(s(u))+double(s(u-0.0001)))/2*0.0001;
% end

figure(1);
ezplot(omega_r,[0 6.28 -20 20]);
hold on;
ezplot(omega_l,[0 6.28 -20 20]);

figure(2);
ezplot(omega_r_h,[0 6.28 -20 20]);
hold on;
ezplot(omega_l_h,[0 6.28 -20 20]);
ezplot(omega_c_h,[0 6.28 -20 20]);

figure(3);
ezplot(k,[0 2*pi]);
