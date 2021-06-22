clearvars; 

syms u a0 a1 a2 a3 a4 a5 b0 b1 b2 b3 b4 b5

r = 1;
beta = 0.3;
n = 4;
v = 0.7;

t = 0:0.01:1;

sign = 1;

traj1_x = [];
traj1_y = [];
traj2_x = [];
traj2_y = [];
omega = [];
omega_dot = [];

for k=0:n-1
    ang1 = k*2*pi/n;
    ang2 = (k+1)*2*pi/n;
    
    p0 = [r*cos(ang1) r*sin(ang1)];
    p1 = [r*cos(ang2) r*sin(ang2)];
    
    [a,b] = get_quintic_params([p0(1) p0(2) ang1+pi/2-sign*beta],[p1(1) p1(2) ang2+pi/2+sign*beta]);
    
    sign = -sign;

    x(u) = a(1)+a(2)*u+a(3)*u^2+a(4)*u^3+a(5)*u^4+a(6)*u^5;
    y(u) = b(1)+b(2)*u+b(3)*u^2+b(4)*u^3+b(5)*u^4+b(6)*u^5;
    
    traj1_x = [traj1_x double(x(t))];
    traj1_y = [traj1_y double(y(t))];
end
for k=0:n-1
    ang1 = (k+1)*2*pi/n;
    ang2 = (k+2)*2*pi/n;
    
    p0 = [r*cos(ang1) r*sin(ang1)];
    p1 = [r*cos(ang2) r*sin(ang2)];
    
    [a,b] = get_quintic_params([p0(1) p0(2) ang1+pi/2-sign*beta],[p1(1) p1(2) ang2+pi/2+sign*beta]);
    
    sign = -sign;

    x(u) = a(1)+a(2)*u+a(3)*u^2+a(4)*u^3+a(5)*u^4+a(6)*u^5;
    y(u) = b(1)+b(2)*u+b(3)*u^2+b(4)*u^3+b(5)*u^4+b(6)*u^5;
    
    x_1 = diff(x);
    x_2 = diff(x_1);

    y_1 = diff(y);
    y_2 = diff(y_1);
    
    k = (x_1*y_2 - x_2*y_1)/(x_1^2+y_1^2)^(3/2);
    k_dot = diff(k);
    
    traj2_x = [traj2_x double(x(t))];
    traj2_y = [traj2_y double(y(t))];
    omega = [omega v*double(k(t))];
    omega_dot = [omega_dot v*double(k_dot(t))];
end

figure(1);
plot(traj1_x,traj1_y);
hold on;
axis equal;
plot(traj2_x,traj2_y);

figure(2);
plot(omega);
hold on;
plot(omega_dot);


% x_1 = diff(x);
% x_2 = diff(x_1);
% 
% y_1 = diff(y);
% y_2 = diff(y_1);
% 
% s = (x_1^2+y_1^2)^0.5;
% 
% s_int = int(s);


% plot(x,y);

