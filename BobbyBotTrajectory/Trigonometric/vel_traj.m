clearvars; 

syms t

vi = 0.6;
vf = 0;
tf = 1;

b01 = vi;
b11 = ((vf+vi)/2-vi)/(tf/2)^2;

b02 = vf;
b12 = -b11;

v1(t) = b11*t^2 + b01;
v2(t) = b12*(t-tf)^2 + b02;

a_max = b11*tf

fplot(v1,[0 tf/2]);
hold on;
fplot(v2,[tf/2 tf]);
