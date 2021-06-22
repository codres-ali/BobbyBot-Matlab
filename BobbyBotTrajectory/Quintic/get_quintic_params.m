function [a,b] = get_quintic_params(P0,P1)

% syms u

eta1 = 2;
eta2 = 2;

a(1) = P0(1);
a(2) = eta1*cos(P0(3));
a(3) = 0;
a(4) = 10*(P1(1)-P0(1))-6*eta1*cos(P0(3))-4*eta2*cos(P1(3));
a(5) = -15*(P1(1)-P0(1))+8*eta1*cos(P0(3))+7*eta2*cos(P1(3));
a(6) = 6*(P1(1)-P0(1))-3*eta1*cos(P0(3))-3*eta2*cos(P1(3));

b(1) = P0(2);
b(2) = eta1*sin(P0(3));
b(3) = 0;
b(4) = 10*(P1(2)-P0(2))-6*eta1*sin(P0(3))-4*eta2*sin(P1(3));
b(5) = -15*(P1(2)-P0(2))+8*eta1*sin(P0(3))+7*eta2*sin(P1(3));
b(6) = 6*(P1(2)-P0(2))-3*eta1*sin(P0(3))-3*eta2*sin(P1(3));

end