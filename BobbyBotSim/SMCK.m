function [wrc wlc vc wc vc_dot wc_dot vcf wcf traj x_e y_e theta_e s1 s2]=SMCK(xr,yr,thetar,vr,wr,vc,wc,vcf,wcf,traj,dt)
    x_d = traj(1);     
    y_d = traj(2);    
    theta_d = traj(3);
    v_d = traj(4);      
    w_d = traj(5);
    v_d_dot = traj(6); 
    w_d_dot = traj(7);
    traj = getTrajectory(traj,dt);
    
    % (6) Compute the robot velocities (linear and angular)
%     v_r_dot = (Robot.v - MyAlg.vr)/dt;
%     w_r_dot = (Robot.theta_dot - MyAlg.wr)/dt;
%     MyAlg.vr = Robot.v; 
%     MyAlg.wr = Robot.theta_dot;
    
%     v_r = Robot.v;
%     w_r = Robot.theta_dot;

    x_r_dot = vr*cos(thetar);
    y_r_dot = vr*sin(thetar);
        
    % (7) Generate the error signal of each state
    x_e = (xr-x_d)*cos(theta_d)+(yr-y_d)*sin(theta_d);
    y_e = -(xr-x_d)*sin(theta_d)+(yr-y_d)*cos(theta_d);
    theta_e = reduce_angle_fullCircle(thetar-theta_d);
    
    % (8) Define the derivatives
    x_e_dot = vr*cos(theta_e) + y_e*w_d - v_d;
    y_e_dot = vr*sin(theta_e) - x_e*w_d;
    theta_e_dot = wr - w_d;

    %Parametri buni model Mohamed
    k1 = 3;           k2 = 7;       k0 = 10;
    Q1 = 0.5;         Q2 = 5.5;
    P1 = 0.5;         P2 = 0.25;
    
%       if  y_e < 0 
%          k2 = - abs(k2);
%       elseif y_e > 0
%          k2 = abs(k2);
%       end;
      
    % (10) Compute sliding surface
    s1 = x_e_dot + k1*x_e;
    %s2 = y_e_dot + k2*y_e+k0*sign(y_e)*theta_e;
    s2 = theta_e_dot + k2*theta_e + k0*y_e;
    %s2 = theta_e_dot + k2*y_e + k0*sign(y_e)*abs(theta_e);

    % (11) Compute control signals
    T1 = vr*theta_e_dot*sin(theta_e)-w_d_dot*y_e-w_d*y_e_dot+v_d_dot-k1*x_e_dot;
    T2 = w_d_dot - k2*theta_e_dot - k0*y_e_dot;
    %T2 = w_d_dot - k2*y_e_dot - k0*sign(theta_e*y_e)*theta_e_dot;

    Rrnom = 0.1145;
    Rlnom = 0.1145;
    dRr = 0.06;
    global dRl fault

    vc_dot = (-Q1*s1-P1*saturatie(s1/0.15)+T1)/cos(theta_e);
    wc_dot = -Q2*s2-P2*saturatie(s2/0.15)+T2;
    
    vcf_dot = vc_dot*(Rlnom+dRl)/Rlnom;
    wcf_dot = wc_dot*(Rlnom+dRl)/Rlnom;
    
%     wc_dot=0;
%     wc = (-Q2*s2-P2*saturatie(s2/0.15) - k2*y_e_dot + w_d_dot*x_e+w_d*x_e_dot - vc_dot*sin(theta_e))/(vr*cos(theta_e)+k0*signum(y_e))+w_d;
%     wc = (-Q2*s2-P2*saturatie(s2/0.15) - k2*y_e_dot + w_d_dot*x_e+w_d*x_e_dot)/(vr*cos(theta_e)+k0*signum(y_e))+w_d;
    
    vc = vc+dt*vc_dot;
    wc = wc+dt*wc_dot;
    
    vcf = vcf+dt*vcf_dot;
    wcf = wcf+dt*wcf_dot;
    
    L = 0.245;
    Rr = 0.1145;
    Rl = 0.1145;

    wrc = (vc + L*wc)/Rr;
    if(fault)
%     wlc = (vc - L*wc)/0.0745;%(Rlnom+dRl);
    wlc = (vc - L*wc)/(Rlnom-dRl);
    else
    wlc = (vc - L*wc)/Rlnom;
    end;
    %wlc = (vc - L*wc)/Rlnom;
    
   return;
    
function traj = getTrajectory(traj, dt)
% This function generates trjectory where:
%   <dt>        (1x1) change time.
%   <traj>      (7,1) (x, y, theta, v, w, v_dot, w_dot)
%
% Implementation:   Mohamed Mustafa, University of Manchester
%                   February 2013
% Modifications:    ...
% -------------------------------------------------------------------------

% Extract information
x_old = traj(1);        y_old = traj(2);      theta_old = traj(3);
v_old = traj(4);        w_old = traj(5);
v_dot_old = traj(6);    w_dot_old = (7);

% Maximum values (robot specs)
v_max = 3;          % m/sec
w_max = 1;          % rad/sec
v_dot_max = 0.1;      % m/sec^2
w_dot_max = 0.066;      % rad/sec^2


global v_new w_new

% Trajectory specification (can be a function of time)
%v_new = 0.3;
%w_new = 0.2;

% compute acceleration and edit velocities (do not exceed max values)
v_dot_temp = (v_new - v_old)/dt;
if abs(v_dot_temp)>v_dot_max
    v_dot_temp = sign(v_dot_temp)*v_dot_max;
end
v_new1 = v_old + dt*v_dot_temp;
if abs(v_new1)>v_max
    v_new1 = sign(v_new1)*v_max;
end
v_dot_new = (v_new1 - v_old)/dt;

w_dot_temp = (w_new - w_old)/dt;
if abs(w_dot_temp)>w_dot_max
    w_dot_temp = sign(w_dot_temp)*w_dot_max;
end
w_new1 = w_old + dt*w_dot_temp;
if abs(w_new1)>w_max
    w_new1 = sign(w_new1)*w_max;
end
w_dot_new = (w_new1 - w_old)/dt;

% Compute the pose
theta_new = theta_old + dt*w_new1;
x_new = x_old + dt*v_new1*cos(theta_new);
y_new = y_old + dt*v_new1*sin(theta_new);

% put all together for the output
traj = [x_new; y_new; theta_new; v_new1; w_new1; v_dot_new; w_dot_new];
return


